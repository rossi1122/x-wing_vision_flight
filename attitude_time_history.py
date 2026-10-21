"""Receive-time angle history. No assumption that receive time equals exposure time."""
from collections import deque
import math
import threading

class AngleHistory:
    def __init__(self, size=600):
        self.samples = deque(maxlen=size)
        self.lock = threading.Lock()

    def append(self, timestamp, angles):
        if not all(math.isfinite(float(v)) for v in angles):
            return
        with self.lock:
            self.samples.append((float(timestamp), tuple(float(v) for v in angles)))

    def at(self, timestamp, max_gap):
        with self.lock:
            values = list(self.samples)
        if not values:
            return None
        for a,b in zip(values, values[1:]):
            if a[0] <= timestamp <= b[0] and b[0] > a[0]:
                if max(timestamp-a[0], b[0]-timestamp) > max_gap:
                    return None
                weight = (timestamp-a[0])/(b[0]-a[0])
                angles = tuple(x+weight*((y-x+180)%360-180) for x,y in zip(a[1],b[1]))
                return {'angles':angles, 'method':'interpolated',
                        'distance_s':max(timestamp-a[0],b[0]-timestamp)}
        nearest = min(values,key=lambda x:abs(x[0]-timestamp))
        delta=abs(nearest[0]-timestamp)
        if delta > max_gap:
            return None
        return {'angles':nearest[1], 'method':'nearest', 'distance_s':delta}
