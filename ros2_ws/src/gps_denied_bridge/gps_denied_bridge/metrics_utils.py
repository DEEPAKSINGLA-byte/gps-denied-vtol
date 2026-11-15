"""Small helpers shared by the receivers and the metrics monitor."""

import collections
import math
import time


class RateTracker:
    """Message rate (Hz), inter-arrival jitter (ms) and staleness (s)."""

    def __init__(self, window=50):
        self.stamps = collections.deque(maxlen=window)
        self.total = 0

    def tick(self, t=None):
        self.stamps.append(time.monotonic() if t is None else t)
        self.total += 1

    def rate_hz(self):
        if len(self.stamps) < 2:
            return 0.0
        span = self.stamps[-1] - self.stamps[0]
        # A stalled stream must decay to 0 rather than keep its last value.
        if time.monotonic() - self.stamps[-1] > 2.0:
            return 0.0
        return (len(self.stamps) - 1) / span if span > 0 else 0.0

    def jitter_ms(self):
        if len(self.stamps) < 3:
            return 0.0
        s = list(self.stamps)
        dts = [b - a for a, b in zip(s, s[1:])]
        mean = sum(dts) / len(dts)
        return math.sqrt(sum((d - mean) ** 2 for d in dts) / len(dts)) * 1e3

    def age_s(self):
        if not self.stamps:
            return float('inf')
        return time.monotonic() - self.stamps[-1]


class LatencyTracker:
    """One-way latency from a sender wall-clock timestamp.

    ``raw_ms``  = receive_time - send_time. Includes any clock offset between
                  the Pi and the laptop, so it needs chrony/NTP to be absolute.
    ``excess_ms`` = raw minus the minimum over a sliding window. This cancels a
                  constant clock offset and the fixed path delay, leaving the
                  queueing/Wi-Fi jitter component, which is clock-independent.
    """

    def __init__(self, window_s=10.0, offset_s=0.0):
        self.window_s = window_s
        self.offset_s = offset_s
        self.hist = collections.deque()  # (monotonic, raw_ms)
        self.raw_ms = 0.0
        self.excess_ms = 0.0
        self.avg_ms = 0.0

    def update(self, sent_wall_time):
        now_m = time.monotonic()
        self.raw_ms = (time.time() - sent_wall_time - self.offset_s) * 1e3
        self.hist.append((now_m, self.raw_ms))
        while self.hist and now_m - self.hist[0][0] > self.window_s:
            self.hist.popleft()
        vals = [v for _, v in self.hist]
        self.excess_ms = self.raw_ms - min(vals)
        self.avg_ms = sum(vals) / len(vals)
        return self.raw_ms
