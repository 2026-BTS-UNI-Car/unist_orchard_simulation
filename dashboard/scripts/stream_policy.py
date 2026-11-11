"""Preview-only network budgets; source ROS messages and recordings are unchanged."""
import math
import asyncio
import json
import time

PROFILES = {
    "tiny": (160, 90, 2, 30, 3000),
    "slow": (320, 180, 4, 40, 30000),
    "balanced": (640, 360, 10, 55, 120000),
    "detail": (1280, 720, 15, 70, 400000),
}


def fit_size(width, height, max_width, max_height):
    ratio = min(1.0, max_width / width, max_height / height)
    return max(1, round(width * ratio)), max(1, round(height * ratio))


async def wait_for_ack(websocket, predicate, timeout=2.0):
    """One absolute deadline; obsolete/malformed ACKs never extend it."""
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise asyncio.TimeoutError()
        raw = await asyncio.wait_for(websocket.recv(), remaining)
        if not isinstance(raw, str):
            continue
        try:
            reply = json.loads(raw)
        except (TypeError, ValueError):
            continue
        if isinstance(reply, dict) and predicate(reply):
            return


async def pace_stream(interval, started):
    """Only the unused portion of a period remains after send/ACK latency."""
    delay = max(0.0, interval - (time.monotonic() - started))
    await asyncio.sleep(delay)
    return delay


class FramePacer:
    """Absolute frame cadence with strict spacing for the preceding packet.

    Timer overshoot does not accumulate every frame. A whole missed period is
    discarded, never repaid with an immediate catch-up burst. Byte-rate limits
    use actual send time rather than the ideal FPS deadline.
    """
    def __init__(self, *, clock=None, sleep=None):
        self.clock = clock or time.monotonic
        self.sleep = sleep or asyncio.sleep
        self.next_frame_at = 0.0
        self.last_sent_at = None
        self.last_bytes = 0
        self.fps = None

    async def wait(self, fps, bytes_per_second):
        fps = max(1, fps)
        if self.last_sent_at is None:
            self.fps = fps
            return 0.0
        if fps != self.fps:
            self.next_frame_at = self.last_sent_at + 1.0 / fps
            self.fps = fps
        byte_deadline = self.last_sent_at + self.last_bytes / max(1, bytes_per_second)
        delay = max(0.0, max(self.next_frame_at, byte_deadline) - self.clock())
        if delay > 0:
            await self.sleep(delay)
        return delay

    def sent(self, when, size):
        period = 1.0 / max(1, self.fps or 1)
        deadline = self.next_frame_at + period
        self.next_frame_at = deadline if deadline > when else when + period
        self.last_sent_at = when
        self.last_bytes = size


class StreamBudget:
    def __init__(self, profile="auto", local_camera_30fps=False):
        self.profile = profile if profile in {*PROFILES, "auto"} else "auto"
        self.level = 0
        self.fast_count = 0
        self.local_camera_30fps = local_camera_30fps
        self.slow_count = 0
        self.cooldown_until = 0.0
        self.ack_latency_ms = None

    @property
    def effective_profile(self):
        return ("tiny", "slow", "balanced", "detail")[self.level] if self.profile == "auto" else self.profile

    @property
    def values(self):
        name = self.effective_profile
        if name == "detail" and self.local_camera_30fps:
            # Explicit local simulation policy only. Internet/Jetson defaults
            # retain their original rate, quality and bandwidth limits.
            return (1280, 720, 30, 70, 5000000)
        return PROFILES[name]

    def acknowledge(self, elapsed):
        self.ack_latency_ms = round(elapsed * 1000, 2) if math.isfinite(elapsed) else None
        if self.profile != "auto":
            return
        period = 1.0 / self.values[2]
        now = time.monotonic()
        severe = not math.isfinite(elapsed) or elapsed > max(.4, 1.6 * period)
        moderate = math.isfinite(elapsed) and elapsed > max(.25, 1.6 * period)
        self.slow_count = self.slow_count + 1 if moderate else 0
        if severe or self.slow_count >= 3:
            self.level = max(0, self.level - 1)
            self.fast_count = 0
            self.slow_count = 0
            self.cooldown_until = now + 5.0
        elif elapsed < .8 * period and now >= self.cooldown_until:
            # A 150 ms WAN can sustain more than the tiny profile's 2 fps.
            # Compare ACK time with the current period, not a fixed LAN-only
            # threshold. One in-flight frame still bounds queues and latency.
            self.fast_count += 1
            if self.fast_count >= 15:
                self.level = min(3, self.level + 1)
                self.fast_count = 0
        else:
            self.fast_count = 0

    def delay(self, frame_bytes, fps):
        return max(1.0 / max(1, fps), frame_bytes / self.values[4])
