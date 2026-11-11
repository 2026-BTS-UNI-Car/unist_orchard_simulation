"""Camera preview policy only; source capture, archives and other sensors are untouched."""
import math
import time

from scripts.stream_policy import PROFILES


class CameraStreamBudget:
    """Preserve Auto detail before cadence, with one ACK-serviced frame at a time.

    ACK service rate includes RTT and browser work; it is not link bandwidth.
    Explicit Tiny/Slow are retained for an intentional emergency tradeoff.
    """
    TIERS = ((640, 360), (960, 540), (1280, 720))
    MAX_BYTES_S = 4_000_000
    QUALITY = 70

    def __init__(self, profile="auto", local_camera_30fps=False, *, clock=None):
        self.profile = profile if profile in {*PROFILES, "auto"} else "auto"
        self.level = 0
        self.clock = clock or time.monotonic
        self.ceiling = 5_000_000 if local_camera_30fps else self.MAX_BYTES_S
        self.ack_latency_ms = None
        self.service_s = None
        self.service_bytes = None
        self.fast_count = self.slow_count = 0
        self.next_upgrade_at = self.clock() + 3.0
        self.delivery_state = "starting"
        self.last_decode_status = "pending"

    @property
    def effective_profile(self):
        return ("readable", "clear", "detail")[self.level] if self.profile == "auto" else self.profile

    @property
    def values(self):
        if self.profile in ("tiny", "slow"):
            return PROFILES[self.profile]
        width, height = self.TIERS[self.level] if self.profile == "auto" else ((640, 360) if self.profile == "balanced" else (1280, 720))
        fps, byte_rate = 30., float(self.ceiling)
        if self.service_s is not None:
            # A fraction of observed service capacity reserves headroom. Both
            # limits describe the same interval; FramePacer takes their max.
            # FramePacer's timer floor is 1 FPS. Slower updates remain bounded
            # by byte spacing; report the actual timer ceiling, not a lower
            # fractional value the shared timer does not enforce.
            fps = max(1., min(fps, .85 / self.service_s))
            byte_rate = min(byte_rate, .85 * self.service_bytes / self.service_s)
        return width, height, fps, self.QUALITY, max(1., byte_rate)

    def acknowledge(self, elapsed, frame_bytes, *, decode_status="unknown"):
        self.last_decode_status = decode_status
        valid = (isinstance(elapsed, (int, float)) and not isinstance(elapsed, bool)
                 and math.isfinite(elapsed) and elapsed > 0
                 and isinstance(frame_bytes, int) and not isinstance(frame_bytes, bool) and frame_bytes > 0)
        if not valid:
            self.delivery_state = "invalid_feedback"
            self.fast_count = 0
            return
        self.ack_latency_ms = round(elapsed * 1000, 2)
        alpha = .25
        self.service_s = elapsed if self.service_s is None else (1-alpha)*self.service_s + alpha*elapsed
        self.service_bytes = frame_bytes if self.service_bytes is None else (1-alpha)*self.service_bytes + alpha*frame_bytes
        self.delivery_state = "bandwidth_limited" if self.service_s > .35 else "live"
        if decode_status != "decoded":
            self.delivery_state = "decode_" + decode_status
            self.fast_count = 0
            return
        if self.profile != "auto":
            return
        now = self.clock()
        self.slow_count = self.slow_count + 1 if elapsed > .45 else 0
        if elapsed > 1.0 or self.slow_count >= 3:
            self.level = max(0, self.level-1)
            self.fast_count = self.slow_count = 0
            self.next_upgrade_at = now + 5.0
        elif elapsed < .22 and self.service_s < .22 and now >= self.next_upgrade_at:
            self.fast_count += 1
            if self.fast_count >= 10:
                self.level = min(len(self.TIERS)-1, self.level+1)
                self.fast_count = 0
                self.next_upgrade_at = now + 5.0
        else:
            self.fast_count = 0

    def status(self):
        width, height, fps, quality, rate = self.values
        return dict(camera_policy="readability_first_v1", effective_profile=self.effective_profile,
                    output_limit_width=width, output_limit_height=height, preview_quality=quality,
                    effective_fps_limit=round(fps, 2), budget_bytes_s=round(rate),
                    ack_latency_ms=self.ack_latency_ms, delivery_state=self.delivery_state,
                    decode_status=self.last_decode_status,
                    automatic_quality_floor=self.profile == "auto")
