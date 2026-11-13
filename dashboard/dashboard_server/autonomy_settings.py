"""Hardware preference only; never a motor command or autonomous permit."""
import json
import math
from pathlib import Path
import threading


class AutonomySettings:
    def __init__(self, path):
        self.path = Path(path)
        self.lock = threading.Lock()
        self.percent = 100.
        try:
            self.percent = self.validate(json.loads(self.path.read_text())['speed_percent'])
        except (OSError, ValueError, KeyError, TypeError):
            pass

    @staticmethod
    def validate(value):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= 100:
            raise ValueError('invalid_autonomy_speed')
        return float(value)

    def status(self, live=None):
        from .field_preflight import report
        preparation=report(Path(__file__).resolve().parents[1]/'config/field_autonomy_preparation.json',live)
        return dict(ok=True, active=False, enabled=False, speed_percent=self.percent,
                    state='NOT_QUALIFIED', runtime_authority=False, motor_command=False,
                    reason='encoderless_autonomy_pending_physical_validation',
                    message='실차 자율주행 준비 중 · LiDAR·IMU 위치 추정과 제동 검증 필요',
                    speed_units='percent_of_configured_cruise', calibrated_speed_m_s=None,
                    preparation=preparation,
                    body_feedback=live.get('body_feedback') if isinstance(live,dict) else None)

    def set_speed(self, value):
        percent = self.validate(value)
        with self.lock:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_suffix('.tmp')
            temporary.write_text(json.dumps({'speed_percent':percent}), encoding='utf-8')
            temporary.replace(self.path)
            self.percent = percent
            return self.status()
