"""Linux RAW-clock freshness; wall dates are never expiry clocks.

CLOCK_MONOTONIC_RAW is not disciplined by adjtimex or time namespace offsets.
Boot identity plus clock source identifies comparable container timestamps,
and rejects persisted data from an earlier kernel boot.
"""
from functools import lru_cache
import hashlib
import math
from pathlib import Path
import time
import uuid

SCHEMA = 'linux-monotonic-raw-v1'
SOURCE = 'CLOCK_MONOTONIC_RAW'


def raw_now():
    try:
        value = time.clock_gettime(time.CLOCK_MONOTONIC_RAW)
        return value if math.isfinite(value) else None
    except (AttributeError, OSError, ValueError):
        return None


@lru_cache(maxsize=1)
def clock_domain():
    try:
        boot = str(uuid.UUID(Path('/proc/sys/kernel/random/boot_id').read_text().strip()))
        if raw_now() is None:
            return None
        return hashlib.sha256(f'{boot}:{SOURCE}'.encode()).hexdigest()
    except (OSError, ValueError):
        return None


def metadata():
    return {'clock_schema': SCHEMA, 'clock_source': SOURCE, 'clock_domain': clock_domain(),
            'updated_monotonic_raw': raw_now()}


def freshness(payload, ttl, now=None):
    """Bounded JSON-safe diagnostic; legacy/malformed/future data fails closed."""
    payload = payload if isinstance(payload, dict) else {}
    now = raw_now() if now is None else now
    stamp = payload.get('updated_monotonic_raw')
    local = clock_domain()
    result = {'clock': SOURCE, 'schema': payload.get('clock_schema')[:48] if isinstance(payload.get('clock_schema'), str) else None,
              'clock_domain_matches': bool(local and payload.get('clock_domain') == local),
              'age_s': None, 'ttl_s': ttl, 'fresh': False, 'reason': 'clock_metadata_missing'}
    if payload.get('clock_schema') != SCHEMA or payload.get('clock_source') != SOURCE:
        return result
    if not result['clock_domain_matches']:
        result['reason'] = 'clock_domain_mismatch'
        return result
    if not all(isinstance(v, (int,float)) and not isinstance(v, bool) and math.isfinite(v) for v in (stamp,now)):
        result['reason'] = 'invalid_monotonic_timestamp'
        return result
    age = now-stamp
    if not math.isfinite(age):
        result['reason'] = 'invalid_monotonic_timestamp'
        return result
    result['age_s'] = age
    result['fresh'] = 0 <= age < ttl
    result['reason'] = 'fresh' if result['fresh'] else 'future_monotonic_timestamp' if age < 0 else 'expired'
    return result
