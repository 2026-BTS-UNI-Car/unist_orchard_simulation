"""Display clock correction only. Never changes sensor/playback timestamps."""
from datetime import datetime, timezone, timedelta
import json
import math
from pathlib import Path

START_CLOCK_SCHEMA = 'ubts.recording_start_clock.v1'
MAX_RECEIVER_AGE_S = 2.
MAX_TIME_ACCURACY_S = .05
CLOCK_SAMPLE_KEYS = ('schema', 'protocol', 'received_at', 'gps_time',
    'gps_time_valid_date', 'gps_time_valid_time', 'gps_time_fully_resolved',
    'gps_time_accuracy_s', 'gps_time_fraction_ns')
UNVERIFIED_REASONS = frozenset(('receiver_unavailable', 'invalid_host_capture',
    'invalid_receiver_receipt', 'receiver_receipt_future', 'receiver_stale',
    'unsupported_receiver_protocol', 'receiver_time_unresolved',
    'receiver_accuracy_unqualified', 'invalid_receiver_fraction',
    'invalid_receiver_utc', 'receiver_host_receipt_invalid',
    'start_capture_clock_discontinuity', 'receiver_offset_out_of_bounds'))


def clock_sample(sample):
    """Only time evidence; never copies position, velocity or arbitrary payload."""
    return {key: sample[key] for key in CLOCK_SAMPLE_KEYS if key in sample}


def _number(value):
    if type(value) not in (int, float):
        return None
    try:
        number = float(value)
        return number if math.isfinite(number) else None
    except (OverflowError, ValueError):
        return None


def _receiver_epoch(sample):
    if sample.get('schema') != 'orchard.gps.v1' or sample.get('protocol') != 'UBX_NAV_PVT':
        return None, 'unsupported_receiver_protocol'
    if any(sample.get(key) is not True for key in (
            'gps_time_valid_date', 'gps_time_valid_time', 'gps_time_fully_resolved')):
        return None, 'receiver_time_unresolved'
    accuracy = _number(sample.get('gps_time_accuracy_s'))
    if accuracy is None or not 0 <= accuracy <= MAX_TIME_ACCURACY_S:
        return None, 'receiver_accuracy_unqualified'
    fraction = sample.get('gps_time_fraction_ns')
    if type(fraction) is not int or not -1_000_000_000 <= fraction <= 1_000_000_000:
        return None, 'invalid_receiver_fraction'
    utc_text = sample.get('gps_time')
    try:
        if not isinstance(utc_text, str) or len(utc_text) > 64:
            raise ValueError('invalid UTC text')
        utc = datetime.fromisoformat(utc_text.replace('Z', '+00:00'))
        # NAV-PVT's nano is separate from its whole-second UTC calendar.
        if utc.tzinfo is None or utc.utcoffset() != timedelta(0) or utc.microsecond:
            raise ValueError('not whole-second UTC')
        return utc.timestamp()+fraction*1e-9, None
    except (ValueError, TypeError, OverflowError, OSError):
        return None, 'invalid_receiver_utc'


def clock_epoch(sample):
    """A repeated receiver UTC epoch cannot renew its time-evidence age."""
    return _receiver_epoch(sample)[0]


def start_clock_snapshot(started_at, sample, received_monotonic, *,
                         now_monotonic, host_now_s):
    """Freeze start-display evidence, not sensor clocks or playback time.

    NAV-PVT UTC validity is independent of a position fix. HPPOSLLH's copied
    time fields are deliberately not treated as a newly received time epoch.
    Receiver tAcc does not measure USB/ROS latency or prove host NTP sync.
    """
    result = dict(schema=START_CLOCK_SCHEMA, original_started_at=started_at,
        display_started_at=started_at, clock_quality='host_unverified',
        offset_s=0., reason='receiver_unavailable', evidence={})

    def denied(reason):
        result['reason'] = reason
        return result

    host = _number(host_now_s)
    current = _number(now_monotonic)
    started = timestamp(started_at)
    if host is None or current is None or current < 0 or started is None:
        return denied('invalid_host_capture')
    try:
        result['evidence']['host_snapshot_at'] = datetime.fromtimestamp(host, timezone.utc).isoformat()
    except (ValueError, OverflowError, OSError):
        return denied('invalid_host_capture')
    if abs(started-host) > 1:
        return denied('start_capture_clock_discontinuity')
    if not isinstance(sample, dict) or not sample:
        return denied('receiver_unavailable')
    receipt = _number(received_monotonic)
    if receipt is None or receipt < 0:
        return denied('invalid_receiver_receipt')
    age = current-receipt
    if age < 0:
        return denied('receiver_receipt_future')
    if age > MAX_RECEIVER_AGE_S:
        return denied('receiver_stale')
    receiver_epoch, error = _receiver_epoch(sample)
    if error:
        return denied(error)
    receiver_host_receipt = _number(sample.get('received_at'))
    if receiver_host_receipt is None or not 0 <= host-receiver_host_receipt <= MAX_RECEIVER_AGE_S:
        return denied('receiver_host_receipt_invalid')
    offset = receiver_epoch-receiver_host_receipt
    if not math.isfinite(offset) or abs(offset) > 86400:
        return denied('receiver_offset_out_of_bounds')
    try:
        corrected = datetime.fromtimestamp(started+offset, timezone.utc).isoformat()
    except (ValueError, OverflowError, OSError):
        return denied('invalid_receiver_utc')
    evidence = dict(clock_sample(sample), host_snapshot_at=result['evidence']['host_snapshot_at'],
        snapshot_monotonic_s=current, receiver_receipt_monotonic_s=receipt,
        receiver_age_s=age, display_estimate='receiver_utc_plus_host_receipt_elapsed',
        transport_latency_verified=False, host_clock_sync_verified=False)
    return dict(result, display_started_at=corrected,
        clock_quality='gnss_receiver_estimated', offset_s=offset,
        reason='receiver_utc_validated_transport_unmeasured', evidence=evidence)


def _start_display(metadata):
    value = metadata.get('clock_quality_snapshot')
    original = metadata.get('started_at')
    if value is None:
        return dict(display_started_at=original, clock_quality='host_clock', clock_offset_s=0)
    result = dict(display_started_at=original, clock_quality='host_unverified',
                  clock_offset_s=0, clock_reason='invalid_start_clock_snapshot')
    if (not isinstance(value, dict) or value.get('schema') != START_CLOCK_SCHEMA
            or value.get('original_started_at') != original):
        return result
    if value.get('clock_quality') == 'host_unverified':
        if (value.get('display_started_at') == original and value.get('offset_s') == 0
                and isinstance(value.get('reason'), str)
                and value['reason'] in UNVERIFIED_REASONS):
            result['clock_reason'] = value['reason']
        return result
    evidence = value.get('evidence')
    if value.get('clock_quality') != 'gnss_receiver_estimated' or not isinstance(evidence, dict):
        return result
    expected = start_clock_snapshot(original, clock_sample(evidence),
        evidence.get('receiver_receipt_monotonic_s'),
        now_monotonic=evidence.get('snapshot_monotonic_s'),
        host_now_s=timestamp(evidence.get('host_snapshot_at')))
    first, second = timestamp(value.get('display_started_at')), timestamp(expected['display_started_at'])
    offset = _number(value.get('offset_s'))
    if (expected['clock_quality'] != 'gnss_receiver_estimated' or offset is None
            or first is None or second is None or abs(first-second) > .01
            or abs(offset-expected['offset_s']) > .01):
        return result
    return dict(display_started_at=expected['display_started_at'],
        clock_quality='gnss_receiver_estimated', clock_offset_s=expected['offset_s'],
        clock_reason=expected['reason'], clock_evidence=expected['evidence'])


def timestamp(value):
    try:
        date = datetime.fromisoformat(str(value).replace('Z', '+00:00'))
        return date.timestamp() if date.tzinfo else None
    except (ValueError, TypeError, OverflowError, OSError):
        return None


def display_clock(path, metadata):
    original = metadata.get('started_at')
    result = _start_display(metadata)
    try:
        file = Path(path) / 'recording_clock.json'
        if not file.is_file() or file.is_symlink() or file.stat().st_size > 16384:
            return result
        value = json.loads(file.read_text(encoding='utf-8'))
        if not isinstance(value, dict):
            return result
        offset = value.get('offset_s')
        corrected = value.get('display_started_at')
        first, second = timestamp(original), timestamp(corrected)
        if (value.get('schema') != 'ubts.recording_clock.v1'
                or value.get('original_started_at') != original
                or isinstance(offset, bool) or not isinstance(offset, (float, int))
                or not math.isfinite(offset) or abs(offset) > 86400
                or first is None or second is None or abs(second-first-offset) > .01):
            return result
        return dict(display_started_at=corrected, clock_quality='gnss_estimated',
                    clock_offset_s=offset, clock_evidence=value.get('evidence', {}))
    except (OSError, ValueError, TypeError):
        return result


def sort_key(row):
    display = timestamp(row.get('display_started_at'))
    original = timestamp(row.get('started_at'))
    return (display if display is not None else original if original is not None else 0,
            str(row.get('id', '')))
