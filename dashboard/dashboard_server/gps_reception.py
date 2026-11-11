"""Bounded read-only u-blox reception diagnostics, independent of position fixes.

F9 HPG1.51/protocol27.50, NAV-SAT 3.15.16 and NAV-SIG 3.15.18:
https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.51_InterfaceDescription_UBXDOC-963802114-13124.pdf
No serial I/O, receiver configuration, antenna-power or fix inference.
"""
import math
import struct

RECEPTION_POLL_SECONDS = 5.0
RECEPTION_STALE_SECONDS = 12.0
WEEK_MS = 604800000
MESSAGES = {(1, 0x35): ('nav_sat', 1, 12, 255), (1, 0x43): ('nav_sig', 0, 16, 120)}
COUNTS = ('reported_count', 'detected_count', 'tracked_count', 'positive_cno_count')


def parse_reception(cls, msg_id, payload):
    """A listed/searching SV or signal is not necessarily detected or tracked."""
    key, version, width, maximum = MESSAGES[(cls, msg_id)]
    if (len(payload) < 8 or payload[4] != version or payload[5] > maximum
            or len(payload) != 8 + width * payload[5]):
        raise ValueError('invalid_reception_length_or_version')
    tow = struct.unpack_from('<I', payload)[0]
    if tow >= WEEK_MS: raise ValueError('invalid_reception_time_of_week')
    qualities, strengths, identities = [], [], set()
    for at in range(8, len(payload), width):
        if key == 'nav_sat':
            identity = tuple(payload[at:at+2])
            cno, quality = payload[at+2], struct.unpack_from('<I', payload, at+8)[0] & 7
        else:
            identity = tuple(payload[at:at+4])  # GNSS/SV/signal/frequency identity.
            cno, quality = payload[at+6], payload[at+7]
        if quality > 7 or identity in identities:
            raise ValueError('invalid_reception_quality_or_duplicate')
        identities.add(identity); qualities.append(quality); strengths.append(cno)
    return dict(protocol='UBX_NAV_SAT' if key == 'nav_sat' else 'UBX_NAV_SIG',
                i_tow_ms=tow, reported_count=len(qualities),
                detected_count=sum(q >= 2 for q in qualities),
                tracked_count=sum(q >= 4 for q in qualities),
                positive_cno_count=sum(c > 0 for c in strengths),
                max_cno_dbhz=max(strengths, default=None))


def reception_snapshot(reception, *, now, connected=True):
    """Recompute age from original report receipt, never a bridge heartbeat.

    Missing/invalid/stale counts are null, not zero. Each message has its own
    epoch and age; NAV-SAT and NAV-SIG are not assumed simultaneous with PVT.
    """
    result = {'schema': 'orchard.gps_reception.v1', 'stale_after_s': RECEPTION_STALE_SECONDS}
    for key in ('nav_sat', 'nav_sig'):
        raw = reception.get(key, {}) if isinstance(reception, dict) else {}
        raw = raw if isinstance(raw, dict) else {}
        receipt = raw.get('received_at')
        valid_time = (type(receipt) in (int, float) and math.isfinite(receipt)
                      and type(now) in (int, float) and math.isfinite(now) and receipt <= now)
        age = now - receipt if valid_time else None
        counts_valid = all(type(raw.get(k)) is int and 0 <= raw[k] <= (120 if key == 'nav_sig' else 255)
                           for k in COUNTS)
        counts_valid = (counts_valid and raw['tracked_count'] <= raw['detected_count'] <= raw['reported_count']
                        and raw['positive_cno_count'] <= raw['reported_count'])
        state = ('disconnected' if not connected else
                 'no_data' if not raw or (raw.get('state') == 'no_data' and receipt is None) else
                 'invalid' if raw.get('error') or not valid_time else
                 'stale' if age > RECEPTION_STALE_SECONDS else
                 'invalid' if not counts_valid else 'fresh')
        entry = dict(protocol=raw.get('protocol'), i_tow_ms=raw.get('i_tow_ms'),
                     received_at=receipt if valid_time else None,
                     age_s=round(age, 3) if age is not None else None, state=state)
        entry.update({k: raw.get(k) if state == 'fresh' else None for k in COUNTS})
        entry['max_cno_dbhz'] = raw.get('max_cno_dbhz') if state == 'fresh' else None
        if raw.get('error'): entry['error'] = str(raw['error'])[:96]
        result[key] = entry
    return result
