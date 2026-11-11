"""Checked NMEA/UBX decoding. No serial I/O or receiver configuration."""
from __future__ import annotations

import copy
import hashlib
from datetime import datetime, timezone
import math
import struct
import time
from .gps_reception import MESSAGES, WEEK_MS, parse_reception, reception_snapshot
from .gps_utc import decorate_receiver_time

GPS_SCHEMA = "orchard.gps.v1"
GPS_TIMELINE_SCHEMA = "orchard.gps_timeline.v1"
FIX_NAMES = {0: "no_fix", 1: "autonomous", 2: "dgps", 4: "rtk_fixed", 5: "rtk_float", 6: "dead_reckoning"}


def finite(value):
    if isinstance(value, bool): return None
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except (TypeError, ValueError):
        return None


def valid_position(sample):
    lat, lon = finite(sample.get("latitude")), finite(sample.get("longitude"))
    return bool(sample.get("valid") is True and lat is not None and lon is not None
                and -90 <= lat <= 90 and -180 <= lon <= 180)


def nmea_coordinate(value, hemisphere):
    number = finite(value)
    if number is None or number < 0 or hemisphere not in ("N", "S", "E", "W"):
        return None
    degrees = int(number // 100)
    minutes = number - degrees * 100
    result = degrees + minutes / 60
    limit = 90 if hemisphere in ("N", "S") else 180
    if minutes >= 60 or result > limit:
        return None
    return -result if hemisphere in ("S", "W") else result


def ubx_packet(cls, msg_id, payload=b""):
    body = bytes((cls, msg_id)) + struct.pack("<H", len(payload)) + payload
    a = b = 0
    for byte in body:
        a = (a + byte) & 255
        b = (b + a) & 255
    return b"\xb5\x62" + body + bytes((a, b))


class GnssDecoder:
    def __init__(self, clock=time.time):
        self.clock = clock
        self.buffer = bytearray()
        self.packets = 0
        self.rejected = 0
        self.last_packet_at = None
        self.last_sample_at = None
        self.last_gga_at = None
        self.last_ubx_at = None
        self.sample = {}
        self.motion = {}
        self.gst = {}
        self.identity = {}
        self.sequence = 0
        self.reception = {}
        self.reception_epochs = {}

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while self.buffer:
            if self.buffer[0] == 36:
                end = self.buffer.find(b"\n")
                if end < 0:
                    if len(self.buffer) > 1024:
                        del self.buffer[0]; self.rejected += 1; continue
                    break
                raw = bytes(self.buffer[:end+1]).strip()
                del self.buffer[:end+1]
                try:
                    body, check = raw[1:].rsplit(b"*", 1)
                    checksum = 0
                    for value in body: checksum ^= value
                    if len(check) != 2 or checksum != int(check, 16): raise ValueError("checksum")
                    fields = body.decode("ascii").split(",")
                    if len(fields[0]) != 5: raise ValueError("sentence")
                    frame = self._nmea(fields)
                except (ValueError, IndexError, UnicodeError, OverflowError):
                    self.rejected += 1; continue
            elif self.buffer[0] == 181:
                if len(self.buffer) < 6: break
                if self.buffer[1] != 98: del self.buffer[0]; continue
                length = struct.unpack_from("<H", self.buffer, 4)[0]
                if length > 4096: del self.buffer[0]; self.rejected += 1; continue
                if len(self.buffer) < length + 8: break
                raw = bytes(self.buffer[:length+8])
                if raw != ubx_packet(raw[2], raw[3], raw[6:-2]):
                    del self.buffer[0]; self.rejected += 1; continue
                del self.buffer[:length+8]
                try:
                    frame = self._ubx(raw[2], raw[3], raw[6:-2])
                    if frame is not None and raw[2:4]==b'\x01\x07' and len(raw)==100:
                        frame.update(receiver_pvt_frame_hex=raw.hex(),receiver_pvt_frame_sha256=hashlib.sha256(raw).hexdigest(),
                            receiver_pvt_payload_hex=raw[6:-2].hex(),receiver_pvt_raw_origin='checked_decoder_input_frame_not_hardware_certificate')
                        frame['receiver_time_evidence']=decorate_receiver_time(frame)
                except (ValueError, IndexError, struct.error):
                    self.rejected += 1; continue
            else:
                del self.buffer[0]
                continue
            self.packets += 1
            self.last_packet_at = self.clock()
            if frame is not None:
                self.sequence += 1
                frame.update(schema=GPS_SCHEMA, sequence=self.sequence, received_at=self.clock(), frame_id="gps_link")
                frame['reception'] = reception_snapshot(self.reception, now=frame['received_at'])
                if frame.get('fix_dimension') == 2:
                    frame.update(altitude_ellipsoid_m=None, altitude_msl_m=None, v_acc_m=None)
                if not valid_position(frame):
                    frame.update(valid=False, latitude=None, longitude=None)
                self.sample = frame
                self.last_sample_at = self.clock()
                frames.append(copy.deepcopy(frame))
        return frames

    def _base(self, protocol, quality):
        return dict(protocol=protocol, valid=quality in (1, 2, 4, 5), quality=quality,
                    fix_type=FIX_NAMES.get(quality, "unknown"), latitude=None, longitude=None,
                    altitude_msl_m=None, altitude_ellipsoid_m=None, satellites=None, hdop=None,
                    h_acc_m=None, v_acc_m=None, accuracy_source=None, speed_mps=None,
                    course_deg=None, gps_time=None, speed_accuracy_m_s=None,
                    course_accuracy_deg=None, motion_accuracy_source=None,
                    velocity_ned_m_s=None)

    def _nmea(self, f):
        kind = f[0][-3:]
        now = self.clock()
        if kind == "RMC":
            if len(f) < 10: raise ValueError("short RMC")
            if f[2] == 'A' and (f[4] not in ('N', 'S') or f[6] not in ('E', 'W')): raise ValueError('hemisphere')
            speed = finite(f[7])
            course = finite(f[8])
            gps_time = None
            try:
                if f[9] and f[1]:
                    gps_time = datetime.strptime(f[9] + f[1].split('.')[0], "%d%m%y%H%M%S").replace(tzinfo=timezone.utc).isoformat()
            except ValueError: pass
            self.motion = dict(at=now, epoch=f[1], speed_mps=speed * .514444444 if speed is not None and speed >= 0 else None,
                               course_deg=course if course is not None and 0 <= course < 360 else None, gps_time=gps_time)
            if self.last_ubx_at is not None and now - self.last_ubx_at < 2: return None
            # GGA supplies the actual RTK quality; RMC is a fallback position source.
            if f[2] == "A" and self.last_gga_at is not None and now - self.last_gga_at < 2: return None
            mode = f[12] if len(f) > 12 else ""
            q = {"D": 2, "R": 4, "F": 5, "E": 6}.get(mode, 1) if f[2] == "A" else 0
            frame = self._base("NMEA_RMC", q)
            frame.update(latitude=nmea_coordinate(f[3], f[4]), longitude=nmea_coordinate(f[5], f[6]))
            frame.update({key: self.motion[key] for key in ("speed_mps", "course_deg", "gps_time")})
            return frame
        if kind == "GST":
            if len(f) < 9: raise ValueError("short GST")
            lat, lon, alt = finite(f[6]), finite(f[7]), finite(f[8])
            self.gst = dict(epoch=f[1], at=now, h_acc_m=math.hypot(lat, lon) if lat is not None and lon is not None and min(lat, lon) >= 0 else None,
                            v_acc_m=alt if alt is not None and alt >= 0 else None)
            if self.sample.get('protocol') == 'NMEA_GGA' and self.sample.get('nmea_utc') == f[1] and now - (self.last_gga_at or 0) < 2:
                return dict(self.sample, h_acc_m=self.gst['h_acc_m'], v_acc_m=self.gst['v_acc_m'], accuracy_source='NMEA_GST')
            return None
        if kind != "GGA": return None
        if len(f) < 12: raise ValueError("short GGA")
        self.last_gga_at = now
        if self.last_ubx_at is not None and now - self.last_ubx_at < 2: return None
        quality = int(f[6] or 0)
        if quality and (f[3] not in ('N', 'S') or f[5] not in ('E', 'W')): raise ValueError('hemisphere')
        frame = self._base("NMEA_GGA", quality)
        alt, geoid = finite(f[9]), finite(f[11])
        frame.update(latitude=nmea_coordinate(f[2], f[3]), longitude=nmea_coordinate(f[4], f[5]),
                     satellites=int(f[7] or 0), hdop=finite(f[8]), altitude_msl_m=alt,
                     altitude_ellipsoid_m=alt + geoid if alt is not None and geoid is not None else None,
                     nmea_utc=f[1])
        if now - self.motion.get("at", -100) <= 2:
            frame.update({key: self.motion[key] for key in ("speed_mps", "course_deg", "gps_time")})
        if now - self.gst.get("at", -100) <= 2 and self.gst.get("epoch") == f[1]:
            frame.update(h_acc_m=self.gst["h_acc_m"], v_acc_m=self.gst["v_acc_m"], accuracy_source="NMEA_GST")
        return frame

    def _ubx(self, cls, msg_id, p):
        if (cls, msg_id) in MESSAGES:
            key = MESSAGES[(cls, msg_id)][0]
            try:
                report = parse_reception(cls, msg_id, p)
                previous = self.reception_epochs.get(key)
                if previous is not None:
                    delta = (report['i_tow_ms'] - previous) % WEEK_MS
                    # Polling the same navigation epoch must not renew its age.
                    if delta == 0: return None
                    if delta >= WEEK_MS // 2: raise ValueError('reordered_reception_epoch')
            except ValueError as error:
                self.reception[key] = dict(error=str(error), received_at=self.clock())
                raise
            self.reception_epochs[key] = report['i_tow_ms']
            self.reception[key] = dict(report, received_at=self.clock())
            return None  # Reception information is never a new position/fix sample.
        if (cls, msg_id) == (10, 4) and len(p) >= 40:
            self.identity = dict(software=p[:30].rstrip(b"\0").decode("ascii", "replace"),
                                 hardware=p[30:40].rstrip(b"\0").decode("ascii", "replace"),
                                 extensions=[p[i:i+30].rstrip(b"\0").decode("ascii", "replace") for i in range(40, len(p), 30)])
        if (cls, msg_id) == (1, 20) and len(p) >= 36:
            if self.last_ubx_at is None or self.clock() - self.last_ubx_at >= 2: return None
            if p[3] & 1 or self.sample.get("i_tow_ms") != struct.unpack_from("<I", p, 4)[0]: return None
            frame = dict(self.sample)
            lon, lat, height, msl = struct.unpack_from("<iiii", p, 8)
            lon_hp, lat_hp, height_hp, msl_hp = struct.unpack_from("<bbbb", p, 24)
            h_acc, v_acc = struct.unpack_from("<II", p, 28)
            frame.update(protocol="UBX_NAV_HPPOSLLH", latitude=lat*1e-7+lat_hp*1e-9, longitude=lon*1e-7+lon_hp*1e-9,
                         altitude_ellipsoid_m=height*.001+height_hp*.0001, altitude_msl_m=msl*.001+msl_hp*.0001,
                         h_acc_m=h_acc*.0001, v_acc_m=v_acc*.0001, accuracy_source="UBX_NAV_HPPOSLLH")
            return frame
        if (cls, msg_id) != (1, 7) or len(p) < 92: return None
        self.last_ubx_at = self.clock()
        valid = bool(p[21] & 1) and p[20] in (2, 3) and not (p[78] & 1)
        carrier = (p[21] >> 6) & 3
        quality = (4 if carrier == 2 else 5 if carrier == 1 else 2 if p[21] & 2 else 1) if valid else 0
        frame = self._base("UBX_NAV_PVT", quality)
        lon, lat, height, msl, h_acc, v_acc = struct.unpack_from("<iiiiII", p, 24)
        vel_n,vel_e,vel_d,speed,heading = struct.unpack_from("<iiiii", p, 48)
        speed_acc,heading_acc = struct.unpack_from("<II", p, 68)
        gps_time = None
        try:
            if p[11] & 3 == 3:
                gps_time = datetime(struct.unpack_from("<H", p, 4)[0], *p[6:11], tzinfo=timezone.utc).isoformat()
        except ValueError: pass
        frame.update(i_tow_ms=struct.unpack_from("<I", p)[0], latitude=lat*1e-7, longitude=lon*1e-7,
                     altitude_ellipsoid_m=height*.001, altitude_msl_m=msl*.001,
                     h_acc_m=h_acc*.001, v_acc_m=v_acc*.001, accuracy_source="UBX_NAV_PVT",
                     satellites=p[23], pdop=struct.unpack_from("<H", p, 76)[0]*.01,
                     speed_mps=max(0, speed*.001), course_deg=(heading*1e-5) % 360,
                     gps_time=gps_time, fix_dimension=p[20],
                     speed_accuracy_m_s=speed_acc*.001 if valid else None,
                     course_accuracy_deg=heading_acc*1e-5 if valid else None,
                     motion_accuracy_source='UBX_NAV_PVT' if valid else None,
                     velocity_ned_m_s=dict(north=vel_n*.001,east=vel_e*.001,
                         down=vel_d*.001 if p[20]==3 else None) if valid else None,
                     gps_time_valid_date=bool(p[11]&1),gps_time_valid_time=bool(p[11]&2),
                     gps_time_fully_resolved=bool(p[11]&4),
                     gps_time_accuracy_s=struct.unpack_from('<I',p,12)[0]*1e-9,
                     gps_time_fraction_ns=struct.unpack_from('<i',p,16)[0])
        frame.update(gps_time_calendar_utc=dict(year=struct.unpack_from('<H',p,4)[0],month=p[6],day=p[7],hour=p[8],minute=p[9],second=p[10]),
                     gps_time_valid_bits=p[11],gps_time_accuracy_ns=struct.unpack_from('<I',p,12)[0],gps_time_origin_protocol='UBX_NAV_PVT')
        if len(p)==92:
            frame.update(receiver_pvt_payload_hex=bytes(p).hex(),receiver_pvt_raw_origin='decoder_payload_argument_only_checksum_unverified')
        frame['receiver_time_evidence']=decorate_receiver_time(frame)
        return frame


def fresh_snapshot(sample, received_at, *, now=None, driver=None, stale_s=3.0, reception=None):
    now = time.time() if now is None else now
    received_at = finite(received_at)
    age = max(0, now - received_at) if received_at is not None and received_at <= now + 1 else None
    fresh = age is not None and age <= stale_s and (driver or {}).get('serial_open') is not False
    payload = copy.deepcopy(sample or {})
    payload.update(schema=GPS_SCHEMA, type="gps", updated_at=now, age_s=round(age, 3) if age is not None else None,
                   connected=fresh, valid=fresh and valid_position(payload))
    payload["state"] = "fix" if payload["valid"] else "no_fix" if fresh else "stale" if sample and received_at is not None else "no_data"
    payload["driver"] = dict(driver or {})
    payload['reception'] = reception_snapshot(reception if reception is not None else payload.get('reception'),
        now=now, connected=(payload['driver'].get('serial_open') is not False
                           and payload['driver'].get('topic_alive') is not False))
    return payload
