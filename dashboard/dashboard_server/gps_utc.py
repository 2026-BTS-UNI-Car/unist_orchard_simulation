"""Pure reported NAV-PVT UTC carrier; never host-clock/pose association."""
from datetime import datetime,timezone
import math,re,hashlib,struct
UTC_ROLE='host_ROS_publication_not_receiver_measurement'
WHOLE=re.compile(r'^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:Z|\+00:00)$')
def require(v,reason):
    if not v:raise ValueError(reason)
def raw_pvt(sample):
    frame=sample.get('receiver_pvt_frame_hex');payload=sample.get('receiver_pvt_payload_hex')
    if frame is not None:
        require(type(frame) is str and len(frame)==200 and all(c in '0123456789abcdef' for c in frame),'bounded_original_PVT_frame_required')
        raw=bytes.fromhex(frame);require(raw[:6]==b'\xb5\x62\x01\x07\x5c\x00','original_PVT_frame_layout')
        a=b=0
        for byte in raw[2:-2]:a=(a+byte)&255;b=(b+a)&255
        require(raw[-2:]==bytes((a,b)),'original_PVT_checksum_invalid')
        require(sample.get('receiver_pvt_frame_sha256')==hashlib.sha256(raw).hexdigest(),'original_PVT_frame_digest_invalid')
        p=raw[6:-2];require(payload==p.hex(),'original_PVT_frame_payload_mismatch');kind='independently_verified_full_input_frame'
    elif payload is not None:
        require(type(payload) is str and len(payload)==184 and all(c in '0123456789abcdef' for c in payload),'bounded_PVT_payload_argument_required')
        p=bytes.fromhex(payload);kind='payload_argument_only_checksum_unverified'
    else:return None,'decoded_fields_only_raw_unavailable'
    expected=dict(year=struct.unpack_from('<H',p,4)[0],month=p[6],day=p[7],hour=p[8],minute=p[9],second=p[10])
    require(sample.get('gps_time_calendar_utc')==expected and sample.get('gps_time_valid_bits')==p[11] and
        sample.get('gps_time_accuracy_ns')==struct.unpack_from('<I',p,12)[0] and sample.get('gps_time_fraction_ns')==struct.unpack_from('<i',p,16)[0] and
        sample.get('i_tow_ms')==struct.unpack_from('<I',p,0)[0],'raw_PVT_decoded_time_mismatch')
    require(all(sample.get(k) is bool(p[11]&mask) for k,mask in (('gps_time_valid_date',1),('gps_time_valid_time',2),('gps_time_fully_resolved',4))),'raw_PVT_validity_mismatch')
    return p,kind
def receiver_utc(sample):
    require(type(sample) is dict and sample.get('schema') in (None,'orchard.gps.v1') and sample.get('protocol')=='UBX_NAV_PVT','NAV_PVT_measurement_epoch_required')
    payload,kind=raw_pvt(sample)
    require(all(sample.get(k) is True for k in ('gps_time_valid_date','gps_time_valid_time','gps_time_fully_resolved')),'receiver_UTC_unresolved')
    fraction=sample.get('gps_time_fraction_ns');require(type(fraction) is int and -10**9<=fraction<=10**9,'invalid_signed_receiver_nano')
    tow=sample.get('i_tow_ms');require(type(tow) is int and 0<=tow<604800000,'invalid_receiver_iTow')
    calendar=sample.get('gps_time_calendar_utc')
    if calendar is not None:
        require(type(calendar) is dict and set(calendar)=={'year','month','day','hour','minute','second'} and all(type(v) is int for v in calendar.values()),'invalid_raw_UTC_calendar')
        require(calendar['second']!=60,'unsupported_UTC_leap_second')
    text=sample.get('gps_time');require(type(text) is str and len(text)<=32 and WHOLE.fullmatch(text),'whole_second_explicit_UTC_required')
    try:dt=datetime.fromisoformat(text.replace('Z','+00:00'))
    except ValueError:raise ValueError('invalid_receiver_UTC_date') from None
    require(1980<=dt.year<=2261,'receiver_UTC_date_out_of_range')
    if calendar is not None:require(tuple(calendar[k] for k in ('year','month','day','hour','minute','second'))==(dt.year,dt.month,dt.day,dt.hour,dt.minute,dt.second),'raw_UTC_calendar_mismatch')
    bits=sample.get('gps_time_valid_bits')
    if bits is not None:require(type(bits) is int and 0<=bits<=255 and bits&7==7,'raw_UTC_valid_bits_mismatch')
    acc=sample.get('gps_time_accuracy_s');require(type(acc) in (int,float) and math.isfinite(acc) and 0<=acc<=.05,'receiver_tAcc_unqualified')
    value=acc*10**9;tacc=int(round(value));require(abs(value-tacc)<=1e-4,'receiver_tAcc_not_integer_ns')
    raw_acc=sample.get('gps_time_accuracy_ns')
    if raw_acc is not None:require(type(raw_acc) is int and 0<=raw_acc<=0xffffffff and raw_acc==tacc,'raw_receiver_tAcc_mismatch')
    delta=dt-datetime(1970,1,1,tzinfo=timezone.utc);utc_ns=(delta.days*86400+delta.seconds)*10**9+fraction
    require(0<=utc_ns<(1<<63),'receiver_UTC_ns_out_of_range')
    return dict(receiver_utc_ns=utc_ns,receiver_i_tow_ms=tow,time_accuracy_ns=tacc,calendar_second=text,signed_nano_ns=fraction,
        time_accuracy_origin='raw_NAV_PVT_uint32' if raw_acc is not None else 'decoded_float_quantized_integer_ns',
        raw_time_evidence_kind=kind,raw_checksum_independently_verified=kind=='independently_verified_full_input_frame',
        raw_frame_sha256=sample.get('receiver_pvt_frame_sha256') if kind=='independently_verified_full_input_frame' else None,
        measurement_clock_domain='receiver_UTC',host_measurement_time_ns=None,scan_IMU_time_association_verified=False,
        source_certificate_qualified=False,clock_sync_verified=False,physical_outputs=False)
def decorate_receiver_time(sample):
    try:e=receiver_utc(sample);return dict(e,schema='ubts.receiver_UTC_carrier.v1',accepted_reported_UTC=True,reason=None,receiver_utc_ns=str(e['receiver_utc_ns']))
    except ValueError as error:return dict(schema='ubts.receiver_UTC_carrier.v1',accepted_reported_UTC=False,reason=str(error),receiver_utc_ns=None,
        measurement_clock_domain='receiver_UTC',host_measurement_time_ns=None,scan_IMU_time_association_verified=False,source_certificate_qualified=False,clock_sync_verified=False,physical_outputs=False)
def mark_publication(sample,sec,nanosec):
    require(type(sample) is dict and type(sec) is int and 0<=sec<(1<<31) and type(nanosec) is int and 0<=nanosec<10**9,'original_ROS_publication_stamp_required')
    sample['stamp']=dict(sec=sec,nanosec=nanosec);sample['stamp_role']=UTC_ROLE
