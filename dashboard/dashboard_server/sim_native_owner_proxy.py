"""Optional SIM owner metadata/control only. OFF never opens a socket."""
import hashlib
import json
import math
import os
from pathlib import Path
import re
import socket
import threading
import time

SCHEMA = 'ubts.sim.native_owner.v1'
SOCKET_PATH = '/run/ubts-sim/native-owner.sock'
TOTAL_NS = 750000000
MAX_RESPONSE = 65536
STATES = {'idle','starting','running','stopping','stopped','failed'}


def require(ok, reason):
    if not ok: raise ValueError(reason)


def token(value):
    return type(value) is str and re.fullmatch('[0-9a-f]{32}',value) is not None


def native_context(value):
    # JSON representation of EngineV8 (publisher_epoch, generation, map_frame).
    # This rendered-map context is independent of the control UUID/session.
    return (type(value) is list and len(value)==3 and token(value[0]) and
        type(value[1]) is int and 1<=value[1]<=9007199254740991 and
        type(value[2]) is str and 0<len(value[2])<=256 and
        all(ord(char)>=32 for char in value[2]))


def raw_now():
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)


def raw_domain():
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    require(re.fullmatch('[0-9a-f-]{36}',boot) is not None,'native_owner_boot_domain')
    return hashlib.sha256((boot+':CLOCK_MONOTONIC_RAW').encode()).hexdigest()


def _object(pairs):
    value={}
    for key,item in pairs:
        require(key not in value,'native_owner_duplicate_JSON_key');value[key]=item
    return value


def _finite_float(value):
    number=float(value)
    require(math.isfinite(number),'native_owner_nonfinite_JSON')
    return number


def _request(payload,deadline,clock):
    connection=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
    def remaining():
        value=(deadline-clock())/10**9
        require(value>0,'native_owner_RPC_expired');return value
    try:
        connection.settimeout(remaining());connection.connect(SOCKET_PATH)
        connection.settimeout(remaining())
        connection.sendall(json.dumps(payload,separators=(',',':'),allow_nan=False).encode()+b'\n')
        data=bytearray()
        while not data.endswith(b'\n'):
            connection.settimeout(remaining());block=connection.recv(min(4096,MAX_RESPONSE+1-len(data)))
            require(bool(block),'native_owner_RPC_truncated');data.extend(block)
            require(len(data)<=MAX_RESPONSE and b'\n' not in data[:-1],'native_owner_RPC_size_or_framing')
        return json.loads(data,object_pairs_hook=_object,parse_float=_finite_float,
            parse_constant=lambda _:(_ for _ in ()).throw(ValueError('native_owner_nonfinite_JSON')))
    finally:
        connection.close()


class NativeOwnerProxy:
    def __init__(self,*,enabled=False,request=None,clock=raw_now,domain=raw_domain):
        require(type(enabled) is bool,'typed_native_owner_enabled')
        self.enabled=enabled;self._request=request or _request;self._clock=clock;self._domain=domain
        self._slots=threading.BoundedSemaphore(2);self._control=threading.Lock()

    def _status(self,value,started):
        require(type(value) is dict and value.get('state') in STATES and
            all(type(value.get(k)) is bool for k in ('enabled','active','priority_active','source_context_fresh')) and
            token(value.get('control_epoch')) and token(value.get('source_session')) and
            (value.get('owner_id') is None or token(value['owner_id'])) and
            (value.get('epoch') is None or native_context(value['epoch'])) and
            type(value.get('progress')) is dict and type(value.get('resources')) is dict and
            (value.get('fault') is None or type(value['fault']) is str and len(value['fault'])<=256) and
            value.get('runtime_authority') is False and value.get('physical_outputs') is False,
            'native_owner_status_contract')
        stamp=value.get('status_generated_RAW_ns')
        domain=self._domain();now=self._clock()
        require(0<=now-started<TOTAL_NS,'native_owner_transport_expired')
        require(type(stamp) is int and 0<=now-stamp<10**9 and
                value.get('status_clock_domain')==domain,'native_owner_status_not_fresh_same_RAW_domain')
        original=value.get('source_context_updated_monotonic_raw')
        require(type(original) in (int,float) and math.isfinite(original) and original>=0,
                'native_owner_original_source_clock_required')
        if value['source_context_fresh']:
            require(0<=now/10**9-original<1,'native_owner_original_source_expired')
        # No stamp substitution: transport freshness and original source freshness
        # stay distinct, including idle/stopped clean owners with stale sources.
        return dict(value,ok=True)

    def _call(self,op,expected_control_epoch=None,source_session=None):
        if not self.enabled:return dict(ok=False,enabled=False,state='disabled',error='native_owner_disabled',active=False,priority_active=False)
        require(op in ('status','start','stop'),'native_owner_operation_not_allowed')
        payload=dict(schema=SCHEMA,op=op)
        if op!='status':
            require(token(expected_control_epoch) and token(source_session),'native_owner_epoch_session_required')
            payload.update(expected_control_epoch=expected_control_epoch,source_session=source_session)
        control=op!='status'
        if control and not self._control.acquire(False):return dict(ok=False,error='native_owner_transition_pending')
        permit=False
        try:
            permit=self._slots.acquire(False)
            if not permit:return dict(ok=False,error='native_owner_requests_busy')
            started=self._clock();reply=self._request(payload,started+TOTAL_NS,self._clock)
            require(type(reply) is dict and type(reply.get('ok')) is bool,'native_owner_RPC_contract')
            if not reply['ok']:
                require(type(reply.get('error')) is str and len(reply['error'])<=256,'native_owner_RPC_error_contract')
                return dict(ok=False,error=reply['error'])
            return self._status(reply.get('result'),started)
        except (OSError,ValueError,TypeError,OverflowError,RecursionError):
            # Unknown start/stop outcome is never retried or called stopped.
            return dict(ok=False,error='native_owner_unavailable_refresh_status')
        finally:
            if permit:self._slots.release()
            if control:self._control.release()

    def status(self):return self._call('status')
    def start(self,expected_control_epoch,source_session):return self._call('start',expected_control_epoch,source_session)
    def stop(self,expected_control_epoch,source_session):return self._call('stop',expected_control_epoch,source_session)
    def archive_blocked(self):
        if not self.enabled:return False
        status=self.status()
        return (status.get('ok') is not True or status.get('enabled') is not True or
            status['priority_active'] or status['active'] or status['state'] in ('starting','running','stopping'))

    def job(self):
        value=self.status()
        if value.get('ok') is not True:
            return dict(id='native-observer',kind='native-observer',active=False,processing=False,
                state='service_unavailable',reason='native_owner_unavailable',priority=0,epoch=None,source_session=None)
        return dict(id='native-observer',kind='native-observer',priority=0,
            epoch=value['control_epoch'],source_session=value['source_session'],state=value['state'],
            active=value['active'] or value['priority_active'],processing=value['priority_active'],
            progress=value['progress'],resources=value['resources'],error=value['fault'],runtime_authority=False)


class NativeObserverOperations:
    def __init__(self,original,owner):self.original=original;self.owner=owner
    def status(self,*args,**kwargs):
        value=self.original.status(*args,**kwargs)
        if not self.owner.enabled:return value
        return {**value,'jobs':[*value.get('jobs',[]),self.owner.job()]}
    def stop(self,job_id,expected_epoch=None):
        # Native job is dispatched explicitly by the authenticated HTTP frontdoor
        # with both epoch/session; never widen this old generic operation.
        return self.original.stop(job_id,expected_epoch)


def optional_archive_priority():
    if os.environ.get('SIM_NATIVE_OWNER_ENABLED','0')!='1':return None
    return NativeOwnerProxy(enabled=True).archive_blocked
