"""Bounded local client for a sensor-only native observation owner.

No constructor/GET starts work. The endpoint has no motor/device commands.
Only explicit start/stop mutate this separate source owner's lifecycle.
"""
import json
import socket
import time


class NativeObservationClient:
    def __init__(self, socket_path):
        if type(socket_path) is not str or not socket_path.startswith('/') or len(socket_path.encode())>100:
            raise ValueError('bounded_absolute_native_observation_socket_required')
        self.socket_path=socket_path

    def _request(self, operation, *, cursor=None, lease=None, map_epoch=None):
        try:
            with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as client:
                deadline=time.monotonic()+(3.5 if operation=='stop' else .6)
                client.settimeout(max(.001,deadline-time.monotonic()))
                client.connect(self.socket_path)
                value={'op':operation}
                if cursor is not None:
                    if operation!='map_tiles' or type(cursor) is not str or len(cursor)>67:
                        raise ValueError('native_map_cursor_required')
                    value['cursor']=cursor
                if lease is not None:
                    if operation!='map_release' or type(lease) is not str or len(lease)!=32:
                        raise ValueError('native_map_lease_required')
                    value['lease']=lease
                if map_epoch is not None:
                    if operation!='map_new' or type(map_epoch) is not str or len(map_epoch)!=32:
                        raise ValueError('native_map_epoch_required')
                    value['map_epoch']=map_epoch
                client.sendall((json.dumps(value)+'\n').encode())
                parts=[];size=0
                while True:
                    remaining=deadline-time.monotonic()
                    if remaining<=0:raise TimeoutError('native_observation_reply_deadline')
                    client.settimeout(remaining)
                    raw=client.recv(min(65536,262145-size))
                    if not raw:raise ValueError('native_observation_reply_incomplete')
                    parts.append(raw);size+=len(raw)
                    if size>262144:raise ValueError('native_observation_reply_capacity')
                    if b'\n' in raw:break
                data=b''.join(parts)
                if data.count(b'\n')!=1 or not data.endswith(b'\n'):
                    raise ValueError('native_observation_reply_shape')
                result=json.loads(data,parse_constant=lambda _:(_ for _ in ()).throw(ValueError('nonfinite_native_reply')))
                if type(result) is not dict or result.get('runtime_authority') is not False or result.get('motor_command_sent') is not False:
                    raise ValueError('native_observation_reply_contract')
                return result
        except (OSError,ValueError,OverflowError):
            return dict(ok=False,state='unavailable',error='native_observation_source_unavailable',
                input=None,runtime_authority=False,motor_command_sent=False,autonomy_output_enabled=False)

    def start(self):return self._request('start')
    def stop(self):return self._request('stop')
    def status(self):return self._request('status')
    def map_status(self):return self._request('map_status')
    def map_tiles(self, cursor=None):return self._request('map_tiles',cursor=cursor)
    def map_release(self, lease):return self._request('map_release',lease=lease)
    def map_new(self, map_epoch):return self._request('map_new',map_epoch=map_epoch)
    def input(self):
        result=self._request('input')
        return result.get('input') if result.get('ok') is True else None

    def control_comparison(self):
        """Read-only scalar post-plan check; unavailable is never fresh."""
        def unique(items):
            value={}
            for key,item in items:
                if key in value:
                    raise ValueError('duplicate_native_comparison_key')
                value[key]=item
            return value
        def ns(value):
            return type(value) is int and 0<=value<=2**63-1
        def name(value):
            return type(value) is str and 0<len(value)<=128 and value.strip()==value
        try:
            with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as client:
                deadline=time.monotonic()+.6
                client.settimeout(max(.001,deadline-time.monotonic()))
                client.connect(self.socket_path)
                client.sendall(b'{"op":"control_comparison"}\n')
                parts=[];size=0
                while True:
                    remaining=deadline-time.monotonic()
                    if remaining<=0:
                        raise TimeoutError('native_comparison_reply_deadline')
                    client.settimeout(remaining)
                    raw=client.recv(min(4096,4097-size))
                    if not raw:
                        raise ValueError('native_comparison_reply_incomplete')
                    parts.append(raw);size+=len(raw)
                    if size>4096:
                        raise ValueError('native_comparison_reply_capacity')
                    if b'\n' in raw:
                        break
                data=b''.join(parts)
                if data.count(b'\n')!=1 or not data.endswith(b'\n'):
                    raise ValueError('native_comparison_reply_shape')
                result=json.loads(data,object_pairs_hook=unique,
                    parse_constant=lambda _:(_ for _ in ()).throw(ValueError('nonfinite_native_reply')))
            if (type(result) is not dict or result.get('ok') is not True or
                    set(result)!={'ok','schema','state','comparison','runtime_authority','motor_command_sent','autonomy_output_enabled'} or
                    result.get('schema')!='ubts.native_observation.local_reply.v1' or not name(result.get('state')) or
                    result.get('runtime_authority') is not False or
                    result.get('motor_command_sent') is not False or
                    result.get('autonomy_output_enabled') is not False):
                return None
            value=result.get('comparison')
            if (type(value) is not dict or
                    set(value)!={'schema','context','scan','imu','motion','source_now_ns','receipt_now_ns'} or
                    value.get('schema')!='ubts.native_observation.control_comparison.v1' or
                    not all(ns(value.get(k)) for k in ('source_now_ns','receipt_now_ns'))):
                return None
            context=value.get('context');required={'source_session','generation','source_clock_domain','receipt_clock_domain'}
            if (type(context) is not dict or not required<=set(context)<=required|{'epoch','reconnect_id'} or
                    not ns(context['generation']) or not all(name(context[k]) for k in set(context)-{'generation'})):
                return None
            for key,fields in (('scan',{'sequence','frame_id','stamp_ns','receipt_ns'}),
                    ('imu',{'stamp_ns','receipt_ns'}),
                    ('motion',{'previous_scan_stamp_ns','current_scan_stamp_ns','receipt_ns'})):
                part=value.get(key)
                if type(part) is not dict or set(part)!=fields or not all(ns(part[k]) for k in fields-{'frame_id'}):
                    return None
            return value if name(value['scan']['frame_id']) else None
        except (OSError,ValueError,TypeError,OverflowError,RecursionError):
            return None
