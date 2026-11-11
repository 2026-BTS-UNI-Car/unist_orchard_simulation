"""Read cache and explicit RPC actions; never create a second map owner."""
import json
import socket
import threading
import time
from types import SimpleNamespace

from .native_mapping_service import SCHEMA


class NativeMappingProxy:
    def __init__(self,path):
        self.path=str(path);self.lock=threading.Lock();self.cached=None;self.cached_at=0.
        self.navigation=SimpleNamespace(snapshot=self.navigation_status)

    def navigation_status(self):
        try:return self.rpc('navigation')
        except (OSError,ValueError):return dict(state='HOLD',reason='mapping_service_unavailable',
            runtime_authority=False,motor_command_sent=False,applied_linear_mps=0.,
            applied_yaw_rate_rad_s=0.,proposed_linear_mps=0.,proposed_yaw_rate_rad_s=0.)

    def rpc(self,op,*args,**kwargs):
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as connection:
            connection.settimeout(5.)
            connection.connect(self.path)
            connection.sendall(json.dumps(dict(schema=SCHEMA,op=op,args=args,kwargs=kwargs),allow_nan=False).encode()+b'\n')
            with connection.makefile('rb') as stream:line=stream.readline(8*1024*1024+1)
        if not line.endswith(b'\n') or len(line)>8*1024*1024:raise ValueError('mapping_rpc_response_size')
        value=json.loads(line)
        if value.get('ok') is not True:raise ValueError('mapping_rpc_'+str(value.get('error')))
        return value['result']

    def snapshot(self,since=None,transport=None,tile_since=None,tile_epoch=None):
        with self.lock:
            try:
                if self.cached is None or time.monotonic()-self.cached_at>=.025:
                    fresh=self.rpc('snapshot',transport='tiles',
                        tile_epoch=self.cached.get('tile_epoch') if self.cached else None,
                        tile_since=self.cached.get('tile_version') if self.cached else None)
                    if self.cached and not fresh['tiles_reset'] and fresh['tile_epoch']==self.cached['tile_epoch']:
                        tiles={(t['x'],t['y']):t for t in self.cached['tiles']}
                        tiles.update({(t['x'],t['y']):t for t in fresh['tiles']})
                        fresh['tiles']=list(tiles.values())
                    self.cached=fresh;self.cached_at=time.monotonic()
                # Completed JSON generations are immutable. Copying every RLE
                # cell for every client monopolized the web interpreter.
                value=dict(self.cached)
            except (OSError,ValueError) as exc:
                if self.cached is None:return dict(ok=False,error='mapping_service_unavailable')
                value=dict(self.cached)
                value.update(active=False,state='service_unavailable',error='mapping_service_unavailable')
                value['navigation']=dict(state='HOLD',reason='mapping_service_unavailable',
                    runtime_authority=False,motor_command_sent=False,applied_linear_mps=0.,
                    applied_yaw_rate_rad_s=0.,proposed_linear_mps=0.,proposed_yaw_rate_rad_s=0.)
            try:revision=int(tile_since) if tile_epoch==value['tile_epoch'] else -1
            except (ValueError,TypeError):revision=-1
            full=not 0<=revision<=value['tile_version']
            if not full:value['tiles']=[t for t in value['tiles'] if t['revision']>revision]
            return dict(value,tiles_reset=full,mapper_process_isolated=True)

    def action(self,*args,**kwargs):
        value=self.rpc('action',*args,**kwargs)
        with self.lock:self.cached=None;self.cached_at=0.
        return value

    def task_status(self):
        try:return self.rpc('task_status')
        except (OSError,ValueError):return dict(id='live',kind='live',active=False,
            state='service_unavailable',reason='mapping_service_unavailable',runtime_authority=False)
    def begin_archive(self,*args,**kwargs):return self.rpc('begin_archive',*args,**kwargs)
    def finish_archive(self,*args,**kwargs):return self.rpc('finish_archive',*args,**kwargs)
    def archive_snapshot(self,*args,**kwargs):return self.rpc('archive_snapshot',*args,**kwargs)
    def regenerate(self,*args,**kwargs):return self.rpc('regenerate',*args,**kwargs)
    def jobs(self,*args,**kwargs):return self.rpc('jobs',*args,**kwargs)
    def stop_job(self,*args,**kwargs):return self.rpc('stop_job',*args,**kwargs)
    def close(self):pass  # The explicit service, not a web client, owns lifetime.


class RemoteMappingOperations:
    def __init__(self,proxy):self.proxy=proxy
    def status(self,*args,**kwargs):
        value=self.proxy.rpc('operations',*args,**kwargs)
        value['resources']['scope']='native_mapper_process'
        return value
    def stop(self,*args,**kwargs):return self.proxy.rpc('stop_operation',*args,**kwargs)
