"""Small SIM-only camera receipt heartbeat; no image transport or control code."""
import json
import math
import os
from pathlib import Path
import threading
import uuid

from scripts import sim_clock

SCHEMA='ubts-sim-camera-health-v1'
TTL_S=2.
STREAMS=('camera','camera_info')


def finite(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)


def counter(value):
    return isinstance(value,int) and not isinstance(value,bool) and 0<=value<=2**63-1


class CameraHealthProducer:
    def __init__(self, clock=sim_clock.raw_now):
        self.clock=clock;self.instance=uuid.uuid4().hex;self.lock=threading.Lock()
        self.streams={key:{'count':0,'received_monotonic_raw':None,'stamp_ns':None,'frame_id':''} for key in STREAMS}

    def observe(self, stream, stamp_ns, frame_id, received_raw):
        with self.lock:
            self.streams[stream]={'count':self.streams[stream]['count']+1,
                'received_monotonic_raw':received_raw,'stamp_ns':stamp_ns,'frame_id':frame_id}

    def snapshot(self):
        with self.lock:
            return {'schema':SCHEMA,'simulated':True,'instance_id':self.instance,
                    **sim_clock.metadata(),'updated_monotonic_raw':self.clock(),
                    'streams':{key:dict(value) for key,value in self.streams.items()}}


def simulation_producer():
    return CameraHealthProducer() if os.environ.get('DASHBOARD_MODE')=='simulation' else None


class CameraHealthReader:
    """Polling never refreshes sensor receipts; all ages use the producer RAW clock."""
    def __init__(self, clock=sim_clock.raw_now):
        self.clock=clock;self.previous=None

    @staticmethod
    def invalid(reason):
        return {'valid':False,'reason':reason,'instance_id':None,
                **{key:{'alive':False,'count':0,'age_s':None} for key in STREAMS}}

    def read(self, path='/runtime/camera_bridge_status.json'):
        try:
            with Path(path).open('rb') as stream:raw=stream.read(16385)
            if len(raw)>16384:return self.invalid('health_file_too_large')
            return self.evaluate(json.loads(raw))
        except (OSError,ValueError,TypeError,UnicodeError):
            return self.invalid('health_file_unavailable_or_malformed')

    def evaluate(self, payload):
        if not isinstance(payload,dict):return self.invalid('health_payload_invalid')
        if (payload.get('image_topic')!='/camera/image_raw' or
                payload.get('camera_info_topic')!='/camera/camera_info'):
            return self.invalid('health_topic_mismatch')
        health=payload.get('sim_camera_health')
        if not isinstance(health,dict) or health.get('schema')!=SCHEMA or health.get('simulated') is not True:
            return self.invalid('health_schema_invalid')
        instance=health.get('instance_id')
        try:
            if not isinstance(instance,str) or uuid.UUID(instance).hex!=instance:raise ValueError()
        except (ValueError,AttributeError):return self.invalid('health_instance_invalid')
        now=self.clock();fresh=sim_clock.freshness(health,TTL_S,now=now)
        if not fresh['fresh']:return self.invalid(fresh['reason'])
        produced=health['updated_monotonic_raw'];streams=health.get('streams')
        if not isinstance(streams,dict) or set(streams)!=set(STREAMS):return self.invalid('health_streams_invalid')
        previous=self.previous
        if previous and produced<previous['produced']:return self.invalid('health_producer_regression')
        result={'valid':True,'reason':'fresh','instance_id':instance}
        saved={}
        for key in STREAMS:
            value=streams[key]
            if not isinstance(value,dict):return self.invalid('health_stream_invalid')
            count=value.get('count');receipt=value.get('received_monotonic_raw')
            stamp=value.get('stamp_ns');frame=value.get('frame_id')
            if not counter(count):return self.invalid('health_count_invalid')
            if count==0:
                if receipt is not None or stamp is not None or frame!='':return self.invalid('health_empty_stream_invalid')
                row={'alive':False,'count':0,'age_s':None}
            else:
                if (not finite(receipt) or receipt<0 or receipt>produced or not counter(stamp)
                        or not isinstance(frame,str) or not 0<len(frame)<=128):
                    return self.invalid('health_receipt_invalid')
                status=sim_clock.freshness({**health,'updated_monotonic_raw':receipt},TTL_S,now=now)
                row={'alive':status['fresh'],'count':count,'age_s':status['age_s']}
            if previous and previous['instance']==instance:
                old=previous['streams'][key]
                if count<old['count'] or (count==old['count'] and value!=old):
                    return self.invalid('health_counter_regression_or_rewritten_receipt')
                if count>old['count'] and old['count'] and receipt<old['received_monotonic_raw']:
                    return self.invalid('health_receipt_regression')
            saved[key]=dict(value);result[key]=row
        self.previous={'instance':instance,'produced':produced,'streams':saved}
        return result
