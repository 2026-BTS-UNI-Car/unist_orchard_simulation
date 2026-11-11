"""Bounded observational RAW-clock timings; never freshness or motion inputs."""
from collections import OrderedDict
from dataclasses import dataclass, field
import os
import re
import threading
import time

from scripts.sim_clock import clock_domain

SCHEMA='ubts.native_stage_timing.v1'
METRICS=frozenset(('callback_convert','journal_validation','journal_lock',
    'source_rpc_read','source_rpc_encode','client_rpc_connect','client_rpc_wire',
    'client_rpc_decode','callback_to_ingest','journal_to_ingest','ingest_lock','server_handler_to_ingest',
    'journal_to_source_selected','source_selected_to_ingest',
    'scan_queue_wait','pair_check','pair_wait','pair_wait_to_imu_ingest','pair_ready_dispatch','native_ingest','numeric_core',
    'localized','observer_submit','after_submit_core','snapshot_tiles',
    'snapshot_publish','core_to_publish','run_loop_wait','source_loop_wait',
    'processing_lock_wait'))
MARKS=frozenset(('callback_entry','converted','journal_admitted','ingest_entry',
    'queued','first_pair_wait','paired_imu_ingest','pair_ready','numeric_begin','localized_enter',
    'localized_exit','numeric_return','published'))
BOUNDS_MS=(1,2,4,8,16,32,64,96,128,160,200,256,320,512,1024,2048)
LIMIT=(1<<63)-1

@dataclass(frozen=True)
class Tick:
    domain: str
    ns: int

def _valid(tick):
    return (type(tick) is Tick and type(tick.ns) is int and 0<=tick.ns<=LIMIT
        and type(tick.domain) is str and re.fullmatch('[0-9a-f]{64}',tick.domain) is not None)

def raw_tick():
    # No wall/perf_counter/MONOTONIC fallback or rounded old float timestamp.
    domain=clock_domain()
    return Tick(domain,time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW))

@dataclass
class Trace:
    key: tuple
    marks: dict=field(default_factory=dict)
    pair_state: str='not_checked'

class Recorder:
    def __init__(self, enabled=None, clock=raw_tick):
        self.enabled=os.environ.get('UBTS_NATIVE_STAGE_TIMING')=='1' if enabled is None else bool(enabled)
        self.clock=clock;self.lock=threading.Lock();self.traces=OrderedDict();self.imu_ingest=OrderedDict();self.last=None;self.domain=None
        self.metrics={};self.errors={key:0 for key in ('clock_unavailable','domain_mismatch',
            'clock_reversed','unknown_metric','bad_marks','trace_evicted','trace_missing','trace_discarded','timing_failure')}
    def error(self,key):
        with self.lock:self.errors[key]=min(LIMIT,self.errors[key]+1)
    def now(self):
        if not self.enabled:return None
        try:
            tick=self.clock()
            if not _valid(tick):raise ValueError('invalid_raw_tick')
            with self.lock:
                if self.domain is None:self.domain=tick.domain
                changed=self.domain!=tick.domain
            if changed:self.error('domain_mismatch');return None
            return tick
        except Exception:
            self.error('clock_unavailable');return None
    def span(self,name,start,end):
        if not self.enabled or start is None or end is None:return None
        if name not in METRICS:self.error('unknown_metric');return None
        if not _valid(start) or not _valid(end):self.error('bad_marks');return None
        if start.domain!=end.domain or self.domain is not None and start.domain!=self.domain:
            self.error('domain_mismatch');return None
        duration=end.ns-start.ns
        if duration<0:self.error('clock_reversed');return None
        with self.lock:
            item=self.metrics.setdefault(name,dict(count=0,sum_ns=0,max_ns=0,buckets=[0]*(len(BOUNDS_MS)+1)))
            item['count']=min(LIMIT,item['count']+1);item['sum_ns']=min(LIMIT,item['sum_ns']+duration)
            item['max_ns']=max(item['max_ns'],duration)
            bucket=next((i for i,bound in enumerate(BOUNDS_MS) if duration<=bound*1000000),len(BOUNDS_MS))
            item['buckets'][bucket]=min(LIMIT,item['buckets'][bucket]+1)
        return duration
    def observe(self,name,start,end):
        # A diagnostics implementation failure must not stop native flow.
        try:return self.span(name,start,end)
        except Exception:
            self.error('timing_failure');return None
    def source_marks(self,entry,converted,admitted):
        if not self.enabled:return None
        values=(entry,converted,admitted)
        if not all(_valid(value) for value in values) or len({value.domain for value in values})!=1:return None
        if not entry.ns<=converted.ns<=admitted.ns:return None
        return dict(schema=SCHEMA,clock_source='CLOCK_MONOTONIC_RAW',clock_domain=entry.domain,
            marks_ns=dict(callback_entry=entry.ns,converted=converted.ns,journal_admitted=admitted.ns))
    def rpc_marks(self,entry,read,served):
        values=(entry,read,served)
        if not all(_valid(value) for value in values) or len({value.domain for value in values})!=1:return None
        if not entry.ns<=read.ns<=served.ns:return None
        return dict(schema=SCHEMA,clock_source='CLOCK_MONOTONIC_RAW',clock_domain=entry.domain,
            marks_ns=dict(server_entry=entry.ns,request_read=read.ns,journal_read_done=served.ns))
    def wire_tick(self,payload,name):
        if not self.enabled or payload is None:return None
        if (type(payload) is not dict or payload.get('schema')!=SCHEMA or payload.get('clock_source')!='CLOCK_MONOTONIC_RAW'
            or type(payload.get('marks_ns')) is not dict or set(payload['marks_ns'])!={'server_entry','request_read','journal_read_done'}):
            self.error('bad_marks');return None
        values={key:Tick(payload.get('clock_domain'),value) for key,value in payload['marks_ns'].items()}
        if not all(_valid(value) for value in values.values()) or not values['server_entry'].ns<=values['request_read'].ns<=values['journal_read_done'].ns:
            self.error('bad_marks');return None
        return values.get(name)
    def source_snapshot(self,payload):
        # Optional remote diagnostics cannot add arbitrary metrics/history.
        if not self.enabled or payload is None:return None
        if (type(payload) is not dict or payload.get('schema')!=SCHEMA or
            payload.get('clock_source')!='CLOCK_MONOTONIC_RAW' or payload.get('hist_upper_bounds_ms')!=list(BOUNDS_MS) or
            payload.get('diagnostic_only') is not True or payload.get('used_for_freshness') is not False or
            payload.get('used_for_localization') is not False or payload.get('last') is not None):
            self.error('bad_marks');return None
        domain=payload.get('clock_domain')
        if type(domain) is not str or re.fullmatch('[0-9a-f]{64}',domain) is None:
            self.error('bad_marks');return None
        if self.domain is not None and domain!=self.domain:
            self.error('domain_mismatch');return None
        metrics=payload.get('metrics');errors=payload.get('errors')
        if type(metrics) is not dict or not set(metrics)<=METRICS or type(errors) is not dict or set(errors)!=set(self.errors):
            self.error('bad_marks');return None
        for item in metrics.values():
            if type(item) is not dict or set(item)!={'count','sum_ns','max_ns','buckets'}:
                self.error('bad_marks');return None
            if type(item['buckets']) is not list or len(item['buckets'])!=len(BOUNDS_MS)+1:
                self.error('bad_marks');return None
            if any(type(value) is not int or not 0<=value<=LIMIT for value in
                   [item['count'],item['sum_ns'],item['max_ns']]+item['buckets']):
                self.error('bad_marks');return None
        if any(type(value) is not int or not 0<=value<=LIMIT for value in errors.values()):
            self.error('bad_marks');return None
        return dict(schema=SCHEMA,clock_source='CLOCK_MONOTONIC_RAW',clock_domain=domain,
            metrics={name:dict(item,buckets=list(item['buckets'])) for name,item in metrics.items()},errors=dict(errors),
            hist_upper_bounds_ms=list(BOUNDS_MS),diagnostic_only=True,used_for_freshness=False,used_for_localization=False)
    def source_callback(self,journal,topic,convert,message):
        entry=self.now();converted_message=convert(message);converted=self.now()
        self.observe('callback_convert',entry,converted)
        journal.append(topic,converted_message,timing=(entry,converted))
    def admit(self,key,source,entry,queued,selected=None):
        if not self.enabled:return None
        trace=Trace(key)
        if type(source) is dict:
            points=source.get('marks_ns')
            if (source.get('schema')==SCHEMA and source.get('clock_source')=='CLOCK_MONOTONIC_RAW'
                and type(points) is dict and set(points)=={'callback_entry','converted','journal_admitted'}):
                ticks={name:Tick(source.get('clock_domain'),value) for name,value in points.items()}
                if all(_valid(value) for value in ticks.values()) and ticks['callback_entry'].ns<=ticks['converted'].ns<=ticks['journal_admitted'].ns:
                    trace.marks.update(ticks)
                else:self.error('bad_marks')
            else:self.error('bad_marks')
        elif source is not None:self.error('bad_marks')
        if entry is not None:trace.marks['ingest_entry']=entry
        if queued is not None:trace.marks['queued']=queued
        self.observe('callback_to_ingest',trace.marks.get('callback_entry'),entry)
        self.observe('journal_to_ingest',trace.marks.get('journal_admitted'),entry)
        self.observe('journal_to_source_selected',trace.marks.get('journal_admitted'),selected)
        self.observe('source_selected_to_ingest',selected,entry)
        self.observe('callback_convert',trace.marks.get('callback_entry'),trace.marks.get('converted'))
        self.observe('ingest_lock',entry,queued)
        with self.lock:
            self.traces[key]=trace;self.traces.move_to_end(key)
            if len(self.traces)>8:self.traces.popitem(last=False);self.errors['trace_evicted']=min(LIMIT,self.errors['trace_evicted']+1)
        return trace
    def find(self,key):
        if not self.enabled:return None
        with self.lock:return self.traces.get(key)
    def discard(self,key):
        if not self.enabled:return
        with self.lock:
            if self.traces.pop(key,None) is not None:self.errors['trace_discarded']=min(LIMIT,self.errors['trace_discarded']+1)
    def note_imu(self,key,tick):
        if not self.enabled or not _valid(tick):return
        with self.lock:
            self.imu_ingest[key]=tick;self.imu_ingest.move_to_end(key)
            if len(self.imu_ingest)>64:self.imu_ingest.popitem(last=False)
    def pair_ready(self,trace,key):
        ready=self.mark(trace,'pair_ready')
        if trace is None:return
        with self.lock:received=self.imu_ingest.get(key)
        if received is not None:
            self.mark(trace,'paired_imu_ingest',received)
    def mark(self,trace,name,tick=None):
        if trace is None or not self.enabled:return None
        if name not in MARKS:self.error('bad_marks');return None
        tick=self.now() if tick is None else tick
        if tick is not None:trace.marks[name]=tick
        return tick
    def finish(self,trace,status):
        if trace is None or not self.enabled:return
        pairs={'scan_queue_wait':('queued','numeric_begin'),
               'pair_wait':('first_pair_wait','pair_ready'),
               'pair_wait_to_imu_ingest':('first_pair_wait','paired_imu_ingest'),
               'pair_ready_dispatch':('paired_imu_ingest','pair_ready'),
               'numeric_core':('numeric_begin','numeric_return'),
               'localized':('numeric_begin','localized_enter'),
               'observer_submit':('localized_enter','localized_exit'),
               'after_submit_core':('localized_exit','numeric_return'),
               'core_to_publish':('numeric_return','published')}
        intervals={name:self.observe(name,trace.marks.get(a),trace.marks.get(b)) for name,(a,b) in pairs.items()}
        with self.lock:
            self.traces.pop(trace.key,None)
            self.last={'key':list(trace.key),'status':status,'pair_state':trace.pair_state,
                       'intervals_ns':intervals}
    def clear(self):
        with self.lock:self.traces.clear();self.imu_ingest.clear();self.last=None
    def snapshot(self):
        if not self.enabled:return None
        with self.lock:
            return dict(schema=SCHEMA,clock_source='CLOCK_MONOTONIC_RAW',clock_domain=self.domain,
                diagnostic_only=True,used_for_freshness=False,used_for_localization=False,
                hist_upper_bounds_ms=list(BOUNDS_MS),
                metrics={name:dict(value,buckets=list(value['buckets'])) for name,value in self.metrics.items()},
                errors=dict(self.errors),pending_traces=len(self.traces),imu_timing_entries=len(self.imu_ingest),last=None if self.last is None else
                dict(self.last,key=list(self.last['key']),intervals_ns=dict(self.last['intervals_ns'])))
