"""Bounded diagnostic ages only. Never supplies an input or decision clock."""
from dataclasses import dataclass
from pathlib import Path
import hashlib,sys,threading,time,uuid

MAX=(1<<63)-1
PHASES=('PROVIDER_SAMPLED','PRE_PLAN','POST_PLAN_BEFORE_RPC','POST_RPC')
LEAVES=('scan_SOURCE','imu_SOURCE','scan_RAW','imu_RAW','motion_RAW')
BINS_NS=(0,50_000_000,100_000_000,150_000_000,175_000_000,190_000_000,200_000_000,
         225_000_000,250_000_000,300_000_000,500_000_000,1_000_000_000)

@dataclass(frozen=True)
class OriginalRowPin:
    context: tuple
    stamps_and_receipts: tuple
    provider_clocks: tuple

def linux_clock_identity():
    if not sys.platform.startswith('linux')or not all(hasattr(time,k)for k in('CLOCK_REALTIME','CLOCK_MONOTONIC_RAW')):
        raise ValueError('LINUX_CLOCKS_UNAVAILABLE')
    boot=Path('/proc/sys/kernel/random/boot_id').read_text(encoding='ascii').strip()
    if str(uuid.UUID(boot))!=boot:raise ValueError('BOOT_IDENTITY_UNAVAILABLE')
    return hashlib.sha256((boot+':CLOCK_MONOTONIC_RAW').encode('ascii')).hexdigest()

class OriginalRowAgeBudget:
    def __init__(self):
        self.lock=threading.Lock()
        try:self.domain=linux_clock_identity()
        except Exception:self.domain=None
        self.phases={p:dict(samples=0,unknown=0,sampling_span_ns_max=0,
            leaves={k:dict(count=0,min_ns=None,max_ns=None,total_ns=0,negative=0,
                over_RAW200ms=0,over_SOURCE250ms=0,histogram=[0]*(len(BINS_NS)+1))for k in LEAVES})for p in PHASES}

    def pin(self,row):
        try:
            c=row['context'];s=row['scan'];i=row['imu'];m=row['motion']
            if(self.domain is None or c['source_clock_domain']!='ROS_SYSTEM_TIME'or
               c['receipt_clock_domain']!='CLOCK_MONOTONIC_RAW:'+self.domain or
               s.get('original_native')is not True or i.get('original_native')is not True):return None
            context=tuple(c[k]for k in('source_session','epoch','generation','source_clock_domain','receipt_clock_domain'))
            values=(s['stamp_ns'],i['stamp_ns'],s['receipt_ns'],i['receipt_ns'],m['receipt_ns'])
            clocks=(row['source_now_ns'],row['receipt_now_ns'])
            if not(all(type(v)is int and 0<=v<=MAX for v in values+clocks)and type(context[2])is int and
                all(type(v)is str and 0<len(v)<=256 for v in(context[0],context[1],context[3],context[4]))):return None
            return OriginalRowPin(context,values,clocks)
        except Exception:return None

    def observe(self,phase,pin):
        if phase not in PHASES:return
        try:
            if(type(pin)is not OriginalRowPin or self.domain is None or
               pin.context[3]!='ROS_SYSTEM_TIME'or pin.context[4]!='CLOCK_MONOTONIC_RAW:'+self.domain):raise ValueError('CLOCK_DOMAIN_UNQUALIFIED')
            if phase=='PROVIDER_SAMPLED':source,raw=pin.provider_clocks;raw_before=raw
            else:
                raw_before=time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
                source=time.clock_gettime_ns(time.CLOCK_REALTIME)
                raw=time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
            if not(0<=raw-raw_before<=60_000_000_000):raise ValueError('CLOCK_SAMPLE_SPAN_INVALID')
            s,i,sr,ir,mr=pin.stamps_and_receipts
            ages=(source-s,source-i,raw-sr,raw-ir,raw-mr)
            if not all(type(v)is int and -60_000_000_000<=v<=60_000_000_000 for v in ages):raise ValueError('DIAGNOSTIC_AGE_OUTSIDE_CAP')
            with self.lock:
                p=self.phases[phase];p['samples']=min(MAX,p['samples']+1)
                p['sampling_span_ns_max']=max(p['sampling_span_ns_max'],raw-raw_before)
                for key,age in zip(LEAVES,ages):
                    v=p['leaves'][key];v['count']=min(MAX,v['count']+1)
                    v['min_ns']=age if v['min_ns']is None else min(v['min_ns'],age)
                    v['max_ns']=age if v['max_ns']is None else max(v['max_ns'],age)
                    v['total_ns']=max(-MAX,min(MAX,v['total_ns']+age));v['negative']=min(MAX,v['negative']+int(age<0))
                    v['over_RAW200ms']=min(MAX,v['over_RAW200ms']+int(key.endswith('RAW')and age>=200_000_000))
                    v['over_SOURCE250ms']=min(MAX,v['over_SOURCE250ms']+int(key.endswith('SOURCE')and age>=250_000_000))
                    index=next((j for j,b in enumerate(BINS_NS)if age<b),len(BINS_NS))
                    v['histogram'][index]=min(MAX,v['histogram'][index]+1)
        except Exception:
            with self.lock:self.phases[phase]['unknown']=min(MAX,self.phases[phase]['unknown']+1)

    def snapshot(self):
        with self.lock:
            phases={p:dict(samples=v['samples'],unknown=v['unknown'],sampling_span_ns_max=v['sampling_span_ns_max'],
                leaves={k:dict(x,histogram=list(x['histogram']))for k,x in v['leaves'].items()})for p,v in self.phases.items()}
        return dict(schema='ubts.observation.original_row_age_budget.v1',
            clock_contract='DIAGNOSTIC_LINUX_REALTIME_RAW_BOOT_DOMAIN_MATCH_ONLY',
            clock_domain_available=self.domain is not None,original_row_pinned_across_phases=True,
            sensor_or_decision_clock_changed=False,absolute_clock_or_sensor_stamps_exported=False,
            histogram_upper_bounds_ns=list(BINS_NS),histogram_boundary='strict_less_than; final bin unbounded_within_60s_cap',
            phases=phases)
