"""Default-OFF owned lease sink for native planning + measured drive preview.

No hardware, API, ROS, socket or thread creation. The application supplies trusted
local-session authorization, original-native getter, verified physical evidence,
measured BODY/command-specific swept adapter, and rover_api-compatible request.
Those injected declarations are not certificates verified by this component.
Existing manual lease/ticket/deadman gates remain the sole actuator transport.
"""
from collections import deque
from dataclasses import dataclass,asdict
import copy,hashlib,json,math,threading,time
from dashboard_server.record_based_autonomy import RecordBasedAutonomy
from dashboard_server.encoderless_drive_preview import (
    EncoderlessDrivePreview,MeasuredProfile,TimingLimits,NativeContext,NativeFeedback,SweptSpace)

SOURCE_TTL_NS=250_000_000
RAW_TTL_NS=200_000_000
MAX_INPUT_BYTES=262144
MAX_DECISION_BYTES=32768
MAX_COUNTER=(1<<63)-1


def require(value,reason):
    if not value:raise ValueError(reason)


def number(v):
    try:return type(v) in (int,float) and math.isfinite(v)
    except OverflowError:return False
def ns(v):return type(v) is int and 0<=v<=MAX_COUNTER
def name(v):return type(v) is str and 0<len(v)<=128 and v.strip()==v


@dataclass(frozen=True)
class ActivationEvidence:
    context:NativeContext
    body_mount_verified:bool=False
    native_body_velocity_validated:bool=False
    braking_verified:bool=False
    emergency_stop_verified:bool=False
    source_pipeline_validated:bool=False
    swept_provider_validated:bool=False
    motor_configuration_reviewed:bool=False
    body_evidence_id:str=''
    braking_evidence_id:str=''
    emergency_stop_evidence_id:str=''


@dataclass(frozen=True)
class MeasuredControlFrame:
    feedback:NativeFeedback
    swept:SweptSpace
    desired_forward_m_s:float
    desired_yaw_rad_s:float


class AutonomySupervisor:
    def __init__(self,*,enabled=False,input_provider=None,evidence_provider=None,
                 control_provider=None,authorize=None,request=None,planner=None,
                 profile=None,timing=None,percent_cap=None,clock=time.monotonic):
        require(type(enabled) is bool,'explicit_enabled_bool_required')
        require(percent_cap is None or (number(percent_cap) and 0<percent_cap<=100),
                'explicit_percent_cap_in_percent_range_required')
        self.enabled=enabled;self.input_provider=input_provider;self.evidence_provider=evidence_provider
        self.control_provider=control_provider;self.authorize=authorize;self.request=request
        self.planner=planner if planner is not None else RecordBasedAutonomy()
        self.preview=EncoderlessDrivePreview(profile,timing);self.percent_cap=percent_cap;self.clock=clock
        self._lock=threading.RLock();self._step=threading.Lock();self._events=deque(maxlen=128)
        self._generation=0;self._cancel=threading.Event();self._cancel.set()
        self._session=self._context=self._evidence=self._last_input=self._lease=None
        self._ticket_until=0.;self._starting=False;self._stopping=False;self._transport_started=False;self._last_digest=None;self._last_clocks=None;self._state='DISARMED';self._reason='default_OFF'
        self._decision=None;self._requested=(0,0);self._stop_ack=None
        self._counts=dict(inputs=0,plans=0,commands=0,duplicates=0,stops=0,errors=0)
    def _count(self,key):
        with self._lock:
            require(self._counts[key]<MAX_COUNTER,'counter_exhausted');self._counts[key]+=1
    def _event(self,reason):
        with self._lock:self._events.append(dict(generation=self._generation,state=self._state,reason=str(reason)[:128]))
    def status(self):
        with self._lock:
            return dict(schema='ubts.native_autonomy_supervisor.v1',enabled=self.enabled,
                control_path_implemented=True,control_mode='explicit_local_manual_lease_forward_only',state=self._state,reason=self._reason,generation=self._generation,
                active=not self._cancel.is_set(),owner_fingerprint=(hashlib.sha256(self._session.encode()).hexdigest() if self._session else None),
                cruise_percent=getattr(self,'_cruise',None),percent_cap=self.percent_cap,
                effective_percent_cap=(getattr(self.preview.profile,'max_percent',None) if self.percent_cap is None else self.percent_cap),
                requested_left_percent=self._requested[0],requested_right_percent=self._requested[1],
                activation_prerequisites=['reviewed_BODY_feedback','measured_braking_profile','command_specific_swept_clearance','verified_emergency_stop','explicit_local_manual_lease_and_deadman'],
                applied_output_verified=False,stop_ack=copy.deepcopy(self._stop_ack),
                hardware_stop_pending_or_unknown=self._stop_ack is not None,
                trusted_prerequisites_accepted=self._evidence is not None and not self._cancel.is_set(),
                motor_lease_acquired=self._lease is not None,
                owner_cleanup_pending=self._starting or self._stopping or self._step.locked(),
                current_decision=copy.deepcopy(self._decision),counters=dict(self._counts),
                evidence=list(self._events),physical_autonomy_qualified=False,
                calibration_verified_by_this_component=False,motor_command_sent=self._counts['commands']>0)
    def _active(self,generation,cancel):
        require(not cancel.is_set() and self._generation==generation and self._cancel is cancel,
                'supervisor_generation_cancelled')
    def _snapshot(self):
        value=self.input_provider()
        if value is None:return None
        require(type(value) is dict,'native_input_object_required')
        raw=json.dumps(value,allow_nan=False,separators=(',',':')).encode()
        require(len(raw)<=MAX_INPUT_BYTES,'bounded_native_input_required')
        return json.loads(raw)
    @staticmethod
    def _identity(s):
        c=s.get('context');scan=s.get('scan');imu=s.get('imu');motion=s.get('motion')
        require(type(c) is dict and type(scan) is dict and type(imu) is dict and type(motion) is dict,
                'native_scan_IMU_motion_context_required')
        require(all(name(c.get(k)) for k in ('source_session','epoch','source_clock_domain','receipt_clock_domain')) and ns(c.get('generation')),
                'original_native_context_required')
        require(scan.get('original_native') is True and imu.get('original_native') is True and
                motion.get('accepted') is True and motion.get('map_eligible') is True and
                motion.get('path_break') is False and motion.get('motion_prior') is False,
                'accepted_original_native_pair_required')
        fields=(scan.get('stamp_ns'),scan.get('receipt_ns'),scan.get('sequence'),imu.get('stamp_ns'),imu.get('receipt_ns'),motion.get('receipt_ns'))
        require(all(ns(v) for v in fields) and name(scan.get('frame_id')),
                'typed_original_native_stamps_required')
        require(motion.get('current_scan_stamp_ns')==scan['stamp_ns'] and
                ns(motion.get('previous_scan_stamp_ns')) and 0<motion['current_scan_stamp_ns']-motion['previous_scan_stamp_ns']<=200_000_000 and
                0<=scan['stamp_ns']-imu['stamp_ns']<=60_000_000 and
                all(number(motion.get(k)) for k in ('dx_m','dy_m','dyaw_rad')) and number(imu.get('yaw_rate_rad_s')),
                'causal_original_native_pair_required')
        return (tuple(c[k] for k in ('source_session','epoch','generation','source_clock_domain','receipt_clock_domain')),
                scan['sequence'],scan['stamp_ns'],scan['receipt_ns'],scan['frame_id'])
    @staticmethod
    def _fresh(s,source_now=None,raw_now=None):
        source=s.get('source_now_ns') if source_now is None else source_now
        raw=s.get('receipt_now_ns') if raw_now is None else raw_now
        require(ns(source) and ns(raw),'original_comparison_clocks_required')
        scan,imu,motion=s['scan'],s['imu'],s['motion']
        require(all(0<=source-v<SOURCE_TTL_NS for v in (scan['stamp_ns'],imu['stamp_ns'])) and
                all(0<=raw-v<RAW_TTL_NS for v in (scan['receipt_ns'],imu['receipt_ns'],motion['receipt_ns'])),
                'native_input_stale_or_future')
    @staticmethod
    def _fingerprint(s):
        return hashlib.sha256(json.dumps({k:s[k] for k in ('context','scan','motion','imu')},allow_nan=False,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    def _physical_evidence(self,s):
        p=self.preview.profile;t=self.preview.timing
        require(type(p) is MeasuredProfile and p.valid(),'measured_BODY_and_braking_profile_required')
        require(type(t) is TimingLimits and t.valid(),'measured_timing_envelope_required')
        require(self.percent_cap is None or self.percent_cap<=p.max_percent,
                'explicit_cap_within_measured_profile_required')
        e=self.evidence_provider(s)
        require(type(e) is ActivationEvidence and type(e.context) is NativeContext and e.context.valid(),
                'verified_physical_activation_evidence_required')
        for flag in ('body_mount_verified','native_body_velocity_validated','braking_verified','emergency_stop_verified',
                     'source_pipeline_validated','swept_provider_validated','motor_configuration_reviewed'):
            require(getattr(e,flag) is True,'physical_preflight_missing:'+flag)
        require(all(name(getattr(e,k)) for k in ('body_evidence_id','braking_evidence_id','emergency_stop_evidence_id')),
                'reviewed_physical_evidence_ids_required')
        require(e.context.calibration_id==p.calibration_id and
                (e.context.source_session,e.context.epoch,e.context.generation,e.context.source_clock,e.context.receipt_clock)==self._identity(s)[0],
                'physical_native_source_context_mismatch')
        return e
    def start(self,session,*,explicit=False,cruise_percent=0):
        require(explicit is True and name(session),'explicit_local_operator_start_required')
        require(number(cruise_percent) and 0<=cruise_percent<=100,'cruise_preference_in_percent_range_required')
        with self._lock:
            if not self._cancel.is_set():
                require(session==self._session,'single_operator_owner_required');return self.status()
            if not self.enabled:self._state='BLOCKED';self._reason='default_OFF';return self.status()
            require(all(callable(v) for v in (self.input_provider,self.evidence_provider,self.control_provider,self.authorize,self.request)),
                    'trusted_native_physical_and_transport_bindings_required')
            require(not self._starting and not self._stopping and not self._step.locked(),'previous_owner_cleanup_pending')
            self._starting=True;self._transport_started=False
            self._generation+=1;g=self._generation;cancel=self._cancel=threading.Event()
            self._session=session;self._cruise=cruise_percent;self._lease=None;self._last_input=None;self._last_digest=None;self._last_clocks=None
            self._state='PREPARING';self._reason='checking_physical_preflight';self._decision=None;self._evidence=None;self._stop_ack=None;self._requested=(0,0)
        try:
            require(self.authorize(session,'start') is True,'explicit_local_manual_arming_authorization_required')
            s=self._snapshot();require(s is not None,'fresh_native_input_required');self._identity(s);self._fresh(s)
            e=self._physical_evidence(s);self._active(g,cancel)
            planner_limit=getattr(getattr(self.planner,'estimates',None),'max_percent',None)
            require(number(planner_limit) and cruise_percent<=planner_limit,
                    'cruise_preference_exceeds_existing_planner_estimate_range')
            self.planner.reset(s['context']);out=self.preview.reset(e.context)
            require(out.latched is False,'measured_controller_reset_failed')
            self._active(g,cancel)
            with self._lock:
                self._active(g,cancel)
                self._context=self._identity(s)[0];self._evidence=e;self._state='WAIT_DECISION';self._reason='explicit_start_zero'
            self._event('explicit_start_zero')
        except Exception as error:self._fail(str(error),g,cancel,stop_transport=False)
        finally:
            with self._lock:self._starting=False
        return self.status()
    def _wire(self,payload,g,cancel,original,digest):
        self._active(g,cancel)
        require(self.authorize(self._session,'continue') is True,'operator_deadman_or_session_lost')
        latest=self._snapshot();require(latest is not None,'WAIT_NATIVE_INPUT_before_transport')
        old=self._identity(original);current=self._identity(latest);self._fresh(latest)
        require(current[0]==old[0] and current[1]>=old[1] and current[2]>=old[2] and current[3]>=old[3] and current[4]==old[4],
                'native_source_changed_before_transport')
        require(current!=old or self._fingerprint(latest)==digest,'same_sequence_native_input_mutated')
        self._fresh(original,latest['source_now_ns'],latest['receipt_now_ns'])
        self._active(g,cancel)
        began=self.clock()
        with self._lock:
            self._active(g,cancel)
            if payload.get('op') in ('arm','drive'):self._transport_started=True
        answer=self.request(payload)
        if cancel.is_set() or self._generation!=g:
            if self._transport_started:self._priority_stop()
            raise ValueError('supervisor_generation_cancelled')
        require(type(answer) is dict and answer.get('ok') is True,'motor_bridge_rejected_or_unavailable')
        return answer,began
    def _priority_stop(self):
        if not callable(self.request):return
        try:
            a=self.request(dict(op='stop',reason='autonomy_supervisor_stop'))
            require(type(a) is dict and a.get('ok') is True,'bridge_stop_unacknowledged')
            with self._lock:self._stop_ack={'requested':True,'bridge_reply_state':str(a.get('state',''))[:80],'physical_zero_verified':False}
        except Exception as error:
            with self._lock:self._stop_ack={'requested':True,'error':str(error)[:128],'physical_zero_verified':False}
    def _fail(self,reason,g,cancel,*,stop_transport=True):
        cancel.set()
        with self._lock:
            if self._generation!=g or self._cancel is not cancel:return
            had_lease=self._transport_started
            first=self._state!='STOPPED'
            if first:
                self._stopping=True;self._state='STOPPED';self._reason=str(reason)[:128]
                self._decision=dict(decision='STOP',reason=self._reason,proposed_left_percent=0.,proposed_right_percent=0.,
                    applied_left_percent=0.,applied_right_percent=0.,trajectory=[],execution_eligible=False)
            self._requested=(0,0);self._lease=None
            retained_reason=self._reason
        # No state/step/numeric lock is held across the priority STOP request.
        # This precedes planner/controller locks and does not wait for drive RPC.
        try:
            if stop_transport and had_lease:self._priority_stop()
            if first:
                self.planner.stop(retained_reason);self.preview.stop(retained_reason)
                self._count('stops');self._event(retained_reason)
        finally:
            if first:
                with self._lock:
                    if self._generation==g and self._cancel is cancel:self._stopping=False
    def stop(self,reason='operator_stop'):
        with self._lock:
            g,cancel=self._generation,self._cancel;active=not cancel.is_set()
            if active:self._stopping=True
            cancel.set();had_lease=self._lease is not None
        if active:self._fail(reason,g,cancel)
        elif had_lease:self._priority_stop()
        return self.status()
    def manual_takeover(self):return self.stop('manual_takeover')
    def tick(self):
        if not self._step.acquire(blocking=False):return self.status()
        try:
            with self._lock:g,cancel=self._generation,self._cancel
            if cancel.is_set() or self._starting:return self.status()
            s=self._snapshot()
            if s is None:
                if self._lease:self._fail('WAIT_NATIVE_INPUT',g,cancel)
                else:
                    with self._lock:
                        self._active(g,cancel)
                        self._state='WAIT_INPUT';self._reason='WAIT_NATIVE_INPUT'
                return self.status()
            identity=self._identity(s);self._fresh(s);digest=self._fingerprint(s)
            if self._last_clocks is not None:
                require(s['source_now_ns']>=self._last_clocks[0] and s['receipt_now_ns']>=self._last_clocks[1],
                        'native_comparison_clock_reversal')
            require(identity[0]==self._context,'native_source_context_changed')
            e=self._physical_evidence(s);require(e==self._evidence,'physical_evidence_changed')
            if self._last_input is not None:
                previous=self._last_input
                require(identity[1]>=previous[1] and identity[2]>=previous[2] and identity[3]>=previous[3] and identity[4]==previous[4],
                        'native_source_sequence_or_stamp_rewind')
                if identity==previous:
                    require(digest==self._last_digest,'same_sequence_native_input_mutated')
                    require(self.authorize(self._session,'continue') is True,'operator_deadman_or_session_lost')
                    self._count('duplicates');return self.status() # no drive/keepalive or input-TTL renewal
            self._active(g,cancel);self._count('inputs')
            decision=self.planner.plan(s,cruise_percent=self._cruise);self._count('plans')
            require(type(decision) is dict and len(json.dumps(decision,allow_nan=False).encode())<=MAX_DECISION_BYTES,
                    'bounded_planner_decision_required')
            with self._lock:
                self._active(g,cancel)
                self._decision=copy.deepcopy(decision)
            self._active(g,cancel)
            if decision.get('decision')!='PROPOSE_FORWARD':
                if decision.get('decision')=='STOP' or self._lease:self._fail('planner_'+str(decision.get('reason','HOLD'))[:100],g,cancel)
                else:
                    with self._lock:
                        self._active(g,cancel)
                        self._last_input=identity;self._last_digest=digest;self._last_clocks=(s['source_now_ns'],s['receipt_now_ns']);self._state='WAIT_DECISION';self._reason=str(decision.get('reason','HOLD'))[:128]
                return self.status()
            require(decision.get('record_replay_estimate') is False and decision.get('source_provenance_unverified') is False,
                    'record_replay_or_unverified_source_forbidden')
            frame=self.control_provider(s,copy.deepcopy(decision),self.preview.profile)
            require(type(frame) is MeasuredControlFrame,'measured_BODY_and_command_swept_adapter_required')
            require(type(frame.feedback) is NativeFeedback and type(frame.swept) is SweptSpace and
                    frame.feedback.context==e.context and frame.swept.context==e.context and
                    frame.feedback.source_stamp_ns==s['scan']['stamp_ns'] and frame.feedback.receipt_ns==s['scan']['receipt_ns'],
                    'measured_control_original_endpoint_required')
            latest=self._snapshot();require(latest is not None,'WAIT_NATIVE_INPUT_after_planning')
            current=self._identity(latest);self._fresh(latest)
            require(current[0]==identity[0] and current[1]>=identity[1] and current[2]>=identity[2] and current[3]>=identity[3] and current[4]==identity[4],
                    'native_source_changed_during_planning')
            require(current!=identity or self._fingerprint(latest)==digest,'same_sequence_native_input_mutated')
            self._fresh(s,latest['source_now_ns'],latest['receipt_now_ns']);self._active(g,cancel)
            output=self.preview.step(frame.feedback,frame.swept,desired_forward_m_s=frame.desired_forward_m_s,
                desired_yaw_rad_s=frame.desired_yaw_rad_s,source_now_ns=latest['source_now_ns'],receipt_now_ns=latest['receipt_now_ns'])
            require(not output.latched,'measured_controller_HOLD:'+output.reason)
            self._active(g,cancel)
            if self._lease is None:
                available,_began=self._wire(dict(op='status'),g,cancel,s,digest)
                require(available.get('connected') is True and available.get('state')=='disarmed' and
                        available.get('armed') is False and available.get('moving') is False and
                        available.get('automatic_input_enabled') is False,
                        'existing_manual_bridge_not_idle_or_unavailable')
                answer,began=self._wire(dict(op='arm',owner=self._session),g,cancel,s,digest)
                require(name(answer.get('lease')) and name(answer.get('ticket')) and type(answer.get('seq')) is int and answer['seq']==0 and
                        number(answer.get('ticket_ttl_s')) and 0<answer['ticket_ttl_s']<=1.5,'owned_manual_arm_reply_required')
                with self._lock:
                    self._active(g,cancel)
                    self._lease=dict(lease=answer['lease'],ticket=answer['ticket'],seq=answer['seq'])
                    self._ticket_until=began+answer['ticket_ttl_s']
            cap=self.preview.profile.max_percent if self.percent_cap is None else self.percent_cap
            left,right=(int(math.floor(min(v,cap))) for v in (output.left_percent,output.right_percent))
            require(0<=left<=cap and 0<=right<=cap,'bounded_forward_preview_required')
            if left or right:
                require(left>0 and right>0,'both_wheels_forward_required')
                require(self.clock()<self._ticket_until,'manual_ticket_expired')
                q=dict(self._lease,op='drive',owner=self._session,seq=self._lease['seq']+1,left=left,right=right,deadman=True)
                answer,began=self._wire(q,g,cancel,s,digest)
                require(answer.get('lease')==self._lease['lease'] and name(answer.get('ticket')) and
                        type(answer.get('seq')) is int and answer['seq']==q['seq'] and
                        number(answer.get('ticket_ttl_s')) and 0<answer['ticket_ttl_s']<=1.5,'owned_manual_drive_reply_required')
                with self._lock:
                    self._active(g,cancel)
                    self._lease.update(ticket=answer['ticket'],seq=answer['seq']);self._ticket_until=began+answer['ticket_ttl_s']
                self._count('commands')
            check=self._snapshot();require(check is not None,'WAIT_NATIVE_INPUT_after_transport')
            check_id=self._identity(check);self._fresh(check)
            require(check_id[0]==identity[0] and check_id[1]>=current[1] and check_id[2]>=current[2] and check_id[3]>=current[3] and check_id[4]==identity[4],
                    'native_source_changed_during_transport')
            require(check_id!=identity or self._fingerprint(check)==digest,'same_sequence_native_input_mutated')
            self._fresh(s,check['source_now_ns'],check['receipt_now_ns']);self._active(g,cancel)
            with self._lock:
                self._active(g,cancel)
                self._last_input=identity;self._last_digest=digest;self._last_clocks=(check['source_now_ns'],check['receipt_now_ns']);self._requested=(left,right);self._state='ARMED_EXPERIMENT';self._reason=output.reason
            self._event(output.reason)
        except Exception as error:
            self._count('errors');self._fail(str(error),g,cancel)
        finally:self._step.release()
        return self.status()
    def run(self,stop_event,*,interval_s=.02):
        require(type(stop_event) is threading.Event and number(interval_s) and .01<=interval_s<=.1,
                'bounded_source_loop_arguments_required')
        try:
            while not stop_event.wait(interval_s):
                self.tick()
                if self._cancel.is_set():break
        finally:self.stop('supervisor_loop_stop')
