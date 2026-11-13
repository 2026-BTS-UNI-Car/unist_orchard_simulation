"""Explicit app binding; no constructor IO and no env/RPC physical feature switch."""
from dataclasses import dataclass
import threading
from dashboard_server.native_autonomy_supervisor import AutonomySupervisor

@dataclass(frozen=True)
class TrustedPhysicalBindings:
    profile:object
    timing:object
    evidence_provider:object
    control_provider:object
    authorize:object
    percent_cap:float

class NativeAutonomyApplication:
    def __init__(self,native_client=None,*,session_valid=lambda session:False,bindings=None,request=None):
        if bindings is not None and type(bindings) is not TrustedPhysicalBindings:
            raise ValueError('trusted_boot_physical_bindings_required')
        def transport(payload):
            if request is not None:return request(payload)
            from dashboard_server.rover_api import request as actual_request
            return actual_request(payload)
        def authorize(session,phase):
            return (session_valid(session) is True and bindings is not None and
                    callable(bindings.authorize) and bindings.authorize(session,phase) is True)
        self.supervisor=AutonomySupervisor(enabled=bindings is not None,
            input_provider=native_client.input if native_client is not None else lambda:None,
            evidence_provider=bindings.evidence_provider if bindings else lambda snapshot:None,
            control_provider=bindings.control_provider if bindings else None,
            authorize=authorize,request=transport,
            profile=bindings.profile if bindings else None,timing=bindings.timing if bindings else None,
            percent_cap=bindings.percent_cap if bindings else None)
        self._lock=threading.RLock();self._thread=None;self._exit=threading.Event()
        self._creating=False;self._unknown=False
    def status(self):
        value=self.supervisor.status()
        with self._lock:
            alive=self._thread is not None and self._thread.is_alive()
            return dict(value,loop_alive=alive,loop_cleanup_pending=self._creating or self._unknown or alive,
                physical_bindings_configured=self.supervisor.enabled)
    def start(self,session,*,explicit=False,cruise_percent=0):
        with self._lock:
            if self._creating or self._unknown:raise ValueError('previous_supervisor_loop_cleanup_pending')
            if self._thread is not None:
                if self._thread.is_alive():
                    if self.supervisor.status()['active']:
                        self.supervisor.start(session,explicit=explicit,cruise_percent=cruise_percent)
                        return self.status()
                    raise ValueError('previous_supervisor_loop_cleanup_pending')
                self._thread=None
            self._creating=True;exit_event=self._exit=threading.Event()
        try:
            state=self.supervisor.start(session,explicit=explicit,cruise_percent=cruise_percent)
            thread=None
            if state['active'] and not exit_event.is_set():
                with self._lock:
                    if not exit_event.is_set() and self.supervisor.status()['active']:
                        thread=self._thread=threading.Thread(target=self.supervisor.run,args=(exit_event,),name='native-autonomy-supervisor',daemon=False)
                if thread is not None:
                    try:thread.start()
                    except Exception:
                        with self._lock:self._unknown=True
                        self.supervisor.stop('supervisor_thread_start_unknown');raise
                    if not self.supervisor.status()['active']:exit_event.set()
        finally:
            with self._lock:self._creating=False
        # Snapshot only after the creation attempt resolved. In particular a
        # blocked default-OFF Start never reports a fictitious pending worker.
        return self.status()
    def stop(self,reason='operator_stop'):
        self._exit.set();self.supervisor.stop(reason)
        with self._lock:thread=self._thread;creating=self._creating;unknown=self._unknown
        if thread is not None and thread is not threading.current_thread() and not creating:
            try:thread.join(.8)
            except RuntimeError:unknown=True
        closed=not creating and not unknown and (thread is None or not thread.is_alive())
        if closed:
            with self._lock:self._thread=None
        return dict(self.status(),loop_joined=closed)
    def manual_takeover(self):return self.stop('manual_takeover')
    def close(self):return self.stop('dashboard_shutdown')
