"""Preparatory single-owner lifecycle; no ROS, process spawn, socket or motor IO.

An explicitly supplied factory receives an OwnerContext and returns ResourceHooks
for the exact resources it created. It must honor cancellation, track partial
creation, and make request_stop/close bounded and join(timeout) cooperative.
Unknown creation/cleanup retains ownership. This module cannot preempt a hung
hook or independently prove its resource claims; an external supervisor is
required for that boundary. It is not wired to physical_shadow_runner.

Only diagnostic retention is rolling. It is not a sensor ingress queue or a
clock/frame/calibration/admission validator. No returned value grants authority.
"""
from collections import deque
from dataclasses import dataclass, field
import json
import math
import threading
import time
import uuid


def _timeout(value):
    if type(value) not in (int, float) or not 0 <= value <= 60 or not math.isfinite(value):
        raise ValueError("bounded_timeout_required")
    return float(value)


def _label(value):
    return type(value) is str and 0 < len(value) <= 128 and value.strip() == value


def _reason(error):
    if isinstance(error, BaseException):
        try:
            return (type(error).__name__ + ": " + str(error))[:256]
        except BaseException:
            return type(error).__name__[:256]
    return error[:256] if _label(error) else "worker_failed"


def _json_bytes(value, cap):
    # Snapshot only primitive JSON shapes before allocating a serialized copy.
    # These are byte/work limits, not source timestamp or calibration assertions.
    nodes = 0
    flags = {"runtime_authority", "motor_command_sent", "autonomy_output_enabled",
             "execution_eligible", "physical_outputs"}

    def check(item, depth=0):
        nonlocal nodes
        nodes += 1
        if nodes > 4096 or depth > 16:
            raise ValueError("diagnostic_shape_capacity")
        if type(item) is dict:
            if len(item) > 4096:
                raise ValueError("diagnostic_shape_capacity")
            result = {}
            for key, child in item.items():
                if type(key) is not str or len(key) > cap:
                    raise ValueError("diagnostic_key_required")
                frozen = check(child, depth + 1)
                if key in flags and frozen is not False:
                    raise ValueError("diagnostic_authority_forbidden")
                result[key] = frozen
            return result
        elif type(item) is list:
            if len(item) > 4096:
                raise ValueError("diagnostic_shape_capacity")
            return [check(child, depth + 1) for child in item]
        elif type(item) is str:
            if len(item) > cap:
                raise ValueError("diagnostic_byte_capacity")
        elif type(item) is int:
            if item.bit_length() > 256:
                raise ValueError("diagnostic_number_capacity")
        elif type(item) is float:
            if not math.isfinite(item):
                raise ValueError("diagnostic_nonfinite")
        elif item is not None and type(item) is not bool:
            raise ValueError("diagnostic_JSON_primitives_required")
        return item

    snapshot = check(value)
    # iterencode bounds accumulated output without first building a giant JSON
    # string. A string fragment is bounded by the primitive checks above.
    result = bytearray()
    for part in json.JSONEncoder(allow_nan=False, ensure_ascii=True,
                                 sort_keys=True, separators=(",", ":")).iterencode(snapshot):
        raw = part.encode("utf8")
        if len(result) + len(raw) > cap:
            raise ValueError("diagnostic_byte_capacity")
        result.extend(raw)
    return bytes(result)


@dataclass(frozen=True)
class ResourceHooks:
    """Trusted exact-resource hooks; close is called once, after join is True."""
    request_stop: object
    join: object
    close: object

    def valid(self):
        return all(callable(value) for value in
                   (self.request_stop, self.join, self.close))


@dataclass(frozen=True)
class DiagnosticRecord:
    session: str
    generation: int
    sequence: int
    diagnostic_json: bytes
    evidence_json: bytes
    runtime_authority: bool = field(default=False, init=False)
    motor_command_sent: bool = field(default=False, init=False)
    autonomy_output_enabled: bool = field(default=False, init=False)


@dataclass(frozen=True)
class LifecycleStatus:
    state: str
    session: str | None
    generation: int
    owned: bool
    creating: bool
    cleanup_active: bool
    cleanup_unknown: bool
    cancelled: bool
    first_failure_reason: str | None
    latest: DiagnosticRecord | None
    latest_stale: bool
    retained_records: int
    retained_bytes: int
    evicted_records: int
    rejected_publications: int
    runtime_authority: bool = field(default=False, init=False)
    motor_command_sent: bool = field(default=False, init=False)
    autonomy_output_enabled: bool = field(default=False, init=False)
    integration_pending: bool = field(default=True, init=False)


@dataclass
class _Owner:
    session: str
    generation: int
    cancel: threading.Event = field(default_factory=threading.Event)
    creating: bool = True
    resource: ResourceHooks | None = None
    unknown: bool = False
    cleaning: bool = False
    stop_attempted: bool = False
    close_attempted: bool = False
    sequence: int = 0


@dataclass(frozen=True)
class OwnerContext:
    """A generation-bound publisher/cancel handle, not a sensor identity proof."""
    session: str
    generation: int
    _lifecycle: object = field(repr=False)
    _owner: object = field(repr=False)

    @property
    def cancelled(self):
        return self._owner.cancel.is_set()

    def wait_cancel(self, timeout):
        return self._owner.cancel.wait(_timeout(timeout))

    def publish(self, diagnostic, evidence=None):
        return self._lifecycle._publish(self._owner, diagnostic, evidence)

    def fail(self, error):
        return self._lifecycle._fail(self._owner, error)

    def run(self, worker, *args, **kwargs):
        """Injected workers use this wrapper to expose exceptions; no thread starts."""
        try:
            if not self.cancelled:
                worker(self, *args, **kwargs)
        except BaseException as error:
            self.fail(error)
            return False
        return True


class NativeOwnerLifecycle:
    """Explicit single factory lifetime, with retained ownership on uncertainty.

    start() is synchronous factory activation; concurrent starts deduplicate.
    "running" means the injected resource factory returned, not sensor READY.
    stop(timeout) inhibits publication immediately and waits for creating/other
    cleanup with one deadline, then calls supplied hooks outside the state lock.
    A timeout/False/exception never authorizes another owner. Call stop again to
    reconcile a late factory return or a previously unjoined resource. A close
    failure is permanently unknown because close is never repeated.
    A factory that raises after partial allocation cannot be reconciled here;
    it must instead return hooks tracking those exact partial resources.
    """
    def __init__(self, factory, *, max_records=128, max_bytes=1048576,
                 max_record_bytes=16384):
        if not callable(factory):
            raise ValueError("injected_factory_required")
        if (type(max_records) is not int or not 1 <= max_records <= 4096
                or type(max_bytes) is not int or not 1 <= max_bytes <= 8388608
                or type(max_record_bytes) is not int
                or not 1 <= max_record_bytes <= max_bytes):
            raise ValueError("bounded_retention_limits_required")
        self._factory = factory
        self._max_records, self._max_bytes = max_records, max_bytes
        self._max_record_bytes = max_record_bytes
        self._condition = threading.Condition(threading.RLock())
        self._publication_lock = threading.Lock()
        self._owner = None
        self._session, self._generation = None, 0
        self._state, self._failure = "idle", None
        self._records, self._bytes = deque(), 0
        self._latest = None
        self._evicted = self._rejected = 0

    def status(self):
        with self._condition:
            o = self._owner
            return LifecycleStatus(self._state, self._session, self._generation,
                o is not None, bool(o and o.creating), bool(o and o.cleaning),
                bool(o and o.unknown), bool(o and o.cancel.is_set()), self._failure,
                self._latest, self._latest is not None and
                (o is None or o.cancel.is_set() or self._latest.generation != o.generation),
                len(self._records), self._bytes, self._evicted, self._rejected)

    def evidence(self):
        with self._condition:
            return tuple(self._records)

    def _failure_locked(self, reason):
        self._failure = self._failure or reason

    def _fail(self, owner, error):
        reason = _reason(error)
        with self._condition:
            if self._owner is not owner:
                return False
            self._failure_locked(reason)
            owner.cancel.set()
            self._state = "stopping"
            self._condition.notify_all()
            return True

    def start(self, session=None):
        if session is not None and not _label(session):
            raise ValueError("bounded_session_label_required")
        with self._condition:
            if self._owner is not None:
                return self.status()  # includes stopping/unknown ownership
            if self._generation >= (1 << 63) - 1:
                raise ValueError("generation_capacity")
            self._generation += 1
            self._session = session if session is not None else uuid.uuid4().hex
            owner = _Owner(self._session, self._generation)
            self._owner, self._state, self._failure = owner, "starting", None
            context = OwnerContext(owner.session, owner.generation, self, owner)
        try:
            resource = self._factory(context)
            if type(resource) is not ResourceHooks or not resource.valid():
                raise ValueError("exact_resource_hooks_required")
        except BaseException as error:
            with self._condition:
                owner.creating, owner.unknown = False, True
                self._failure_locked(_reason(error))
                owner.cancel.set()
                self._state = "stopping"
                self._condition.notify_all()
            return self.status()  # factory absence/partial allocation unproved
        with self._condition:
            owner.resource, owner.creating = resource, False
            self._state = "stopping" if owner.cancel.is_set() else "running"
            self._condition.notify_all()
        if owner.cancel.is_set():
            return self._stop_owner(owner, time.monotonic())
        return self.status()

    def _publish(self, owner, diagnostic, evidence):
        with self._condition:
            if self._owner is not owner or owner.cancel.is_set():
                self._rejected += 1
                return False
        if not self._publication_lock.acquire(blocking=False):
            with self._condition:
                self._rejected += 1
            return False
        try:
            data = _json_bytes(diagnostic, self._max_record_bytes)
            proof = b"" if evidence is None else _json_bytes(evidence, self._max_record_bytes)
            size = len(data) + len(proof)
            if size > self._max_record_bytes:
                raise ValueError("diagnostic_record_capacity")
            with self._condition:
                if self._owner is not owner or owner.cancel.is_set():
                    self._rejected += 1
                    return False
                if owner.sequence >= (1 << 63) - 1:
                    raise ValueError("diagnostic_sequence_capacity")
                owner.sequence += 1
                record = DiagnosticRecord(owner.session, owner.generation,
                                          owner.sequence, data, proof)
                while self._records and (len(self._records) >= self._max_records
                                         or self._bytes + size > self._max_bytes):
                    old = self._records.popleft()
                    self._bytes -= len(old.diagnostic_json) + len(old.evidence_json)
                    self._evicted += 1
                self._records.append(record)
                self._bytes += size
                self._latest = record
                return True
        finally:
            self._publication_lock.release()

    def stop(self, timeout=2.):
        deadline = time.monotonic() + _timeout(timeout)
        with self._condition:
            owner = self._owner
        return self._stop_owner(owner, deadline)

    def _stop_owner(self, owner, deadline):
        # Bind cleanup to the captured generation. A late Start A must never
        # inhibit or clean a newer owner B after another caller finished A.
        with self._condition:
            if owner is None or self._owner is not owner:
                return self.status()
            owner.cancel.set()  # visible before waiting for factory/resource IO
            self._state = "stopping"
            while owner.creating or owner.cleaning:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return self.status()
                self._condition.wait(remaining)
                if self._owner is not owner:
                    return self.status()
            if owner.unknown or owner.resource is None or owner.close_attempted:
                return self.status()
            owner.cleaning = True
            request = not owner.stop_attempted
            owner.stop_attempted = True
        try:
            if request:
                try:
                    owner.resource.request_stop()
                except BaseException as error:
                    self._fail(owner, error)
            try:
                joined = owner.resource.join(max(0., deadline - time.monotonic()))
            except BaseException as error:
                self._fail(owner, error)
                joined = False
            if joined is True:
                with self._condition:
                    owner.close_attempted = True
                try:
                    closed = owner.resource.close()
                except BaseException as error:
                    self._fail(owner, error)
                    closed = False
                with self._condition:
                    if closed is True:
                        self._owner = None
                        self._state = "failed" if self._failure else "stopped"
                    else:
                        owner.unknown = True
                        self._failure_locked("resource_close_unproved")
        finally:
            with self._condition:
                owner.cleaning = False
                self._condition.notify_all()
        return self.status()


if __name__ == "__main__":
    print(json.dumps(dict(default_inert=True, integration_pending=True,
                         ROS_initialized=False, process_started=False,
                         motor_command_sent=False, runtime_authority=False)))
