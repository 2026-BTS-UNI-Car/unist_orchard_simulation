"""Single-owner OBSERVE_ONLY source loop, with no actuator or device interface.

The planner, nondestructive latest-input getter and optional clock are injected.
Clock/context labels are caller declarations, never a source qualification.
Without a clock override, the getter must provide fresh comparison times on
each call; original sensor stamps/receipts are never restamped. No scan history
is archived. Stop inhibits publication before waiting for the owned worker.
Hooks are cooperative: a hung getter/planner retains stopping ownership.
"""
import hashlib
import json
import math
import queue
import threading
import time

from dashboard_server.native_owner_lifecycle import NativeOwnerLifecycle, ResourceHooks
from dashboard_server.original_row_age_budget import OriginalRowAgeBudget

NS_MAX = (1 << 63) - 1
QUEUE_LIMIT = 8
EVIDENCE_LIMIT = 128
COMPARISON_JUMP_NS = 50_000_000
AUTHORITY = ("runtime_authority", "motor_command_sent", "autonomy_output_enabled",
             "physical_autonomy_qualified", "execution_eligible")


def require(value, reason):
    if not value:
        raise ValueError(reason)


class _InputUnavailable(Exception):
    """A getter has no current snapshot; this never qualifies retained data."""


def ns(value):
    return type(value) is int and 0 <= value <= NS_MAX


def name(value):
    return type(value) is str and 0 < len(value) <= 128 and value.strip() == value


def finite(value):
    return type(value) in (int, float) and -1e100 <= value <= 1e100 and math.isfinite(value)


def bounded_copy(value):
    """Private primitive snapshot. Nonfinite range values remain unknown rays."""
    count = 0

    def copy(item, depth=0):
        nonlocal count
        count += 1
        require(count <= 10000 and depth <= 12, "observation_input_shape_capacity")
        if type(item) is dict:
            require(len(item) <= 64 and all(type(k) is str and len(k) <= 128 for k in item),
                    "bounded_observation_keys_required")
            return {key: copy(child, depth+1) for key, child in item.items()}
        if type(item) in (list, tuple):
            require(len(item) <= 4096, "observation_array_capacity")
            return [copy(child, depth+1) for child in item]
        if type(item) is str:
            require(len(item) <= 512, "observation_string_capacity")
        elif type(item) is int:
            require(item.bit_length() <= 256, "observation_integer_capacity")
        elif item is not None and type(item) not in (bool, float):
            raise ValueError("observation_primitives_required")
        return item

    result = copy(value)
    require(len(json.dumps(result, allow_nan=True).encode()) <= 256*1024,
            "observation_snapshot_capacity")
    return result


def context_of(row):
    context = row.get("context")
    required = {"source_session", "generation", "source_clock_domain", "receipt_clock_domain"}
    require(type(context) is dict and required <= set(context) and
            set(context) <= required | {"epoch", "reconnect_id"}, "observation_source_context_required")
    require(ns(context["generation"]) and all(name(context[key]) for key in required-{"generation"}) and
            all(name(context[key]) for key in set(context)-required), "observation_source_context_invalid")
    return context


def zero_decision(reason, decision="HOLD"):
    return dict(schema="ubts.autonomy_observation.decision.v1", mode="OBSERVE_ONLY",
                decision=decision, reason=reason, proposed_left_percent=0., proposed_right_percent=0.,
                applied_left_percent=0., applied_right_percent=0., trajectory=[],
                runtime_authority=False, motor_command_sent=False, autonomy_output_enabled=False,
                physical_autonomy_qualified=False, execution_eligible=False)


def checked_decision(value):
    require(type(value) is dict, "observation_planner_decision_required")
    result = bounded_copy(value)
    require(result.get("decision") in ("PROPOSE_FORWARD", "HOLD", "STOP") and name(result.get("reason")),
            "observation_planner_decision_invalid")
    require(all(result.get(key, False) is False for key in AUTHORITY), "observation_authority_forbidden")
    require(all(finite(result.get(key)) and 0 <= result[key] <= 30 for key in
                ("proposed_left_percent", "proposed_right_percent")), "bounded_observation_proposal_required")
    require(all(type(result.get(key)) in (int, float) and result[key] == 0 for key in
                ("applied_left_percent", "applied_right_percent")), "observation_applied_output_forbidden")
    trajectory = result.get("trajectory")
    require(type(trajectory) is list and len(trajectory) <= 128, "observation_trajectory_capacity")
    require(all(type(point) is dict and set(point) == {"x_m", "y_m", "yaw_rad"} and
                all(finite(v) for v in point.values()) for point in trajectory),
            "bounded_observation_trajectory_required")
    # Output is observational even when the supplied planner proposes motion.
    result.update(mode="OBSERVE_ONLY", applied_left_percent=0., applied_right_percent=0.,
                  **{key: False for key in AUTHORITY})
    require(len(json.dumps(result, allow_nan=False).encode()) <= 24*1024,
            "observation_decision_capacity")
    return result


class _ObservationWorker:
    def __init__(self, parent, context):
        self.parent, self.context = parent, context
        self.lock, self.wake = threading.RLock(), threading.Event()
        self.rows = queue.Queue(maxsize=QUEUE_LIMIT)
        self.thread = None
        self.thread_attempted = False
        self.closed = False
        self.source = self.frontier = self.fingerprint = None
        self.stop_reason = None
        self.age = None
        self.received = self.processed = self.decisions = self.duplicates = 0
        self.rejected = self.late_results = self.queue_peak = self.discarded = 0
        self.last_progress = time.monotonic_ns()
        self.preference_version = None
        self.phase = "waiting_input"
        self.wait_reason = "WAIT_INPUT"
        self.latest_input = None
        self.stale_inputs = self.stale_results = 0
        self.waiting_after_sequence = None
        self.planner_initialized = self.reanchor_required = False
        self.comparison_frontier = self.last_wait_record = None
        self.planner_stop_reason = None
        # Diagnostic clocks are relative host MONOTONIC only. They never
        # replace or qualify sensor/source/receipt clocks or freshness checks.
        self.stage_timing = {key: dict(calls=0, total_ns=0, max_ns=0,
            last_ns=None, invalid_samples=0) for key in
            ("poll_input", "post_plan_input", "planner_and_validation")}
        self.provider_unavailable = {key: 0 for key in ("poll_input", "post_plan_input")}
        self.plans_finished = 0
        self.last_plan_sequence = self.last_published_sequence = None
        self.last_planner_counts = {}
        self.last_discard_reason = None
        self.original_age_budget = OriginalRowAgeBudget()

    def pin_age_budget(self,row):
        try:
            pinned=None if self.parent.clock is not None else self.original_age_budget.pin(row)
            self.original_age_budget.observe('PROVIDER_SAMPLED',pinned)
            return pinned
        except Exception:return None

    def record_age_budget(self,phase,pin):
        try:self.original_age_budget.observe(phase,pin)
        except Exception:pass

    def record_stage(self, phase, started):
        elapsed = time.monotonic_ns()-started
        with self.lock:
            value = self.stage_timing[phase]
            value["calls"] = min(NS_MAX, value["calls"]+1)
            if not 0 <= elapsed <= 60_000_000_000:
                value["last_ns"] = None
                value["invalid_samples"] = min(NS_MAX, value["invalid_samples"]+1)
                return
            value["last_ns"] = elapsed
            value["max_ns"] = max(value["max_ns"], elapsed)
            value["total_ns"] = min(NS_MAX, value["total_ns"]+elapsed)

    def read_input(self, phase):
        started = time.monotonic_ns()
        try:
            value = self.parent.input_provider()
            if value is None:
                with self.lock:
                    self.provider_unavailable[phase] = min(NS_MAX, self.provider_unavailable[phase]+1)
            return value
        finally:
            self.record_stage(phase, started)

    def finish_plan_diagnostic(self, sequence, decision):
        # Only bounded non-authoritative counters, never ranges/pose/command.
        field = decision.get("field10")
        counts = {}
        if type(field) is dict:
            for key in ("scans_fed", "imus_fed", "scans_processed", "imu_fed"):
                value = field.get(key)
                if type(value) is int and 0 <= value <= NS_MAX:
                    counts[key] = value
        with self.lock:
            self.plans_finished = min(NS_MAX, self.plans_finished+1)
            self.last_plan_sequence = sequence
            self.last_planner_counts = counts

    def fail(self, reason):
        with self.lock:
            self.stop_reason = self.stop_reason or reason[:128]
        self.context.fail(reason)
        self.wake.set()

    def enqueue(self, row):
        try:
            private = bounded_copy(row)
            require(type(private) is dict, "observation_snapshot_required")
        except Exception as error:
            self.fail(str(error)[:128])
            return False
        if self.context.cancelled:
            with self.lock:
                self.rejected += 1
            return False
        with self.lock:
            bound = None if self.source is None else self.source["context"]
        if bound is not None:
            try:
                require(context_of(private) == bound, "observation_source_context_changed")
            except Exception as error:
                self.fail(str(error)[:128])
                return False
        with self.parent.lifecycle._condition:
            if self.parent.lifecycle._owner is not self.context._owner or self.context.cancelled:
                with self.lock:
                    self.rejected += 1
                return False
            try:
                self.rows.put_nowait(private)
            except queue.Full:
                self.fail("observation_input_queue_overflow")
                return False
            with self.lock:
                self.received += 1
                self.queue_peak = max(self.queue_peak, self.rows.qsize())
        self.wake.set()
        return True

    def read_comparison(self):
        started = time.monotonic_ns()
        try:
            value = self.parent.comparison_provider()
            if value is None:
                with self.lock:
                    self.provider_unavailable["post_plan_input"] = min(
                        NS_MAX, self.provider_unavailable["post_plan_input"]+1)
            return value
        finally:
            self.record_stage("post_plan_input", started)

    def validate_comparison(self, row, comparison):
        require(set(comparison) == {"schema", "context", "scan", "imu", "motion",
                                   "source_now_ns", "receipt_now_ns"} and
                comparison.get("schema") == "ubts.native_observation.control_comparison.v1",
                "observation_scalar_comparison_schema_required")
        require(context_of(comparison) == context_of(row),
                "observation_source_reconnected_during_plan")
        fields = {"scan": {"sequence", "frame_id", "stamp_ns", "receipt_ns"},
                  "imu": {"stamp_ns", "receipt_ns"},
                  "motion": {"previous_scan_stamp_ns", "current_scan_stamp_ns", "receipt_ns"}}
        for key, expected in fields.items():
            value = comparison.get(key)
            require(type(value) is dict and set(value) == expected and
                    all(ns(value[k]) for k in expected-{"frame_id"}),
                    "observation_scalar_comparison_endpoints_required")
        scan, imu, motion = comparison["scan"], comparison["imu"], comparison["motion"]
        require(scan["frame_id"] == row["scan"]["frame_id"] and
                scan["sequence"] >= row["scan"]["sequence"],
                "observation_source_rewind_during_plan")
        require(imu["stamp_ns"] <= scan["stamp_ns"] and
                motion["previous_scan_stamp_ns"] < motion["current_scan_stamp_ns"] == scan["stamp_ns"] and
                motion["receipt_ns"] == scan["receipt_ns"],
                "observation_scalar_comparison_causal_endpoints_required")
        require(all(ns(comparison.get(k)) for k in ("source_now_ns", "receipt_now_ns")) and
                comparison["source_now_ns"] >= scan["stamp_ns"] and
                comparison["source_now_ns"] >= imu["stamp_ns"] and
                all(comparison["receipt_now_ns"] >= comparison[k]["receipt_ns"]
                    for k in ("scan", "imu", "motion")),
                "observation_scalar_comparison_future_endpoint")
        if scan["sequence"] == row["scan"]["sequence"]:
            require(all(row.get(key) is not None and
                        all(value == row[key].get(k) for k, value in comparison[key].items())
                        for key in fields), "observation_same_comparison_sequence_changed")
        else:
            require(scan["stamp_ns"] > row["scan"]["stamp_ns"] and
                    scan["receipt_ns"] > row["scan"]["receipt_ns"] and
                    motion["previous_scan_stamp_ns"] >= row["scan"]["stamp_ns"] and
                    (row.get("imu") is None or
                     (imu["stamp_ns"] >= row["imu"]["stamp_ns"] and
                      imu["receipt_ns"] >= row["imu"]["receipt_ns"] and
                      (imu["stamp_ns"] != row["imu"]["stamp_ns"] or
                       imu["receipt_ns"] == row["imu"]["receipt_ns"]))),
                    "observation_scalar_comparison_original_timing_rewind")

    def current_clock(self, row, *, refresh=False):
        context = context_of(row)
        comparison = row
        if refresh and self.parent.input_provider is not None:
            scalar = getattr(self.parent, "comparison_provider", None) is not None
            comparison = self.read_comparison() if scalar else self.read_input("post_plan_input")
            if comparison is None:
                raise _InputUnavailable()
            if scalar:
                require(type(comparison) is dict, "observation_scalar_comparison_required")
                self.validate_comparison(row, comparison)
            require(type(comparison) is dict and context_of(comparison) == context,
                    "observation_source_reconnected_during_plan")
            require(type(comparison.get("scan")) is dict and
                    comparison["scan"].get("frame_id") == row["scan"]["frame_id"] and
                    ns(comparison["scan"].get("sequence")) and
                    comparison["scan"]["sequence"] >= row["scan"]["sequence"],
                    "observation_source_rewind_during_plan")
        if self.parent.clock is not None:
            current = self.parent.clock()
        else:
            if refresh:
                require(self.parent.input_provider is not None, "observation_clock_provider_required")
            current = dict(source_now_ns=comparison.get("source_now_ns"),
                           receipt_now_ns=comparison.get("receipt_now_ns"),
                           source_clock_domain=context["source_clock_domain"],
                           receipt_clock_domain=context["receipt_clock_domain"])
        require(type(current) is dict and ns(current.get("source_now_ns")) and
                ns(current.get("receipt_now_ns")) and
                current.get("source_clock_domain") == context["source_clock_domain"] and
                current.get("receipt_clock_domain") == context["receipt_clock_domain"],
                "observation_comparison_clock_invalid")
        return current

    def validate(self, row, current):
        context = context_of(row)
        scan, imu, motion = row.get("scan"), row.get("imu"), row.get("motion")
        require(type(scan) is dict and ns(scan.get("sequence")) and name(scan.get("frame_id")),
                "observation_scan_identity_required")
        require(all(ns(scan.get(key)) for key in ("stamp_ns", "receipt_ns")),
                "observation_scan_timing_required")
        age = dict(scan_source_ns=current["source_now_ns"]-scan["stamp_ns"],
                   scan_receipt_ns=current["receipt_now_ns"]-scan["receipt_ns"],
                   imu_source_ns=None, imu_receipt_ns=None, motion_receipt_ns=None)
        if imu is not None:
            require(type(imu) is dict and all(ns(imu.get(key)) for key in ("stamp_ns", "receipt_ns")),
                    "observation_IMU_timing_required")
            require(imu["stamp_ns"] <= scan["stamp_ns"], "observation_IMU_not_causal")
            age.update(imu_source_ns=current["source_now_ns"]-imu["stamp_ns"],
                       imu_receipt_ns=current["receipt_now_ns"]-imu["receipt_ns"])
        if motion is not None:
            require(type(motion) is dict and ns(motion.get("receipt_ns")) and
                    ns(motion.get("previous_scan_stamp_ns")) and ns(motion.get("current_scan_stamp_ns")) and
                    motion["previous_scan_stamp_ns"] < motion["current_scan_stamp_ns"] == scan["stamp_ns"],
                    "observation_motion_endpoint_invalid")
            age["motion_receipt_ns"] = current["receipt_now_ns"]-motion["receipt_ns"]
        require(all(value >= 0 for value in age.values() if value is not None),
                "observation_input_stale_or_future")
        with self.lock:
            clocks = (current["source_now_ns"], current["receipt_now_ns"])
            require(self.comparison_frontier is None or
                    all(a >= b for a, b in zip(clocks, self.comparison_frontier)),
                    "observation_comparison_clock_reversal")
            require(self.comparison_frontier is None or abs(
                    (clocks[0]-self.comparison_frontier[0])-
                    (clocks[1]-self.comparison_frontier[1])) <= COMPARISON_JUMP_NS,
                    "observation_comparison_clock_jump")
            self.comparison_frontier = clocks
        source = dict(context=bounded_copy(context), scan_frame_id=scan["frame_id"],
                      scan_sequence=scan["sequence"], scan_stamp_ns=scan["stamp_ns"],
                      scan_receipt_ns=scan["receipt_ns"], source_qualification_asserted=False)
        return source, age

    def expired(self, age):
        return any(value > self.parent.input_age_ns for value in age.values() if value is not None)

    def wait_input(self, row, source, age, *, reason="WAIT_STALE_INPUT", post_plan=False):
        """Publish no proposal; a newer fresh sequence must reacquire planning."""
        seq = row["scan"]["sequence"]
        with self.lock:
            self.waiting_after_sequence = max(seq, self.waiting_after_sequence or seq)
            self.reanchor_required = True
            self.wait_reason, self.phase, self.age = reason, "waiting_input", age
            if reason == "WAIT_STALE_INPUT":
                self.stale_inputs += 1
                self.stale_results += int(post_plan)
            key = (reason, seq)
            publish = key != self.last_wait_record
            self.last_wait_record = key
            latched_stop = self.planner_stop_reason
        if publish and not self.context.cancelled:
            self.context.publish(zero_decision(latched_stop, "STOP") if latched_stop else zero_decision(reason), dict(source=source, input_age_ns=age,
                discarded_inflight=post_plan, latest_source_sampling=True, physical_authority=False))

    def process(self, row):
        current = self.current_clock(row)
        source, age = self.validate(row, current)
        scan, imu = row["scan"], row.get("imu")
        fingerprint = hashlib.sha256(json.dumps({key: value for key, value in row.items()
            if key not in ("source_now_ns", "receipt_now_ns")}, sort_keys=True, allow_nan=True).encode()).hexdigest()
        with self.lock:
            previous = self.frontier
            duplicate = False
            if self.source is not None:
                require(source["context"] == self.source["context"] and
                        source["scan_frame_id"] == self.source["scan_frame_id"], "observation_source_context_changed")
            if previous is not None:
                require(scan["sequence"] >= previous[0], "observation_scan_sequence_rewind")
                if scan["sequence"] == previous[0]:
                    require(fingerprint == self.fingerprint, "observation_same_sequence_changed")
                    self.duplicates += 1
                    self.age = age
                    duplicate = True
                require(duplicate or (scan["stamp_ns"] > previous[1] and scan["receipt_ns"] > previous[2] and
                        (imu is None or previous[3] is None or
                         (imu["stamp_ns"] >= previous[3] and imu["receipt_ns"] >= previous[4]))),
                        "observation_original_timing_rewind")
                require(imu is None or previous[3] is None or
                        imu["stamp_ns"] != previous[3] or imu["receipt_ns"] == previous[4],
                        "observation_duplicate_IMU_restamped")
            self.source, self.age, self.fingerprint = source, age, fingerprint
            self.frontier = (scan["sequence"], scan["stamp_ns"], scan["receipt_ns"],
                             (None if previous is None else previous[3]) if imu is None else imu["stamp_ns"],
                             (None if previous is None else previous[4]) if imu is None else imu["receipt_ns"])
            self.latest_input = row
            self.processed += int(not duplicate)
            self.last_progress = time.monotonic_ns()
            waiting_sequence = self.waiting_after_sequence
        # Identity, ordering, original endpoints and future clocks are checked
        # even for an expired row. Only positive age expiry is recoverable.
        if self.expired(age):
            self.wait_input(row, source, age)
            return
        if duplicate or (waiting_sequence is not None and scan["sequence"] <= waiting_sequence):
            return
        if not self.planner_initialized:
            if callable(getattr(self.parent.planner, "reset", None)):
                self.parent.planner.reset(bounded_copy(source["context"]))
            self.planner_initialized = True
        elif self.reanchor_required:
            reanchor = getattr(self.parent.planner, "reanchor_after_expiry", None)
            if callable(reanchor):
                # This guarded seam must preserve every planner Stop/fault.
                require(type(reanchor(bounded_copy(source["context"]))) is bool,
                        "typed_planner_reanchor_result_required")
        self.reanchor_required = False
        with self.lock:
            self.wait_reason = None
        with self.parent.lock:
            cruise, version = self.parent.cruise_percent, self.parent.preference_version
        planning = bounded_copy(row)
        planning.update(source_now_ns=current["source_now_ns"], receipt_now_ns=current["receipt_now_ns"])
        age_budget_pin = self.pin_age_budget(planning)
        if imu is None:
            decision = zero_decision("WAIT_IMU")
        elif row.get("motion") is None:
            decision = zero_decision("WAIT_LOCALIZATION")
        else:
            self.record_age_budget("PRE_PLAN", age_budget_pin)
            started = time.monotonic_ns()
            try:
                plan = getattr(self.parent.planner, "plan", None)
                decision = checked_decision(plan(planning, cruise_percent=cruise) if callable(plan)
                                            else self.parent.planner(planning, cruise))
            finally:
                self.record_stage("planner_and_validation", started)
                self.record_age_budget("POST_PLAN_BEFORE_RPC", age_budget_pin)
            self.finish_plan_diagnostic(scan["sequence"], decision)
        if self.context.cancelled:
            with self.lock:
                self.last_discard_reason = "CANCELLED"
                self.late_results += 1
            return
        try:
            try:
                after = self.current_clock(row, refresh=True)
            finally:
                self.record_age_budget("POST_RPC", age_budget_pin)
        except _InputUnavailable:
            with self.lock:
                self.last_discard_reason = "POST_PLAN_INPUT_UNAVAILABLE"
            self.wait_input(row, source, None, reason="WAIT_INPUT", post_plan=True)
            return
        _, final_age = self.validate(row, after)
        if self.expired(final_age):
            with self.lock:
                self.last_discard_reason = "POST_PLAN_POSITIVE_AGE_EXPIRY"
            self.wait_input(row, source, final_age, post_plan=True)
            return
        with self.parent.lock:
            unchanged = version == self.parent.preference_version
        if not unchanged or self.context.cancelled:
            with self.lock:
                self.last_discard_reason = "PREFERENCE_CHANGED" if not unchanged else "CANCELLED"
                self.late_results += 1
            return
        evidence = dict(source=source, input_age_ns=final_age, cruise_percent=cruise,
                        preference_version=version, latest_source_sampling=True,
                        physical_authority=False)
        if self.context.publish(decision, evidence):
            with self.lock:
                self.decisions += 1
                self.last_published_sequence = scan["sequence"]
                self.last_discard_reason = None
                if decision["decision"] == "STOP":
                    self.planner_stop_reason = self.planner_stop_reason or decision["reason"]
                self.preference_version, self.age = version, final_age
                self.waiting_after_sequence, self.last_wait_record = None, None
                self.phase = "waiting_imu" if imu is None else "waiting_localization" if row.get("motion") is None else "observing"

    def run(self):
        try:
            while not self.context.cancelled:
                poll_started = time.monotonic()
                if self.parent.input_provider is not None:
                    supplied = self.read_input("poll_input")
                    if supplied is not None:
                        if not self.enqueue(supplied):
                            break
                    else:
                        with self.lock:
                            old_row, source = self.latest_input, self.source
                            reason = "WAIT_STALE_INPUT" if self.wait_reason == "WAIT_STALE_INPUT" else "WAIT_INPUT"
                            if old_row is None:
                                self.wait_reason, self.phase, self.age = reason, "waiting_input", None
                        if old_row is not None:
                            self.wait_input(old_row, source, None, reason=reason)
                try:
                    row = self.rows.get_nowait()
                except queue.Empty:
                    if self.parent.input_provider is None and self.latest_input is not None:
                        source, current_age = self.validate(self.latest_input, self.current_clock(self.latest_input))
                        with self.lock:
                            self.age = current_age
                        if self.expired(current_age):
                            self.wait_input(self.latest_input, source, current_age)
                    self.wake.wait(self.parent.poll_interval_s
                        if self.parent.input_provider is None else max(0.,
                            self.parent.poll_interval_s-(time.monotonic()-poll_started)))
                    self.wake.clear()
                    continue
                self.process(row)
                if self.parent.input_provider is not None:
                    self.wake.clear()
                    self.context.wait_cancel(max(0., self.parent.poll_interval_s-
                        (time.monotonic()-poll_started)))
        except BaseException as error:
            self.fail((type(error).__name__ + ": " + str(error))[:128])
        finally:
            try:
                method = getattr(self.parent.planner, "manual_takeover", None) if self.stop_reason == "manual_takeover" else None
                if callable(method):
                    method()
                elif callable(getattr(self.parent.planner, "stop", None)):
                    self.parent.planner.stop(self.stop_reason or "observation_worker_finished")
            except BaseException as error:
                self.fail((type(error).__name__ + ": " + str(error))[:128])

    def request_stop(self):
        self.wake.set()  # context cancellation already inhibits publication

    def join(self, timeout):
        if self.thread is None:
            return not self.thread_attempted
        self.thread.join(timeout)
        return not self.thread.is_alive()

    def close(self):
        if (self.thread is None and self.thread_attempted) or (self.thread is not None and self.thread.is_alive()):
            return False
        while True:
            try:
                self.rows.get_nowait()
                self.discarded += 1
            except queue.Empty:
                break
        self.closed = True
        self.latest_input = None
        return True

    def snapshot(self):
        with self.lock:
            return dict(source=bounded_copy(self.source), input_age=bounded_copy(self.age),
                processing_diagnostics=dict(schema="ubts.observation.processing_diagnostics.v1",
                    original_row_age_budget=self.original_age_budget.snapshot(),
                    clock_role="HOST_MONOTONIC_RELATIVE_DURATION_ONLY",
                    physical_authority=False, input_source_fields_modified=False,
                    plans_finished=self.plans_finished,
                    last_plan_sequence=self.last_plan_sequence,
                    last_published_sequence=self.last_published_sequence,
                    planner_counter_phase="AFTER_PLAN_BEFORE_POSTCHECK_NOT_PUBLICATION",
                    last_planner_counts=bounded_copy(self.last_planner_counts),
                    last_discard_reason=self.last_discard_reason,
                    provider_unavailable=bounded_copy(self.provider_unavailable),
                    stage_timing_ns=bounded_copy(self.stage_timing)),
                stop_reason=self.stop_reason, phase=self.phase, preference_version=self.preference_version,
                planner_stop_reason=self.planner_stop_reason,
                wait_reason=self.wait_reason,
                counters=dict(received=self.received, processed=self.processed, decisions=self.decisions,
                    duplicates=self.duplicates, rejected=self.rejected, late_results=self.late_results,
                    discarded_after_cleanup=self.discarded, stale_inputs=self.stale_inputs,
                    stale_results=self.stale_results),
                queue=dict(depth=self.rows.qsize(), limit=QUEUE_LIMIT, peak=self.queue_peak),
                resources_closed=self.closed)


class AutonomyObservation:
    """Explicit observation only. Getter returns the latest snapshot, not a pop."""
    def __init__(self, planner=None, *, input_provider=None, comparison_provider=None, clock=None, cruise_percent=10.,
                 input_age_ns=250_000_000, poll_interval_s=.05, thread_factory=threading.Thread):
        require(planner is None or callable(planner), "injected_observation_planner_required")
        require(input_provider is None or callable(input_provider), "latest_input_getter_required")
        require(comparison_provider is None or (callable(comparison_provider) and callable(input_provider)),
                "comparison_requires_explicit_latest_input_getter")
        require(clock is None or callable(clock), "injected_observation_clock_required")
        require(ns(input_age_ns) and 0 < input_age_ns <= 5_000_000_000 and
                finite(poll_interval_s) and .001 <= poll_interval_s <= 1 and callable(thread_factory),
                "bounded_observation_timing_required")
        self.lock, self.resource = threading.RLock(), None
        self.pending_stop = None
        self.planner, self.input_provider, self.clock = planner, input_provider, clock
        self.comparison_provider = comparison_provider
        self.input_age_ns, self.poll_interval_s, self.thread_factory = input_age_ns, float(poll_interval_s), thread_factory
        self.cruise_percent, self.preference_version = self.cruise(cruise_percent), 0
        self.lifecycle = NativeOwnerLifecycle(self.factory, max_records=EVIDENCE_LIMIT,
                                            max_bytes=1024*1024, max_record_bytes=32768)

    @staticmethod
    def cruise(value):
        require(finite(value) and 0 <= value <= 30, "bounded_cruise_percent_required")
        return float(value)  # percentage preference, never a calibrated speed

    def factory(self, context):
        resource = _ObservationWorker(self, context)
        with self.lock:
            self.resource = resource
            if self.pending_stop is not None and self.pending_stop[0] == context.generation:
                resource.stop_reason = self.pending_stop[1]
        try:
            if not context.cancelled:
                resource.thread_attempted = True
                resource.thread = self.thread_factory(target=resource.run,
                    name="autonomy-observation", daemon=False)
                resource.thread.start()
        except BaseException as error:
            resource.fail((type(error).__name__ + ": " + str(error))[:128])
        return ResourceHooks(resource.request_stop, resource.join, resource.close)

    def start(self, *, cruise_percent=None, session=None):
        require(self.planner is not None, "observation_planner_not_configured")
        if cruise_percent is not None:
            self.set_cruise_percent(cruise_percent)
        self.lifecycle.start(session)
        return self.status()

    def set_cruise_percent(self, value):
        value = self.cruise(value)
        with self.lock:
            if self.cruise_percent != value:
                self.cruise_percent, self.preference_version = value, self.preference_version+1
        return self.status()

    def set_speed(self, speed_percent):
        return self.set_cruise_percent(speed_percent)

    def stop(self, reason="operator_stop", timeout=1.):
        require(name(reason), "bounded_observation_stop_reason_required")
        require(finite(timeout) and 0 <= timeout <= 60, "bounded_observation_stop_timeout_required")
        deadline = time.monotonic()+timeout
        with self.lifecycle._condition:
            owner = self.lifecycle._owner
            with self.lock:
                resource = self.resource
                if owner is not None:
                    if self.pending_stop is None or self.pending_stop[0] != owner.generation:
                        self.pending_stop = (owner.generation, reason)
            if owner is not None and resource is not None and resource.context.generation == owner.generation:
                with resource.lock:
                    resource.stop_reason = resource.stop_reason or reason
            if owner is not None:
                owner.cancel.set()  # publication fence before releasing the control lock
                self.lifecycle._state = "stopping"
        self.lifecycle._stop_owner(owner, deadline)
        return self.status()

    def manual_takeover(self, timeout=1.):
        return self.stop("manual_takeover", timeout)

    def input_token(self):
        status = self.lifecycle.status()
        return dict(session=status.session, generation=status.generation)

    def submit(self, snapshot, *, session, generation):
        """Callbacks must bind the token captured at this explicit Start."""
        with self.lifecycle._condition:
            owner = self.lifecycle._owner
            with self.lock:
                resource = self.resource
            admitted = (owner is not None and not owner.cancel.is_set() and resource is not None and
                resource.context.generation == owner.generation and session == owner.session and
                type(generation) is int and generation == owner.generation)
        if not admitted:
            return False
        return resource.enqueue(snapshot)

    def evidence(self):
        return self.lifecycle.evidence()

    def status(self):
        owner = self.lifecycle.status()
        with self.lock:
            resource, cruise, version = self.resource, self.cruise_percent, self.preference_version
        details = dict(source=None, input_age=None, processing_diagnostics=None, stop_reason=None, planner_stop_reason=None, phase="waiting_input", wait_reason="WAIT_INPUT",
            counters=dict(received=0, processed=0, decisions=0, duplicates=0, rejected=0, late_results=0,
                          discarded_after_cleanup=0, stale_inputs=0, stale_results=0), queue=dict(depth=0, limit=QUEUE_LIMIT, peak=0),
            preference_version=None, resources_closed=False) if resource is None else resource.snapshot()
        latest = None if owner.latest is None else json.loads(owner.latest.diagnostic_json)
        active = owner.owned and not owner.cancelled
        current = latest or zero_decision("WAIT_INPUT")
        if owner.first_failure_reason or details["stop_reason"] or details["planner_stop_reason"]:
            current = zero_decision(details["stop_reason"] or (owner.first_failure_reason or details["planner_stop_reason"])[:128], "STOP")
        elif active and details["wait_reason"]:
            current = zero_decision(details["wait_reason"])
        elif active and latest is not None and details["preference_version"] is None:
            current = zero_decision("WAIT_RESULT")
        elif active and latest is not None and details["preference_version"] != version:
            current = zero_decision("PREFERENCE_CHANGED_WAITING_INPUT")
        state = "fault" if owner.first_failure_reason else details["phase"] if active else owner.state
        return dict(schema="ubts.autonomy_observation.status.v1", mode="OBSERVE_ONLY", state=state,
            active=active, owned=owner.owned, session=owner.session, generation=owner.generation,
            stop_reason=details["stop_reason"], first_failure_reason=owner.first_failure_reason,
            planner_stop_reason=details["planner_stop_reason"],
            current_decision=current, trajectory=current["trajectory"], latest_decision=latest,
            latest_stale=owner.latest_stale or bool(details["wait_reason"]), source=details["source"], input_age=details["input_age"],
            counters=details["counters"], progress=details["counters"], queue=details["queue"],
            processing_diagnostics=details["processing_diagnostics"],
            cruise_percent=cruise, speed_percent=cruise, cruise_preference_is_estimate=True,
            calibrated_speed_m_s=None, retained_evidence=owner.retained_records,
            evidence_limit=EVIDENCE_LIMIT, retained_evidence_bytes=owner.retained_bytes,
            evicted_evidence=owner.evicted_records, resources_closed=details["resources_closed"],
            applied_left_percent=0., applied_right_percent=0., runtime_authority=False,
            motor_command_sent=False, autonomy_output_enabled=False, physical_autonomy_qualified=False,
            execution_eligible=False, body_feedback_authority=False)
