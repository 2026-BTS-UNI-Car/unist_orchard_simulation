"""A13 truth isolation, static part for the F-SIM (design 1.4 rule 1, 2.6, 7.2 A13 F; 8.3 P2 and P4).

AST check of the Python sources in SCOPE: controller, estimator and executor-side code (role 'decision') and the
code they share ('shared') must never read SIM truth; plant-side, evaluator and test code may, but only through
an explicit allowlist (ROLE_TABLE, ROLE_PATTERNS); a file in scope that reads truth and has no role is an error
('unclassified'). Comments and docstrings are not code and are not checked.

Truth sources (design 2.6, A13 criterion, decision R11):
  truth_field        backend_api.TRUTH_OUTPUT_FIELDS (true wheel angles, motor-side angle, applied_steer_pwm, model
                     flags) as an attribute or a string (key, getattr name)
  truth_key          a string 'truth_*' (the fast_sim trace truth columns)
  truth_contract     the names TRUTH_OUTPUT_FIELDS, ModelFlag, MODEL_FLAGS, MODEL_FLAG_NAMES
  truth_method       a call of .truth(), .rear_axle_pose(), .run_info(), .servo_view_rad(), .lidar_pose() (the true
                     sensor pose), or of the world truth API .corridor_truth(), .pose_in_field(), .to_field(),
                     .clearance(), .clearances(), .class_at()
  plant_reference    a name or attribute segment containing 'plant' (the plant object, its x / y / yaw / v / w /
                     distance_m state and everything else on it), or the subscript key 'plant'
  world_reference    a name or attribute segment that is exactly 'world' (the SIM world: geometry, map, labels)
  truth_topic        a ROS topic string /odom, /tf (dynamic), /joint_states, /ground_truth/odom, /sim/truth/*,
                     /gps/*, or any string containing 'ground_truth' or the world label 'lane_centers_y'
  truth_import       an import of a plant / world / runner / evaluator / truth-fed driver / scene module or of the
                     Gazebo bindings gz.* (TRUTH_MODULES; steering.scene_spec holds the true start pose and the world of
                     a scene), or of the plant names of a mixed module (PARTIAL_TRUTH_MODULES: field_response.FieldPlant;
                     its nominal drive curve forward_curve and the parameter tables are profile values, not truth);
                     also importlib.import_module('<truth module>') and __import__('<truth module>')
  dynamic_import     importlib.import_module / __import__ with a name that is not a string constant (cannot be
                     checked)
  rig_reference      (decision / shared code only) a name or attribute segment that is the T1 rig: 'rig', '_rig',
                     'rig_*', '*_rig' (T1Rig, the RigClient's private rig handle), or the firmware process handle .fw
  rig_channel        (decision / shared code only) a call of the T1 rig's evaluation channels (LOCKSTEP 11.6: the MCU
                     RAM channels and lines, the serial log, the trace): .mcu_channel(), .mcu_channel_at(),
                     .mcu_lines(), .serial_log(), .serial_log_bytes(), .serial_log_sha256(), .trace_arrays(),
                     .trace_sha256(), .host_writes()
  scene_truth        (decision / shared code only) the true start of a SIM scene or rig: the attributes .start_pose,
                     .start_steer_rad
Decision and shared code may contain none of them. ALL-CAPS constant names (e.g. PLANT_PROFILE_NAMES, a list of
profile names) are not plant objects and do not count as plant references. The three rig rules apply to the strict
roles only (the plant-side glue hands the executor the rig's clock and RigClient by design).

Sink rule (plant-side and evaluator code, the place where truth and the decision side meet; e.g. the harness loop):
inside every function, values derived from a truth source are tainted (flow-insensitive over the function's
assignments, containers filled with tainted values become tainted; a nested function starts from the tainted names
of the function around it, i.e. closures carry the taint). A 'truth_to_decision' error is a tainted value that
reaches a decision object: a call on a receiver whose name contains estimator, controller, executor or bridge_client
(estimator.on_scan(x)); a call of a NAME containing one of these words (a constructor or factory such as
SensorEstimator(x), controller_factory(x); other factories - world_factory, plant_factory - are plant side); a store
into an attribute or item of such an object (estimator.pose = x, controller.hints['end'] = x).
Status calls clean their result (STATUS_SANITIZERS: .imu_sample(), .steer_status(), .status(), .snapshot()). The
sensor-frame calls .sample() and .scan() do NOT clean their result: a ScanFrame / scan() result also carries the
true sensor origin and the world-frame directions; only its LaserScan fields are sensor data (SENSOR_ATTRIBUTES:
.ranges, .angle_min, .angle_increment, ...; in a tuple unpack of such a call the names in SENSOR_ATTRIBUTES, e.g.
`ranges, origin, directions = caster.scan(...)` -> ranges). So the harness giving the estimator frame.ranges made from
the true pose is sensor synthesis (the design), while the whole frame, frame.origin or origin are truth.

Scope (repository-relative, SCOPE_PATTERNS): sim/ubts/steering/**, sim/ubts/closed_loop/**, the estimator's
orchard_perception/scan_corridor.py, sim/ubts/field_response.py, sim/ubts/cpu_lidar.py, the steering and closed-loop
tests, tools/row_executor/** (P4), the identification analysis library tools/field_calib/** (P4 ident: it reads the
records only, role shared; its tests role test) and the steering host code dashboard/scripts/steer_*.py,
esp_motor_v3.py (P3). The rest of the repository (the differential SIM stack) is outside this check; the ROS graph check
of A13(G) is P6a.

Exemptions (SINK_EXEMPTIONS): a sink finding of a plant-side module is exempted only by its exact key (path, function,
detail), never by (path, function) alone, and one key exempts one finding (a second finding with the same key in the
same function is an error). The report lists every exempted finding with its reason (check_tree 'exempted') and the keys
that matched nothing (stale keys, 'unused_exemptions').

Limits: a static over-approximation of the data flow inside one function; values passed through attributes of
other objects, globals or other functions are not followed (the role table covers whole modules for that). Decision
objects are recognised by name. steer_status() is a status sanitizer although its target_rad after a hold_measured
stop is the plant's servo view (made from truth; P1 / P3 move it to the MCU measurement). A pass is not a proof that
no truth can reach a decision module at run time.

CLI (from sim/ubts): python -B -m steering.truth_isolation [--root <repository>] [--json]   (exit 0 = no error)
"""
from __future__ import annotations

import argparse
import ast
import fnmatch
import json
from pathlib import Path, PurePosixPath
import sys

from . import backend_api as ba

SCHEMA = 'ubts.steer_sim.truth_isolation.v1'
ROLE_DECISION = 'decision'           # controller, estimator, executor, host bridge: no truth at all
ROLE_SHARED = 'shared'               # library code both sides use: no truth at all
ROLE_PLANT = 'plant'                 # plant, actuator backends, worlds, sensor synthesis, runners, harness loop
ROLE_EVALUATOR = 'evaluator'         # metrics and acceptance checks from truth
ROLE_DRIVER = 'truth_fed_driver'     # SIM test drivers that read truth by design (maneuvers)
ROLE_CONTRACT = 'contract'           # defines the truth contract (the truth names are data there)
ROLE_TOOL = 'tool'                   # this checker (names the truth sources as data)
ROLE_TEST = 'test'
STRICT_ROLES = (ROLE_DECISION, ROLE_SHARED)
SINK_ROLES = (ROLE_PLANT, ROLE_EVALUATOR, ROLE_DRIVER)
ALLOWED_ROLES = (ROLE_PLANT, ROLE_EVALUATOR, ROLE_DRIVER, ROLE_CONTRACT, ROLE_TOOL, ROLE_TEST)
ROLES = STRICT_ROLES + ALLOWED_ROLES

TRUTH_FIELDS = tuple(ba.TRUTH_OUTPUT_FIELDS)
TRUTH_KEY_PREFIX = 'truth_'
TRUTH_CONTRACT_NAMES = ('TRUTH_OUTPUT_FIELDS', 'ModelFlag', 'MODEL_FLAGS', 'MODEL_FLAG_NAMES')
PLANT_TRUTH_METHODS = ('truth', 'rear_axle_pose', 'run_info', 'servo_view_rad', 'lidar_pose')
WORLD_TRUTH_METHODS = ('corridor_truth', 'pose_in_field', 'to_field', 'clearance', 'clearances', 'class_at')
TRUTH_METHODS = PLANT_TRUTH_METHODS + WORLD_TRUTH_METHODS
TOPICS_EXACT = ('/odom', '/tf', '/joint_states', '/ground_truth/odom')
TOPIC_PREFIXES = ('/sim/truth', '/gps')
TRUTH_SUBSTRINGS = ('ground_truth', 'lane_centers_y')
SINK_RECEIVER_WORDS = ('estimator', 'controller', 'executor', 'bridge_client')
STATUS_SANITIZERS = ('imu_sample', 'steer_status', 'status', 'snapshot')
SENSOR_FRAME_CALLS = ('sample', 'scan')       # results carry the true sensor pose: only SENSOR_ATTRIBUTES are clean
SANITIZERS = STATUS_SANITIZERS                # (name kept for callers of the earlier version)
SENSOR_ATTRIBUTES = ('ranges', 'intensities', 'angle_min', 'angle_max', 'angle_increment', 'time_increment',
                     'scan_time', 'range_min', 'range_max')
DYNAMIC_IMPORT_CALLS = ('import_module', '__import__')
# T1 rig evaluation channels (LOCKSTEP 11.6: as truth columns, never an executor or estimator input); strict roles only
RIG_CHANNEL_METHODS = ('mcu_channel', 'mcu_channel_at', 'mcu_lines', 'serial_log', 'serial_log_bytes',
                       'serial_log_sha256', 'trace_arrays', 'trace_sha256', 'host_writes')
RIG_HANDLE_ATTRIBUTES = ('fw',)                         # the firmware process of the rig (FirmwareSteeringBackend)
SCENE_TRUTH_ATTRIBUTES = ('start_pose', 'start_steer_rad')
# Modules a decision / shared module must not import (dotted, relative to sim/ubts).
TRUTH_MODULES = (
    'steering.steered_plant', 'steering.fast_sim', 'steering.flat_world', 'steering.l0_servo',
    'steering.actuator_link', 'steering.native', 'steering.evaluate_steer', 'steering.entry_feasibility',
    'steering.maneuvers', 'steering.scene_spec', 'steering.gz_model', 'steering.gz_assemble', 'steering.gz_probe',
    'steering.gz_direct_drive', 'steering.sim_in_loop_steer',
    'steering.procedural_orchard', 'steering.scan_unobserved_clip', 'steering.gz_esp_emulator',
    'steering.t1_rig', 'steering.safety_scripts', 'steering.supervisor_steer', 'steering.sensor_contract_steer',
    'steering.verify_steer', 'steering.p6a_checks', 'steering.exec_checks', 'steering.p6a_imu_model', 'steering.ident.rig',
    'steering.ident.yard', 'steering.ident.instruments', 'steering.ident.recorder', 'steering.ident.procedures',
    'steering.ident.bench',
    'closed_loop.harness', 'closed_loop.world', 'closed_loop.raster_corridor', 'closed_loop.raster_catalogue',
    'closed_loop.evaluate', 'closed_loop.single_row_eval', 'closed_loop.matrix', 'cpu_lidar',
    # the executor's SIM harnesses (both spellings: tools/ on sys.path, or relative inside the package)
    'row_executor.sim_in_loop', 'row_executor.sim_in_loop_steer', 'row_executor.single_row_sim',
    'row_executor.single_row_video', 'tools.row_executor.sim_in_loop', 'tools.row_executor.sim_in_loop_steer',
    'tools.row_executor.single_row_sim', 'tools.row_executor.single_row_video',
    'gz')                    # Gazebo bindings (gz.sim8, gz.transport13, ...): direct access to the SIM world
# Mixed modules: importing the whole module or one of these names is a truth import; the other names are not.
PARTIAL_TRUTH_MODULES = {'field_response': ('FieldPlant',)}

SCOPE_PATTERNS = (
    'sim/ubts/steering/*.py', 'sim/ubts/steering/**/*.py', 'sim/ubts/closed_loop/*.py', 'sim/ubts/closed_loop/**/*.py',
    'sim/ubts/orchard_perception/scan_corridor.py', 'sim/ubts/field_response.py', 'sim/ubts/cpu_lidar.py',
    'sim/ubts/test_steering_*.py', 'sim/ubts/test_closed_loop_*.py', 'tools/row_executor/*.py',
    'tools/row_executor/**/*.py', 'tools/field_calib/*.py', 'tools/field_calib/**/*.py', 'dashboard/scripts/steer_*.py',
    'dashboard/scripts/esp_motor_v3.py')

# Explicit roles (repository-relative paths). Every allowlisted (truth-reading) module has a reason.
ROLE_TABLE = {
    # --- decision side: controller, estimator (no truth) -------------------------------------------------------------
    'sim/ubts/closed_loop/estimator.py': (ROLE_DECISION, 'SensorEstimator: 2D LiDAR + raw IMU; opt-in (design v2): '
                                                         'the MCU status line (applied duty, measured steering '
                                                         'angle) and corridor observations of a provider'),
    'sim/ubts/closed_loop/actuator_prior.py': (ROLE_DECISION, 'actuator priors of the estimator (layer L1): the '
                                                              'status line values only'),
    'sim/ubts/closed_loop/controllers.py': (ROLE_DECISION, 'RowController: estimator snapshot only'),
    'sim/ubts/closed_loop/single_row.py': (ROLE_DECISION, 'SingleRowController: estimator snapshot only'),
    'sim/ubts/orchard_perception/scan_corridor.py': (ROLE_DECISION, 'scan corridor detector used by the estimator'),
    'sim/ubts/closed_loop/steer_control.py': (ROLE_DECISION, 'P8 row-following steering controller: the estimator '
                                                             'snapshot only (design 6.2-6.4, v2 V2-4)'),
    # --- shared library code (no truth) -----------------------------------------------------------------------------
    'sim/ubts/steering/__init__.py': (ROLE_SHARED, 'package marker'),
    'sim/ubts/steering/vehicle_profile.py': (ROLE_SHARED, 'vehicle profile (values, ranges, provenance)'),
    'sim/ubts/steering/geometry.py': (ROLE_SHARED, 'steering geometry (also for executor-side code)'),
    'sim/ubts/steering/drive.py': (ROLE_SHARED, 'drive curve and stop mode values'),
    'sim/ubts/steering/host_profile_export.py': (ROLE_SHARED, 'host profile export (P3)'),
    'sim/ubts/closed_loop/__init__.py': (ROLE_SHARED, 'package marker'),
    'sim/ubts/closed_loop/external_paths.py': (ROLE_SHARED, 'external data locations'),
    # --- contract ------------------------------------------------------------------------------------------------
    'sim/ubts/steering/backend_api.py': (ROLE_CONTRACT, 'defines TRUTH_OUTPUT_FIELDS and the model flags'),
    'sim/ubts/steering/truth_isolation.py': (ROLE_TOOL, 'this checker'),
    # --- plant side ---------------------------------------------------------------------------------------------------
    'sim/ubts/steering/steered_plant.py': (ROLE_PLANT, 'the plant (owns the truth)'),
    'sim/ubts/steering/l0_servo.py': (ROLE_PLANT, 'steering actuator backend (plant side)'),
    'sim/ubts/steering/actuator_link.py': (ROLE_PLANT, 'native actuator backend link (plant side)'),
    'sim/ubts/steering/fast_sim.py': (ROLE_PLANT, 'runner: plant loop, trace (truth columns), truth-fed probe'),
    'sim/ubts/steering/flat_world.py': (ROLE_PLANT, 'W0 world and its LiDAR synthesis'),
    'sim/ubts/closed_loop/harness.py': (ROLE_PLANT, 'harness loop: sensor synthesis and metrics from truth'),
    'sim/ubts/closed_loop/world.py': (ROLE_PLANT, 'procedural world and LiDAR synthesis'),
    'sim/ubts/closed_loop/raster_corridor.py': (ROLE_PLANT, 'raster world, LiDAR synthesis and the PREREG evaluator'),
    'sim/ubts/closed_loop/raster_catalogue.py': (ROLE_PLANT, 'corridor catalogue builder (world labels)'),
    'sim/ubts/field_response.py': (ROLE_PLANT, 'differential plant (FieldPlant) and drive curves'),
    'sim/ubts/cpu_lidar.py': (ROLE_PLANT, 'LiDAR synthesis; subscribes /ground_truth/odom in the G-SIM (design 2.6)'),
    # --- evaluators -------------------------------------------------------------------------------------------------
    'sim/ubts/steering/evaluate_steer.py': (ROLE_EVALUATOR, 'steering episode evaluator (design 7.1)'),
    'sim/ubts/steering/entry_feasibility.py': (ROLE_EVALUATOR, 'A24 ideal-control feasibility'),
    'sim/ubts/closed_loop/evaluate.py': (ROLE_EVALUATOR, 'matrix evaluation from harness results'),
    'sim/ubts/closed_loop/single_row_eval.py': (ROLE_EVALUATOR, 'single-row comparison from harness results'),
    'sim/ubts/closed_loop/matrix.py': (ROLE_EVALUATOR, 'scenario matrix and truth-based judge'),
    # --- truth-fed SIM test drivers -----------------------------------------------------------------------------------
    'sim/ubts/steering/maneuvers.py': (ROLE_DRIVER, 'open-loop maneuvers time phases from the true state'),
    # --- SIM inputs -----------------------------------------------------------------------------------------------
    'sim/ubts/steering/scene_spec.py': (ROLE_PLANT, 'scene specification: SIM inputs (world, start, maneuver); '
                                                    'never executor code'),
    # --- 3rd phase (P4 executor in the loop; WF-3a) -------------------------------------------------------------------
    'sim/ubts/steering/t1_rig.py': (ROLE_PLANT, 'T1 rig: host bridge + MCU + plant + sensors; an executor gets only the '
                                                'RigClient replies, poll_sensors() and clock()'),
    'sim/ubts/steering/scripted_control.py': (ROLE_DECISION, 'SIM scripted controllers of the executor slot: the '
                                                             'estimator snapshot and the steering status only'),
    'sim/ubts/steering/s2_format.py': (ROLE_SHARED, 'real S2 LaserScan format of a SIM scan (sensor data)'),
    'sim/ubts/steering/ident/__init__.py': (ROLE_SHARED, 'package marker'),
    'sim/ubts/steering/ident/analysis.py': (ROLE_SHARED, 'identification analysis from the records only (P4 ident)'),
    'tools/row_executor/single_row_sim.py': (ROLE_PLANT, 'single-row SIM episode runner of the executor: harness '
                                                         '(sensor synthesis, truth for scoring)'),
    'tools/row_executor/single_row_video.py': (ROLE_EVALUATOR, 'episode video from the SIM truth (display only)'),
    # --- 3rd phase G-SIM files (WF-3b; classified here, the owner may refine) ---------------------------------------
    'sim/ubts/steering/supervisor_steer.py': (ROLE_PLANT, 'G-SIM process supervisor (plant side)'),
    'sim/ubts/steering/sensor_contract_steer.py': (ROLE_PLANT, 'G-SIM sensor contract node: IMU model, virtual GPS and '
                                                               'status files from the SIM truth (plant side)'),
    'sim/ubts/steering/verify_steer.py': (ROLE_EVALUATOR, 'G-SIM integration check (reads the truth for evaluation)'),
    'sim/ubts/steering/p6a_checks.py': (ROLE_EVALUATOR, 'P6a checks (evaluation)'),
    'sim/ubts/steering/exec_checks.py': (ROLE_EVALUATOR, 'executor-in-the-stack checks of the G-SIM stack (A5(G), A13(G) R6, '
                                         'A17(G) executor items, W2 chain; evaluation)'),
    'sim/ubts/steering/p6a_client.py': (ROLE_DRIVER, 'P6a scripted clients of the control socket (SIM test driver)'),
    'sim/ubts/steering/p6a_graph.py': (ROLE_TOOL, 'ROS graph rules of A13 (G): names the truth topics as data'),
    'sim/ubts/steering/p6a_imu_model.py': (ROLE_PLANT, 'G-SIM IMU error model (plant side)'),
    'sim/ubts/steering/p6a_fw_rules.py': (ROLE_EVALUATOR, 'firmware monitor rule ticks from a recorded emulator session '
                                                          '(A19(G) judgement times; evaluation)'),
}
# Sink findings that are by design, per exact (path, function, detail) key: never a sensor or plant value given to the
# executor's decision code, each with its reason (reported as exempted, not as errors). One key exempts one finding.
_PLANT_NAME = 'the plant-bracket NAME (plant["name"]) selects the SIM calibration values of that plant profile (nominal ' \
              'profile, not the run draw); the flow-insensitive rule taints the name because it contains "plant"'
_RESULT_RECORD = 'result["executor"] is the episode result record (a dict), not the executor: nothing goes into the ' \
                 'executor'
SINK_EXEMPTIONS = {
    ('tools/row_executor/sim_in_loop.py', 'run_job', 'executor_config(...) in run_job'): _PLANT_NAME,
    ('tools/row_executor/sim_in_loop.py', 'run_job', 'ExecutorInTheLoop(...) in run_job'):
        'differential history harness (exec_field / exec_cone variants): the configuration of the plant-bracket name '
        '(see executor_config) and the operator-measured row length of exec_field: true_row_length(ep), the tape '
        'length from the catalogue row end, is an operator input by design (exec_cone gives the length + margin); '
        'not used by the front-steer harness sim_in_loop_steer (row_len_m None)',
    ('tools/row_executor/sim_in_loop.py', 'run_job', 'result.executor = ... in run_job'): _RESULT_RECORD,
    ('tools/row_executor/sim_in_loop.py', 'run_job', 'result.executor.row_len_m = ... in run_job'): _RESULT_RECORD,
    ('tools/row_executor/sim_in_loop.py', 'run_job', 'result.executor.speed_ref = ... in run_job'): _RESULT_RECORD,
    ('tools/row_executor/sim_in_loop.py', 'run_job', 'result.executor.tb_trace = ... in run_job'): _RESULT_RECORD,
    ('tools/row_executor/sim_in_loop.py', 'load_matrix', 'executor_config(...) in load_matrix'): _PLANT_NAME,
    ('tools/row_executor/sim_in_loop.py', 'main', 'executor_config(...) in main'): _PLANT_NAME,
    ('tools/row_executor/single_row_sim.py', 'main', 'single_row_executor_config(...) in main'):
        'the loop variable is a plant NAME: it selects the nominal SIM calibration values of that plant profile',
    ('tools/row_executor/single_row_sim.py', '_facade_state', 'self._controllers.append(...) in _facade_state'):
        'the evaluation-side rest test stores into self.rest_since, so the flow-insensitive rule taints every self.* '
        'name; the append keeps the runner controller objects in a list for counting (no value goes into a controller)',
    ('sim/ubts/steering/gz_probe.py', 'factory', 'ScriptSteerController(...) in factory'):
        'the closure input is the world KIND string (W0 / W2) through the fixed A12 open-loop schedules '
        '(field_world.a12_twin.scripts); no plant or world value reaches the scripted controller (WF-3b file)',
}
# Dynamic imports of decision code that are checked another way, per (path, function), with the reason.
DYNAMIC_IMPORT_EXEMPTIONS = {
    ('tools/row_executor/core.py', 'single_row_module'):
        "importlib.import_module(<controllers package> + '.single_row'): closed_loop.single_row (a decision module) "
        'next to the verified controllers module; its file is SHA-256 checked against the vendor manifest',
}
# A13: the executor node subscribes to these topics only (design 2.6).
EXECUTOR_TOPIC_ALLOWLIST = ('/scan', '/imu/data_raw', '/rover/control/requested')
EXECUTOR_SUBSCRIPTION_FILES = ('tools/row_executor/executor_main.py',)
# Pattern roles (fnmatch on the repository-relative path), checked after ROLE_TABLE, first match wins.
ROLE_PATTERNS = (
    ('sim/ubts/test_*.py', ROLE_TEST, 'unit test'),
    ('sim/ubts/*/test_*.py', ROLE_TEST, 'unit test'),
    ('sim/ubts/closed_loop/testdata/*', ROLE_TEST, 'test reference data and reference implementations'),
    ('sim/ubts/steering/conformance/*', ROLE_TEST, 'MCU conformance scripts (P3)'),
    ('sim/ubts/steering/safety_scripts/*', ROLE_EVALUATOR, 'P4 safety scripts and acceptance checks (evaluation)'),
    ('sim/ubts/steering/ident/*', ROLE_PLANT, 'identification rig, yard, instruments, bench tool and recorder (plant '
                                              'side, P4 ident)'),
    ('sim/ubts/steering/native/*', ROLE_PLANT, 'native actuator / MCU build (plant side)'),
    ('tools/field_calib/tests/*', ROLE_TEST, 'identification analysis tests'),
    ('tools/field_calib/*', ROLE_SHARED, 'identification analysis library: reads the field / SIM records only (A18)'),
    ('sim/ubts/steering/gz_*.py', ROLE_PLANT, 'G-SIM model, assembly and probes (plant side)'),
    ('sim/ubts/steering/sim_in_loop_steer.py', ROLE_PLANT, 'T1 glue: plant -> sensor packets (P4)'),
    ('sim/ubts/steering/scan_unobserved_clip.py', ROLE_PLANT, 'G-SIM sensor post-processing from the raster (P6a)'),
    ('sim/ubts/steering/procedural_orchard.py', ROLE_PLANT, 'procedural world (P7)'),
    ('tools/row_executor/sim_in_loop*.py', ROLE_PLANT, 'executor T1 glue: plant -> sensor packets (P4)'),
    ('tools/row_executor/test_*.py', ROLE_TEST, 'executor tests'),
    ('tools/row_executor/tests/*', ROLE_TEST, 'executor tests'),
    ('tools/row_executor/*', ROLE_DECISION, 'executor (P4)'),
    ('dashboard/scripts/steer_*.py', ROLE_DECISION, 'steering host bridge / control rules (P3)'),
    ('dashboard/scripts/esp_motor_v3.py', ROLE_DECISION, 'ESP V3 host parser (P3)'),
)
REQUIRED_DECISION_MODULES = ('sim/ubts/closed_loop/estimator.py', 'sim/ubts/closed_loop/controllers.py',
                             'sim/ubts/closed_loop/single_row.py', 'sim/ubts/orchard_perception/scan_corridor.py')


def repository_root():
    return Path(__file__).resolve().parents[3]


def _matches(rel, pattern):
    return fnmatch.fnmatchcase(rel, pattern)


def classify(rel, role_table=None, role_patterns=None):
    """(role or None, reason) of a repository-relative path."""
    table = ROLE_TABLE if role_table is None else role_table
    patterns = ROLE_PATTERNS if role_patterns is None else role_patterns
    if rel in table:
        return table[rel]
    for pattern, role, reason in patterns:
        if _matches(rel, pattern):
            return role, reason
    return None, 'unclassified'


def scope_files(root, patterns=SCOPE_PATTERNS):
    """Repository-relative POSIX paths of the Python files in scope (sorted, no __pycache__)."""
    root = Path(root)
    out = set()
    for path in root.rglob('*.py'):
        if '__pycache__' in path.parts or any(part.startswith('.') for part in path.relative_to(root).parts):
            continue
        rel = path.relative_to(root).as_posix()
        if any(_matches(rel, pattern) for pattern in patterns):
            out.add(rel)
    return sorted(out)


def module_name(rel):
    """Dotted module name relative to sim/ubts (or the repository root) of a relative path."""
    parts = PurePosixPath(rel).with_suffix('').parts
    if parts[:2] == ('sim', 'ubts'):
        parts = parts[2:]
    if parts and parts[-1] == '__init__':
        parts = parts[:-1]
    return '.'.join(parts)


# --- AST helpers ---------------------------------------------------------------------------------------------------
def _docstring_nodes(tree):
    """ids of the docstring Constant nodes and of other bare string statements (not code)."""
    skip = set()
    for node in ast.walk(tree):
        body = getattr(node, 'body', None)
        if isinstance(body, list):
            for statement in body:
                if (isinstance(statement, ast.Expr) and isinstance(statement.value, ast.Constant)
                        and isinstance(statement.value.value, str)):
                    skip.add(id(statement.value))
    return skip


def _segments(node):
    """Identifier segments of a dotted receiver chain (Name / Attribute / Subscript with a constant / Call.func)."""
    out = []
    while True:
        if isinstance(node, ast.Attribute):
            out.append(node.attr)
            node = node.value
        elif isinstance(node, ast.Subscript):
            key = node.slice.value if isinstance(node.slice, ast.Constant) else None
            if isinstance(key, str):
                out.append(key)
            node = node.value
        elif isinstance(node, ast.Call):
            node = node.func
        elif isinstance(node, ast.Name):
            out.append(node.id)
            return out[::-1]
        else:
            return out[::-1]


def string_findings(text):
    """Truth rules a string constant breaks (list of (rule, detail))."""
    out = []
    if text in TRUTH_FIELDS:
        out.append(('truth_field', text))
    if text.startswith(TRUTH_KEY_PREFIX):
        out.append(('truth_key', text))
    if text in TOPICS_EXACT or any(text == p or text.startswith(p + '/') for p in TOPIC_PREFIXES):
        out.append(('truth_topic', text))
    elif any(word in text for word in TRUTH_SUBSTRINGS):
        out.append(('truth_topic' if text.startswith('/') else 'truth_key', text))
    return out


def _is_constant_name(name):
    return name.upper() == name and any(c.isalpha() for c in name)


def _is_plant_word(name):
    """A name that may hold the plant: contains 'plant' and is not an ALL-CAPS constant name."""
    return 'plant' in name.lower() and not _is_constant_name(name)


def _is_rig_word(name):
    """A name that may hold the T1 rig: the segment 'rig' (also '_rig', 'rig_*', '*_rig', '*_rig_*'); not an ALL-CAPS
    constant name. 'origin', 'right', 'trigger' are not rig words (no '_' boundary)."""
    if _is_constant_name(name):
        return False
    low = name.lower().strip('_')
    return low == 'rig' or low.startswith('rig_') or low.endswith('_rig') or '_rig_' in low


def _resolve_import(module, level, current):
    """Absolute dotted module of an import (level = relative dots) inside the module `current`."""
    if not level:
        return module or ''
    package = current.split('.')
    package = package[:-1] if package else []          # module -> its package
    package = package[:len(package) - (level - 1)] if level > 1 else package
    return '.'.join([p for p in package if p] + ([module] if module else []))


def _truth_module(name, imported=None):
    """True for a truth module; for a PARTIAL_TRUTH_MODULES module only the whole module or its plant names."""
    if any(name == m or name.startswith(m + '.') for m in TRUTH_MODULES):
        return True
    for module, names in PARTIAL_TRUTH_MODULES.items():
        if name == module:
            return imported is None or imported in names
    return False


class _StrictVisitor(ast.NodeVisitor):
    """Truth reads of a decision / shared module."""

    def __init__(self, module, skip):
        self.module = module
        self.skip = skip
        self.findings = []
        self.functions = []

    def add(self, node, rule, detail):
        self.findings.append(dict(line=getattr(node, 'lineno', 0), rule=rule, detail=str(detail),
                                  function=self.functions[-1] if self.functions else None))

    def visit_FunctionDef(self, node):
        self.functions.append(node.name)
        self.generic_visit(node)
        self.functions.pop()

    visit_AsyncFunctionDef = visit_FunctionDef

    def visit_Constant(self, node):
        if isinstance(node.value, str) and id(node) not in self.skip:
            for rule, detail in string_findings(node.value):
                self.add(node, rule, detail)

    def visit_Name(self, node):
        if node.id in TRUTH_CONTRACT_NAMES:
            self.add(node, 'truth_contract', node.id)
        if _is_plant_word(node.id):
            self.add(node, 'plant_reference', node.id)
        if node.id == 'world':
            self.add(node, 'world_reference', node.id)
        if _is_rig_word(node.id):
            self.add(node, 'rig_reference', node.id)

    def visit_Attribute(self, node):
        if node.attr in TRUTH_FIELDS:
            self.add(node, 'truth_field', '.'.join(_segments(node)))
        if node.attr in TRUTH_CONTRACT_NAMES:
            self.add(node, 'truth_contract', node.attr)
        if _is_plant_word(node.attr):
            self.add(node, 'plant_reference', '.'.join(_segments(node)))
        if node.attr == 'world':
            self.add(node, 'world_reference', '.'.join(_segments(node)))
        if _is_rig_word(node.attr) or node.attr in RIG_HANDLE_ATTRIBUTES:
            self.add(node, 'rig_reference', '.'.join(_segments(node)))
        if node.attr in SCENE_TRUTH_ATTRIBUTES:
            self.add(node, 'scene_truth', '.'.join(_segments(node)))
        self.generic_visit(node)

    def visit_Subscript(self, node):
        if isinstance(node.slice, ast.Constant) and node.slice.value == 'plant':
            self.add(node, 'plant_reference', "['plant']")
        self.generic_visit(node)

    def visit_Call(self, node):
        if isinstance(node.func, ast.Attribute) and node.func.attr in TRUTH_METHODS:
            self.add(node, 'truth_method', '.'.join(_segments(node.func)) + '()')
        if isinstance(node.func, ast.Attribute) and node.func.attr in RIG_CHANNEL_METHODS:
            self.add(node, 'rig_channel', '.'.join(_segments(node.func)) + '()')
        callee = node.func.id if isinstance(node.func, ast.Name) else (
            node.func.attr if isinstance(node.func, ast.Attribute) else None)
        if callee in DYNAMIC_IMPORT_CALLS:
            # importlib.import_module(name) / __import__(name): a string constant is checked like an import; any other
            # name cannot be checked statically (decision and shared code has no reason to import dynamically)
            arg = node.args[0] if node.args else next((k.value for k in node.keywords if k.arg == 'name'), None)
            text = arg.value if isinstance(arg, ast.Constant) and isinstance(arg.value, str) else None
            if text is None or text.startswith('.'):
                self.add(node, 'dynamic_import', '%s(<not a checkable constant>)' % '.'.join(_segments(node.func)))
            elif _truth_module(text):
                self.add(node, 'truth_import', '%s(%r)' % ('.'.join(_segments(node.func)), text))
        self.generic_visit(node)

    def visit_Import(self, node):
        for alias in node.names:
            if _truth_module(alias.name):
                self.add(node, 'truth_import', alias.name)

    def visit_ImportFrom(self, node):
        base = _resolve_import(node.module, node.level, self.module)
        if any(base == m or base.startswith(m + '.') for m in TRUTH_MODULES):
            self.add(node, 'truth_import', base)
        for alias in node.names:
            full = base + '.' + alias.name if base else alias.name
            if base in PARTIAL_TRUTH_MODULES:
                if _truth_module(base, alias.name):
                    self.add(node, 'truth_import', full)
            elif _truth_module(full) and not _truth_module(base):
                self.add(node, 'truth_import', full)
            if alias.name in TRUTH_CONTRACT_NAMES:
                self.add(node, 'truth_contract', alias.name)


# --- sink rule (plant side) --------------------------------------------------------------------------------------------
def _expression_truth(node, tainted):
    """True if the expression reads truth or a tainted name (sanitizer calls clean their own result)."""
    if isinstance(node, ast.Call):
        func = node.func
        if isinstance(func, ast.Attribute) and func.attr in SANITIZERS:
            return False
        if isinstance(func, ast.Attribute) and func.attr in TRUTH_METHODS:
            return True
        parts = [func] + list(node.args) + [k.value for k in node.keywords]
        return any(_expression_truth(part, tainted) for part in parts)
    if isinstance(node, ast.Name):
        return node.id in tainted or _is_plant_word(node.id) or node.id == 'world'
    if isinstance(node, ast.Attribute):
        if node.attr in TRUTH_FIELDS or _is_plant_word(node.attr) or node.attr == 'world':
            return True
        if node.attr in SENSOR_ATTRIBUTES:
            return False
        return _expression_truth(node.value, tainted)
    if isinstance(node, ast.Constant):
        return isinstance(node.value, str) and bool(string_findings(node.value)) and node.value != 'plant'
    if isinstance(node, ast.Subscript):
        if isinstance(node.slice, ast.Constant) and node.slice.value == 'plant':
            return True
        return _expression_truth(node.value, tainted) or _expression_truth(node.slice, tainted)
    if isinstance(node, (ast.Lambda, ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
        return False
    return any(_expression_truth(child, tainted) for child in ast.iter_child_nodes(node))


def _target_names(target):
    if isinstance(target, ast.Name):
        return [target.id]
    if isinstance(target, (ast.Tuple, ast.List)):
        return [n for element in target.elts for n in _target_names(element)]
    if isinstance(target, ast.Starred):
        return _target_names(target.value)
    if isinstance(target, (ast.Attribute, ast.Subscript)):
        base = _segments(target)
        return base[:1]
    return []


_NESTED = (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef, ast.Lambda)


def _own_nodes(function):
    """Nodes of a function body without the nested functions / classes / lambdas (each is analysed on its own)."""
    out, stack = [], [node for node in function.body if not isinstance(node, _NESTED)]
    while stack:
        node = stack.pop()
        out.append(node)
        stack.extend(child for child in ast.iter_child_nodes(node) if not isinstance(child, _NESTED))
    return out


def _has_sink_word(segments):
    return any(any(word in s.lower() for word in SINK_RECEIVER_WORDS) for s in segments)


def _is_sink(call):
    """A call that hands its arguments to a decision object: a method of a receiver named like one (estimator.on_scan),
    or a NAME containing a decision word (a constructor or factory: SensorEstimator(...), controller_factory(...))."""
    if isinstance(call.func, ast.Attribute):
        return _has_sink_word(_segments(call.func.value))
    if isinstance(call.func, ast.Name):
        return _has_sink_word([call.func.id])
    return False


def _is_sink_store(target):
    """An assignment target that stores into a decision object (estimator.pose = ..., controller.hints['end'] = ...)."""
    if isinstance(target, (ast.Tuple, ast.List)):
        return any(_is_sink_store(element) for element in target.elts)
    if isinstance(target, ast.Starred):
        return _is_sink_store(target.value)
    if isinstance(target, (ast.Attribute, ast.Subscript)):
        return _has_sink_word(_segments(target))
    return False


def _is_sensor_frame_call(node):
    return (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
            and node.func.attr in SENSOR_FRAME_CALLS)


def _assigned_names(target, value):
    """Names a tainted value reaches through this target. In a tuple unpack of a sensor-frame call (.sample(),
    .scan()) the LaserScan field names (SENSOR_ATTRIBUTES, e.g. ranges) are sensor data and stay clean."""
    names = _target_names(target)
    if _is_sensor_frame_call(value) and isinstance(target, (ast.Tuple, ast.List)):
        names = [name for name in names if name not in SENSOR_ATTRIBUTES]
    return names


def _nested_functions(node):
    """Functions defined directly inside `node` (not inside a deeper function)."""
    out, stack = [], list(ast.iter_child_nodes(node))
    while stack:
        child = stack.pop()
        if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)):
            out.append(child)
        else:
            stack.extend(ast.iter_child_nodes(child))
    return out


def sink_findings(tree):
    """truth_to_decision findings of a plant-side module (see the module docstring). A nested function (closure)
    starts from the tainted names of the function that encloses it."""
    findings = []
    pending = [(function, frozenset()) for function in _nested_functions(tree)]
    while pending:
        function, inherited = pending.pop(0)
        nodes = _own_nodes(function)
        tainted = set(inherited)
        changed = True
        while changed:
            changed = False
            for node in nodes:
                targets, value = [], None
                if isinstance(node, ast.Assign):
                    targets, value = node.targets, node.value
                elif isinstance(node, (ast.AugAssign, ast.AnnAssign)) and node.value is not None:
                    targets, value = [node.target], node.value
                elif isinstance(node, (ast.For, ast.AsyncFor)):
                    targets, value = [node.target], node.iter
                elif isinstance(node, ast.withitem) and node.optional_vars is not None:
                    targets, value = [node.optional_vars], node.context_expr
                elif isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.args:
                    # container.append(x), heapq.heappush(container, x), d.update(x): the container gets the taint
                    if node.func.attr in ('append', 'extend', 'update', 'add', 'insert', 'appendleft'):
                        targets, value = [node.func.value], ast.Tuple(elts=list(node.args), ctx=ast.Load())
                    elif node.func.attr in ('heappush', 'put', 'put_nowait') and len(node.args) >= 2:
                        targets, value = [node.args[0]], node.args[1]
                if value is None or not _expression_truth(value, tainted):
                    continue
                for target in targets:
                    for name in _assigned_names(target, value):
                        if name not in tainted:
                            tainted.add(name)
                            changed = True
        for node in nodes:
            if isinstance(node, ast.Call) and _is_sink(node):
                for arg in list(node.args) + [k.value for k in node.keywords]:
                    if _expression_truth(arg, tainted):
                        findings.append(dict(line=node.lineno, rule='truth_to_decision', function=function.name,
                                             detail='%s(...) in %s' % ('.'.join(_segments(node.func)), function.name)))
                        break
            elif isinstance(node, (ast.Assign, ast.AugAssign, ast.AnnAssign)) and node.value is not None:
                targets = node.targets if isinstance(node, ast.Assign) else [node.target]
                if any(_is_sink_store(target) for target in targets) and _expression_truth(node.value, tainted):
                    stored = [t for t in targets if _is_sink_store(t)]
                    findings.append(dict(line=node.lineno, rule='truth_to_decision', function=function.name,
                                         detail='%s = ... in %s' % ('.'.join(_segments(stored[0])) or 'target',
                                                                    function.name)))
        pending.extend((nested, frozenset(tainted)) for nested in _nested_functions(function))
    return sorted(findings, key=lambda item: (item['line'], item['detail']))


def check_source(source, rel, role):
    """Findings of one source: (errors, allowed_reads). errors break the rules of the role; allowed_reads are the
    truth reads of an allowlisted module (reported, not errors). role None = unclassified."""
    tree = ast.parse(source, filename=rel)
    strict = _StrictVisitor(module_name(rel), _docstring_nodes(tree))
    strict.visit(tree)
    reads = strict.findings
    if role in STRICT_ROLES:
        errors, exempted = [], []
        for f in reads:
            reason = DYNAMIC_IMPORT_EXEMPTIONS.get((rel, f.get('function'))) if f['rule'] == 'dynamic_import' else None
            (exempted if reason else errors).append(dict(f, exempted=reason) if reason else f)
        return errors, [dict(f, rule='exempted:' + f['rule']) for f in exempted]
    errors = sink_findings(tree) if role in SINK_ROLES or role is None else []
    if role is None:
        return ([dict(f, rule='unclassified_truth_read:' + f['rule']) for f in reads] + errors), []
    kept, exempted, used = [], [], set()
    for f in errors:
        key = (rel, f.get('function'), f.get('detail'))
        reason = SINK_EXEMPTIONS.get(key)            # exact three-part key only (no (path, function) fallback)
        if reason and key in used:
            # one key exempts one finding: a second identical finding in the same function is a new flow
            kept.append(dict(f, note='same detail as an exempted finding: the exemption covers one finding only'))
            continue
        if reason:
            used.add(key)
            exempted.append(dict(f, exempted=reason))
        else:
            kept.append(f)
    return kept, reads + [dict(f, rule='exempted:' + f['rule']) for f in exempted]


def _topic_default(function, name):
    """The default value of parameter `name` of a function definition, or None."""
    args = function.args
    positional = list(args.args)
    defaults = list(args.defaults)
    offset = len(positional) - len(defaults)
    for index, arg in enumerate(positional):
        if arg.arg == name and index >= offset:
            value = defaults[index - offset]
            return value.value if isinstance(value, ast.Constant) else None
    for arg, value in zip(args.kwonlyargs, args.kw_defaults):
        if arg.arg == name and isinstance(value, ast.Constant):
            return value.value
    return None


def subscription_check(root=None, files=EXECUTOR_SUBSCRIPTION_FILES, allowlist=EXECUTOR_TOPIC_ALLOWLIST):
    """A13: every topic the executor node subscribes to (create_subscription calls; a topic given by a parameter is
    resolved to that parameter's default) and every --*-topic command-line default lies in the allowlist. A topic that
    cannot be resolved is a problem. A run-time override (--scan-topic ...) is an operator setting at launch; the ROS
    graph check of A13 (G) sees the actual subscriptions."""
    root = repository_root() if root is None else Path(root)
    topics, problems = [], []
    for rel in files:
        tree = ast.parse((root / rel).read_bytes().decode('utf-8'), filename=rel)
        for function in [n for n in ast.walk(tree) if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))]:
            for node in ast.walk(function):
                if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
                    continue
                if node.func.attr == 'create_subscription' and len(node.args) >= 2:
                    arg = node.args[1]
                    topic = arg.value if isinstance(arg, ast.Constant) else (
                        _topic_default(function, arg.id) if isinstance(arg, ast.Name) else None)
                    where = '%s:%d %s' % (rel, node.lineno, function.name)
                    if not isinstance(topic, str):
                        problems.append('unresolved subscription topic at %s' % where)
                    else:
                        topics.append(dict(topic=topic, where=where))
                elif node.func.attr == 'add_argument' and node.args and isinstance(node.args[0], ast.Constant) \
                        and str(node.args[0].value).endswith('-topic'):
                    default = next((k.value for k in node.keywords if k.arg == 'default'), None)
                    value = default.value if isinstance(default, ast.Constant) else None
                    where = '%s:%d %s' % (rel, node.lineno, node.args[0].value)
                    if not isinstance(value, str):
                        problems.append('command-line topic without a constant default at %s' % where)
                    else:
                        topics.append(dict(topic=value, where=where))
    outside = sorted({t['topic'] for t in topics if t['topic'] not in allowlist})
    problems += ['topic %s outside the allowlist' % t for t in outside]
    return dict(check='A13(F) executor subscriptions', allowlist=list(allowlist),
                topics=sorted({t['topic'] for t in topics}), sites=topics, problems=problems,
                passed=bool(topics) and not problems)


def check_tree(root=None, files=None, role_table=None, role_patterns=None, require=None):
    """Run the check over the scope of a repository (or the given relative files). Returns the report dict.
    require: repository-relative decision modules that must be in the checked set (default: REQUIRED_DECISION_MODULES
    for the default scope, none for an explicit file list)."""
    root = repository_root() if root is None else Path(root)
    if require is None:
        require = REQUIRED_DECISION_MODULES if files is None else ()
    files = scope_files(root) if files is None else sorted(files)
    modules, errors, exempted = [], [], []
    for rel in files:
        role, reason = classify(rel, role_table, role_patterns)
        source = (root / rel).read_bytes().decode('utf-8')
        module_errors, reads = check_source(source, rel, role)
        rules = {}
        for item in reads:
            rules[item['rule']] = rules.get(item['rule'], 0) + 1
            if item['rule'].startswith('exempted:'):
                exempted.append(dict(path=rel, line=item['line'], function=item.get('function'),
                                     rule=item['rule'][len('exempted:'):], detail=item['detail'],
                                     reason=item.get('exempted')))
        modules.append(dict(path=rel, role=role or 'unclassified', reason=reason, errors=len(module_errors),
                            allowed_truth_reads=rules))
        errors.extend(dict(item, path=rel, role=role or 'unclassified') for item in module_errors)
    missing = [rel for rel in require if rel not in files]
    counts = {}
    for module in modules:
        counts[module['role']] = counts.get(module['role'], 0) + 1
    matched = {(e['path'], e['function'], e['detail']) for e in exempted if e['rule'] == 'truth_to_decision'}
    unused = sorted(list(key) for key in SINK_EXEMPTIONS if key[0] in files and tuple(key) not in matched)
    return dict(schema=SCHEMA, check='A13(F static)', root=str(root), files=len(files), roles=counts,
                errors=errors, missing_required_decision_modules=missing,
                passed=not errors and not missing, modules=modules,
                exempted=exempted, exempted_count=len(exempted), unused_exemptions=unused,
                note='static over-approximation inside each function; not a run-time proof (see the module docstring); '
                     'exempted: sink findings exempted by an exact SINK_EXEMPTIONS key (reported, not errors)')


def main(argv=None):
    parser = argparse.ArgumentParser(prog='python -B -m steering.truth_isolation', description=__doc__.split('\n')[0])
    parser.add_argument('--root', default=None, help='repository root (default: this work tree)')
    parser.add_argument('--json', action='store_true', help='print the whole report')
    args = parser.parse_args(argv)
    report = check_tree(args.root)
    subs = subscription_check(args.root)
    report['subscriptions'] = subs
    passed = report['passed'] and subs['passed']
    if args.json:
        print(json.dumps(report, indent=1, sort_keys=True))
    else:
        print('A13(F static): %d files, roles %s, errors %d, exempted %d, missing decision modules %s; executor '
              'subscriptions %s (allowlist %s) -> %s'
              % (report['files'], json.dumps(report['roles'], sort_keys=True), len(report['errors']),
                 report['exempted_count'], report['missing_required_decision_modules'], subs['topics'],
                 list(subs['allowlist']), 'PASS' if passed else 'FAIL'))
        for item in report['errors']:
            print('  %s:%d %s %s (%s)' % (item['path'], item['line'], item['rule'], item['detail'], item['role']))
        for item in report['exempted']:
            print('  exempted %s:%d %s' % (item['path'], item['line'], item['detail']))
        for key in report['unused_exemptions']:
            print('  unused exemption key %s' % (key,))
        for problem in subs['problems']:
            print('  subscriptions: %s' % problem)
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
