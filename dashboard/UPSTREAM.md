# Dashboard copy: where it comes from

Since 2026-11-13 (D-13, docs/steering/SIM_DASHBOARD_KO.md) the folders `config/`, `dashboard_server/` and `static/` of
this directory are the dashboard repository's files at one commit, unchanged:
https://github.com/2026-BTS-UNI-Car/unist_orachard_dashboard, commit and per-file sha256 / git blob id in
`DASHBOARD_SOURCE.json`. They are written and checked by `tools/steer_sim/sync_dashboard.py` (do not edit them here;
change the dashboard repository and copy again). The simulation's own dashboard code (status provider, extension routes,
extra page script) is in `../dashboard_sim/` and is plugged into the dashboard's extension points.

`scripts/` is not part of that copy. It holds the programs the simulation stack runs in its own containers: the sensor
bridges and the data recorder in their simulation form (simulation clock, camera health), the steering host bridge
(whose source of origin is this repository; the dashboard repository has a port of it) and their tests. Their status
is listed in docs/steering/SIM_DASHBOARD_KO.md section 3.

The notes below describe the earlier copy (a subtree of the deployed dashboard with a simulation overlay, 2026-09/10).
Its simulation-only server modules and pages (SIM occupancy map, native mapping proxies, old control page) were removed
from this directory on 2026-11-13; they are in the history of main up to 09cc932.

## Earlier copy (2026-09 to 2026-11-12)

# Deployed dashboard source and local overlay

2026-10-01: deployed common source `9e5ffd0` is merged with the SIM overlay.
NAV-SAT/SIG reception is separate from fix-used satellites; recorder status
uses the live recorder API and fails to unknown on stale/unavailable replies.
SIM keeps the strict `http://sim:8790` recorder boundary. ROS Jazzy67 tests
passed without skips; independent Node7 suites and WSL6 tests passed.
The detail-row/local freshness timer follow-up `aef3701` is also copied.
Historical-map display and localization-epoch freeze are SIM-only additions.
This checkpoint is source integration, not a replacement of the running18080
dashboard. See `docs/sim_map_history.md` for the deliberate upgrade procedure.
Actual Jetson GPS driver/bridge/web deployment evidence is in
`docs/jetson_gps_reception_recording_deployment.json` (ARM39/preservation11).

GPS reconnect source sync: `3c1c102bfaef623afdad288434266e8416c27e13`
(2026-09-30). The shared hardware driver and Linux PTY regressions handle
disconnect cleanup/retry; local Iron and Jetson ARM each passed 21 tests.
The actual GPS service was directly deployed as image `344dcea67d94` and
passed 10 preservation/reception checks. See `docs/jetson_gps_reconnect_deployment.json`.
This source sync does not change the running simulator.

Upstream: https://github.com/2026-BTS-UNI-Car/unist_orachard_dashboard
Imported deployed commit: `35c67365249651995e71609ba4fa799abbb98fd4` (main, 2026-09-28).
The live Jetson was updated and verified before this subtree merge.
Prior import: c7df02f. The Git subtree history records both snapshots.

Common-source sync: `fbebba0..42f7e3d` (2026-09-30) applied by three-way patch.
This adds sensor-native IMU display/recording, the unchanged `/imu/data` raw
relay, explicit HTTP410 for retired zero APIs, and current USB GPS documentation.
The SIM map/autonomy/archive/clock overlays remain. The hardware Compose delta
is confined to `docker-compose.jetson.reference.yml`; the active SIM wrapper
still includes only `../compose.ubts.yaml`. The removed IMU yaw-offset CLI
argument is also removed from the SIM supervisor. This is a source sync;
it does not claim that the running SIM or Jetson has been redeployed by it.

Common source: raw camera/LaserScan/IMU/GPS pages, no AI worker/tree/fusion pipeline,
manual ESP lease/TTL/STOP, original archive playback. Hardware UI remains the default.

Local overlay: SimulationProvider, server-selected simulation page flag/title,
virtual GPS and the recorded world's orchard layout, live occupancy mapping,
map archive/regeneration, guarded simulator autonomy, compose wrapper, domain42,
loopback18080 and simulated controller. `docker-compose.jetson.reference.yml` preserves the upstream
hardware recipe for reference only; run `../scripts/ubts_sim.ps1` for local work.
Do not push this simulation overlay to the live dashboard repository.

The optional local GPU server uses D3D12/Ogre 1 for RGB only, with a separate
Embree 2D LiDAR process reading the original static visual meshes. The default
original/lite/seed=7 orchard retains 768,113 triangles. Mixed GPU camera/GPU
LiDAR rendering produced black images and degenerate scans, so it is not used.
GUI clients retain CPU rendering. The simulator's DDS profile reserves a 16 MiB
shared-memory segment for RGB frames; GUI containers retain loopback UDP.

The CPU ray model treats triangles as opaque and excludes the rover itself.
Other dynamic models/actors or unsupported geometry in the world file fail
startup; runtime moving/inserted objects are unsupported. Full model attitude
tilts the scan plane, which can intersect terrain. Numerical equivalence with
Gazebo GPU LiDAR has not been established. Ground truth places the synthetic
sensor and generates explicitly virtual GPS. Current mapping/control `/odom`
integrates signed wheel distance using IMU heading relative to its initial
orientation; raw `/sim/wheel_odom` remains available. Encoder yaw is used only
to recover signed travel, never as the estimated heading. Position still has
uncorrected forward-slip/lateral-skid error and no loop closure. Initial heading
interpolation and bounded gyro prediction do not substitute for physical-turn
validation. `simulation_status.json` identifies `wheel_distance_imu_heading`.
Historical archive metadata, including `sim-wheel-odom-logodds-v1`, is immutable
and does not by itself identify whether that run used the newer IMU estimator.

Local motion assumptions are 1.0 m/s maximum wheel speed and 1.5 m/s2
acceleration. Source physics_hz is now 120 (250 and 100 were also tested); updated motion and
performance checks are required. These settings do not change Jetson hardware,
ESP firmware, real PWM calibration or the real dashboard's drive configuration.

Map/autonomy unit and browser-fixture checks have passed. Initial combined
GPU-camera/CPU-LiDAR tests passed sensor/manual-motion/STOP contracts. The short
post-DDS 250 Hz physics measurement was about 23.20 FPS at Gazebo and 21.55 FPS
at ROS RGB. The subsequent 45-second 120 Hz sensor run produced valid RGB at
28.534 wall FPS / 29.988 sim FPS and scans at 9.519 wall Hz / 10.004 sim Hz,
with RTF about 0.9515 and no black/malformed/degenerate samples. This passes
the sensor target's 5% tolerance, not complete 30 FPS browser delivery. Full
Gazebo autonomy, recording integration, the new IMU estimator's physical turns and sustained
performance remain separately tracked. See
`../artifacts/validation/gpu-embree-120hz-sensor.json` for the measured run.

Live occupancy mapping now defaults to one million observed cells within the
unchanged 1024-by-1024 span; the dense cell-value array is about 1 MiB, not the
entire mapper's memory footprint. Archive regeneration retains the recorded
configuration, including the older 300,000-cell limit.
`-Renderer auto` requires the local UBTS_GPU_VALIDATED=1 gate plus eligible WSLg
hardware before choosing GPU; the gate is unset by default. See
`../docs/11_ubts_local_simulation.md`, `../docs/13_wsl_renderer_probes.md` and
`../docs/UBTS_TODO.md` for evidence and the current checkpoint.

No real recordings or .env were copied from Jetson. Deployment helpers and nested
workflow files are source references and are not invoked by the simulation.


2026-09-30: common camera preview delta `42f7e3d..47bdb5bbafd7c8535d532b34738eea20efff9202` merged. SIM camera/clock/map/control additions retained. Source validation: 56 Linux tests and 9 JavaScript suites. Active SIM runtime is not replaced by this source checkpoint.

2026-09-30 follow-up: the native-IMU recording label from `72c9444` is in this
subtree. Sanitized Jetson IMU/camera/label/four-sensor deployment evidence from
`00ec5fa` and GPS RF diagnosis/documentation from `c675519` are copied into
`docs/`. The actual Jetson now runs the corresponding scoped sensor deployments;
its USB GPS communicates but has no position fix in the observed setup.
Diagnostic documentation from upstream `eb7bea1` adds GPS follow-up002,
`docs/jetson_gps_antenna_diagnosis.json` (SHA256
`54d2d18b645d61d32285ecbf5ec50e7c55555da7aed8bd64c7977113af3bd5de`)
and synchronizes `docs/gps_offline_map.md`: four empty satellite lists, one
zero-C/N0 search entry and five false antenna-supervisor RAM values. These values
do not establish physical antenna power OFF. Subsequent Windows-PC observations
are preserved in `docs/windows_gps_reception_diagnosis.json`: 2,745 valid packets,
zero rejected, positive GNSS tracking but all 45 NAV-PVT samples without a fix.
Both the GPS board and the same external antenna were moved toward a window
while host and USB cable also changed;
these observations do not isolate a cause. Both reads closed the port without
receiver configuration writes. Controlled comparison and outdoor fix remain pending.
Upstream `730a26e` adds `docs/windows_gps_placement_comparison.json` for the same PC
and C-to-C cable with both board and antenna moved back indoors: 922 valid
packets, zero rejected, no locked signals in 14 reports and no fix in 15 PVT
samples. Placement-related reception differs; sole cause, same-satellite
attenuation and a Jetson/cable defect are not established.
The existing initial diagnosis JSON is immutable. No SIM README or runtime was
replaced; this subtree has no `RAW_DEPLOYMENT.md` to mirror.
These actual-device observations do not describe simulated GPS accuracy and
do not imply that the visible SIM containers were replaced.
