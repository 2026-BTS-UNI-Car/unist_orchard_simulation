"""IdentRig: the T1 rig for the identification sessions (plant side; design 3.9, 8.3 P4; T1_RIG_KO.md).

A subclass of steering.t1_rig.T1Rig; nothing of the rig is changed. Two modes on the one SIM clock:

  bench   the bench tool (ident/bench.py) owns the serial line, like a laptop on the ESP's USB port with the bridge
          stopped: the bridge core is not stepped; host -> MCU bytes are written by the bench tool at the plant-step
          boundary it writes them (the rig's UART rule from that tick); MCU -> host lines go to a separate
          MemoryTransport the bench tool reads (delivered at LF done + 1 ms like every rig line).
  drive   the bridge runs as in T1Rig (SteerBridgeCore + SteerControl + EspSteerV3, 20 ms grid); the operator uses
          the control API (RigClient). enter_drive_mode() rebuilds the host side with the calibration that the bench
          phase measured (new cal_id, mark angles), like the operator updating the host profile after B2/B3.

Record hooks (ident/recorder.py): sensor packets (LiDAR scans, IMU samples) go to the active archive recorder in
arrival order; in drive mode every MCU status line the bridge read becomes a steering sidecar row
(SteerBridgeCore.steering_state, the wrapper's /rover/steering/state) and the bridge snapshot is written to motor.jsonl
at 2 Hz (the /rover/status rate). set_lidar(False) skips the scan synthesis (bench phases: no LiDAR record).

SIM-only stand-in for 'wheels lifted' (B7 load): set_wheels_lifted(True) blocks forward motion (plant
forward_blocked), so the drive duty is applied while the car stays.

Truth: this module is plant side. Analysis code reads only the record files.
"""
from __future__ import annotations

from dataclasses import replace
import math

from .. import t1_rig as tr
from . import yard as yard_mod

ENV_NOTE = 'IdentRig is a T1Rig subclass (steering/t1_rig.py is not changed)'


class IdentRigError(tr.RigError):
    pass


class IdentRig(tr.T1Rig):
    """T1Rig with bench / drive modes and record hooks (module docstring). Starts in bench mode."""

    def __init__(self, config=None, *, native_executable=None, build_dir=None, dashboard_dir=None, compiler=None):
        self._bench_mode = True
        self._lidar_on = False
        self._recorder = None
        self._bench_reader = None
        self._motor_period_ms = 500
        super().__init__(config, native_executable=native_executable, build_dir=build_dir,
                         dashboard_dir=dashboard_dir, compiler=compiler)
        self.bench_transport = self.v3.MemoryTransport(clock=self.clock, path='bench')
        self.bench_transport.open()
        self._drive_transport = self.transport
        self.transport = self.bench_transport            # MCU lines go to the bench tool
        self.host_rebuilds = 0

    # -- world -------------------------------------------------------------------------------------------------------
    def _make_world(self, spec):
        if isinstance(spec, yard_mod.YardSpec):
            lidar_seed = tr.stream_seed(self.seed, self.stream_key, tr.PURPOSES['lidar_noise'])
            world = yard_mod.Yard(spec.config)
            caster, lidar = yard_mod.make_yard_lidar(world, lidar_seed,
                                                      noise_sd=float(self.profile.value('lidar_noise_sd_m')))
            return world, 'W0', caster, lidar
        return super()._make_world(spec)

    def identity(self, include_files=True):
        out = super().identity(include_files=include_files)
        if isinstance(self.config.world, yard_mod.YardSpec):
            out['world'] = self.world.describe()
        return out

    # -- modes -------------------------------------------------------------------------------------------------------
    @property
    def bench_mode(self):
        return self._bench_mode

    def attach_bench_reader(self, reader):
        self._bench_reader = reader

    def bench_write(self, data, note='bench'):
        """Bench tool -> MCU bytes, written now (the next plant step starts at self.ms)."""
        if not self._bench_mode:
            raise IdentRigError('bench_write in drive mode: the bridge owns the serial line')
        self._to_mcu(self.ms, bytes(data), note)

    def enter_drive_mode(self, calibration=None, steer_limit=None):
        """Stop the bench tool, rebuild the host side (host profile with the measured calibration) and let the bridge
        run. calibration: host profile 'calibration' section values (cal_id, mark_left_mrad, mark_right_mrad,
        permille_per_deg, expected_soft_permille); steer_limit: the 'steer_limit' section. None keeps the rig's."""
        if not self._bench_mode:
            raise IdentRigError('already in drive mode')
        overrides = dict(self.config.host_override_dict())
        host_cal = self.config.cal_id if self.config.host_cal_id == 'same' else self.config.host_cal_id
        if calibration:
            overrides['calibration'] = dict(overrides.get('calibration', {}), **calibration)
            host_cal = calibration.get('cal_id', host_cal)
        if steer_limit:
            overrides['steer_limit'] = dict(overrides.get('steer_limit', {}), **steer_limit)
        old_link = self.link
        config = replace(self.config, host_cal_id=host_cal, host_overrides=tuple(sorted(overrides.items())))
        self._build_host(self.profile, config)
        if old_link is not None and old_link.state != 'closed':
            try:
                old_link.close('host_rebuild')
            except OSError:
                pass
        self.bench_transport.close()
        self._drive_transport = self.transport
        self._bench_mode = False
        self.host_rebuilds += 1
        self.default_client = None
        return self.hp

    # -- stepping hooks -----------------------------------------------------------------------------------------------
    def _bridge_step(self):
        if self._bench_mode:
            self._bridge_done_ms = self.ms
            self.counters['bridge_steps_bench_mode'] += 1
            return
        super()._bridge_step()
        rec = self._recorder
        if rec is not None and rec.wants_steering:
            for line, received in self.link.take_status_history():
                rec.add_steering(self.core.steering_state(line=line, received=received))
        elif rec is None:
            self.link.take_status_history()

    def _mcu_and_plant_step(self):
        super()._mcu_and_plant_step()
        if self._bench_mode and self._bench_reader is not None:
            self._bench_reader.poll()
        rec = self._recorder
        if rec is not None and not self._bench_mode and self.ms % self._motor_period_ms == 0:
            rec.add_motor(self.core.snapshot, self.wall_clock(), self.clock(), self.link.status)

    def _sensors(self):
        before = len(self._packets)
        if self._lidar_on:
            super()._sensors()
        else:
            saved = self.lidar_period_ms
            self.lidar_period_ms = 1 << 62          # no scan this step (t % period == 0 only at t = 0)
            try:
                super()._sensors()
            finally:
                self.lidar_period_ms = saved
        new = self._packets[before:]
        del self._packets[before:]
        rec = self._recorder
        if rec is not None:
            for packet in new:
                rec.add_packet(packet, self.wall_clock_at(packet['arrival_s']))

    def wall_clock_at(self, sim_s):
        """Wall clock of a SIM time (the recorder's receipt clock: epoch + SIM time + wall jumps so far)."""
        return tr.WALL_EPOCH_S + float(sim_s) + self._wall_offset_s

    # -- recorder / sensors / SIM stand-ins ------------------------------------------------------------------------------
    def attach_recorder(self, recorder):
        if self._recorder is not None and recorder is not None:
            raise IdentRigError('a recorder is already attached')
        self._recorder = recorder

    @property
    def recorder(self):
        return self._recorder

    def set_lidar(self, on):
        self._lidar_on = bool(on)

    def set_wheels_lifted(self, lifted):
        """SIM stand-in for wheels lifted off the ground (B7 load): forward motion blocked, duty applied."""
        self.plant.forward_blocked = bool(lifted)

    def place(self, x, y, yaw):
        """SIM stand-in for the operator putting the stopped car back on a start mark between drives (in the field
        they drive or push it there). Standing still only; the steering and the MCU are not touched."""
        p = self.plant
        if abs(p.v) > 1e-3:
            raise IdentRigError('place() needs the car standing still (v = %.3f m/s)' % p.v)
        turn = math.atan2(math.sin(yaw - p.yaw), math.cos(yaw - p.yaw))
        p.x, p.y, p.yaw = float(x), float(y), float(math.atan2(math.sin(yaw), math.cos(yaw)))
        p.yaw_unwrapped += turn

    # -- evaluation helpers (plant side; never analysis input) ------------------------------------------------------------
    def operator_view(self):
        """What the operator sees of the car (pose, speed): truth, used only by the SIM operator scripts."""
        p = self.plant
        return dict(x=p.x, y=p.y, yaw=p.yaw, v=p.v)

    def clearance_m(self):
        world = self.world
        if hasattr(world, 'min_distance'):
            return world.min_distance(self.plant.x, self.plant.y)
        return math.inf
