"""Read-only host view for the vendored simulation dashboard."""
from .telemetry import TelemetryProvider, _load_json
from pathlib import Path
import os
import time

class SimulationProvider(TelemetryProvider):
    def __init__(self,config_path=None):
        super().__init__(config_path)
        self.mode='simulation'
        self.external_native_pc = os.environ.get('SIM_MAPPING_PROFILE', 'legacy') == 'external_native_pc'
        if self.external_native_pc:
            if any(os.environ.get(name,'0') not in ('','0') for name in ('SIM_NATIVE_VIEW_ENABLED','SIM_NATIVE_OWNER_ENABLED')) or os.environ.get('SIM_MAPPING_SERVICE_SOCKET'):
                raise ValueError('external_native_pc_conflicts_with_old_native_owners_or_service')
            from .external_native_pc import ExternalNativePC
            self.native_view_enabled=False;self.native_view=None
            self.native_owner_enabled=False;self.native_owner=None
            self.source_only_mapping=False;self.mapping=ExternalNativePC()
            self.mapping_operations=self.mapping;self.map_history=None
            return
        self.native_view_enabled = os.environ.get('SIM_NATIVE_VIEW_ENABLED', '0') == '1'
        self.native_view = None
        if self.native_view_enabled:
            from .sim_native_view_proxy import NativeViewProxy
            self.native_view = NativeViewProxy(enabled=True)
        self.native_owner_enabled = os.environ.get('SIM_NATIVE_OWNER_ENABLED', '0') == '1'
        self.native_owner = None
        if self.native_owner_enabled:
            from .sim_native_owner_proxy import NativeOwnerProxy
            self.native_owner = NativeOwnerProxy(enabled=True)
        self.source_only_mapping = os.environ.get('SIM_MAPPING_PROFILE', 'legacy') == 'native_sensors'
        if self.source_only_mapping:
            if os.environ.get('SIM_MAPPING_SERVICE_SOCKET'):
                from .native_mapping_proxy import NativeMappingProxy, RemoteMappingOperations
                self.mapping=NativeMappingProxy(os.environ['SIM_MAPPING_SERVICE_SOCKET'])
                self.mapping_operations=RemoteMappingOperations(self.mapping)
            else:
                from .raw_sim_mapping import RawSimulationMapping
                self.mapping = RawSimulationMapping(
                source_socket=os.environ.get('SIM_RAW_SENSOR_SOCKET', '/run/ubts-sim/sensor-source.sock'),
                state_dir=os.environ.get('SIM_MAPPING_STATE_DIR', '/run/ubts-sim/raw-mapping'),
                archive_dir=self.recording_archive_dir)
                from .sim_mapping_operations import MappingOperations
                self.mapping_operations=MappingOperations(self.mapping,self.mapping)
        else:
            from .sim_mapping import SimulationMapping
            self.mapping = SimulationMapping(
                sample_path=os.environ.get('SIM_MAPPING_SAMPLE_FILE', '/runtime/mapping_sample.json'),
                state_dir=os.environ.get('SIM_MAPPING_STATE_DIR', '/run/ubts-sim/mapping'),
                archive_dir=self.recording_archive_dir)
        if self.native_owner_enabled and hasattr(self, 'mapping_operations'):
            from .sim_native_owner_proxy import NativeObserverOperations
            self.mapping_operations = NativeObserverOperations(self.mapping_operations, self.native_owner)
        from .sim_map_history import MapHistory
        self.map_history = MapHistory(os.environ.get('SIM_MAP_HISTORY_DIR', '/runtime/map-history'))
    def _hostname(self):return 'UBTS local Gazebo simulation'
    def rover_status(self):
        return {**super().rover_status(), 'mode': 'simulation', 'simulated': True}
    def _memory(self):
        result=super()._memory()
        result.update(display_total=result['total'],display_percent=result['percent'],display_label='Docker Desktop VM')
        return result
    def _sim(self):
        data=_load_json('/runtime/simulation_status.json')
        if time.time()-data.get('updated_at',0)>3:
            data['sensors_alive']={}
            data['clock_alive']=False
        return data
    def lidar_status(self):
        alive=self._sim().get('sensors_alive',{}).get('scan',False)
        return {'present':alive,'state':'simulated' if alive else 'waiting_for_simulation','device':'Gazebo RPLIDAR S2','test_supported':False,'holders':[],'test_mode':'ros2_subscribe_only'}
    def camera_status(self):
        alive=self._sim().get('sensors_alive',{}).get('camera',False)
        settings=self.camera_settings()['settings']
        modes=[{'resolution':settings['resolution'],'width':settings['width'],'height':settings['height'],'fps':[settings['fps']],'label':'Gazebo render preset'}]
        return {'present':alive,'state':'simulated' if alive else 'waiting_for_simulation','device':'Gazebo RGB camera','holders':[],'driver':self._camera_driver_status(),'settings':settings,'supported_view_modes':modes,'supported_view_modes_source':'simulation'}
    def camera_settings(self):
        d=self._camera_driver_status();w=d.get('width',640);h=d.get('height',360);fps=d.get('fps',15)
        return {'ok':True,'settings':{'width':w,'height':h,'fps':fps,'resolution':f'{w}x{h}','rotate_180':False,'source':'simulation_preset'}}
    def _disabled(self,*_,**__):return {'ok':False,'state':'simulation_only','error':'hardware_action_unavailable_in_simulation'}
    set_camera_settings=_disabled
    set_network_mode=_disabled
    docker_container_action=_disabled
    set_fan_mode=_disabled
    set_fan_pwm=_disabled
    set_fan_preset=_disabled
    zero_imu=_disabled
    def fan_status(self):return {'supported':False,'state':'not_applicable','simulated':True}
    def network_control_status(self):return {'enabled':False,'state':'simulation_only'}
    def docker(self):return {'available':False,'containers':[],'state':'isolated_simulation','socket_mounted':False}
    def recording_status(self):
        # The event file cannot establish current recorder liveness. Keep the
        # same original-request freshness contract as the deployed dashboard.
        if self.recorder_api_url != 'http://sim:8790':
            return {'ok':False,'active':None,'state':'unknown','simulated':True,
                    'status_fresh':False,'status_source':'unavailable',
                    'error':'simulation_recorder_endpoint_required',
                    'archive_count':self._recording_archive_count()}
        return {**super().recording_status(),'simulated':True}
    def start_recording(self,payload):
        # Hard gate even if a copied environment accidentally contains a live URL.
        if self.recorder_api_url != 'http://sim:8790':
            return {'ok':False,'error':'simulation_recorder_endpoint_required'}
        result=super().start_recording(payload)
        if result.get('ok'):
            recording_id=result.get('recording_id') or (result.get('status') or {}).get('recording_id')
            try:
                result['mapping']=self.mapping.begin_archive(recording_id)
            except (OSError,ValueError) as exc:
                result['mapping']={'ok':False,'error':str(exc)}
        return result
    def stop_recording(self):
        if self.recorder_api_url != 'http://sim:8790':
            return {'ok':False,'error':'simulation_recorder_endpoint_required'}
        # Mark the mapping cutoff at the same user stop request, before waiting
        # for the recorder to drain/finalize image and rosbag queues.
        mapping=self.mapping.finish_archive()
        result=super().stop_recording()
        result['mapping']=mapping
        return result
    def simulation_map(self,since=None,**kwargs):return self.mapping.snapshot(since,**kwargs)
    def simulation_map_action(self,action,**kwargs):return self.mapping.action(action,**kwargs)
    def simulation_map_history(self,history_id=None):
        if self.external_native_pc:return self.mapping.snapshot()
        return self.map_history.listing() if history_id is None else self.map_history.detail(history_id)
    def simulation_archive_map(self,recording_id):return self.mapping.archive_snapshot(recording_id)
    def simulation_archive_regenerate(self,recording_id):return self.mapping.regenerate(recording_id)
    def _services(self,*args,**kwargs):
        services=super()._services(*args,**kwargs)
        services.pop('cloudflared',None)
        sim=self._sim()
        clock_alive=sim.get('clock_alive') is True
        odometry=sim.get('odometry') or {}
        fault=odometry.get('fault')
        if odometry.get('source')=='lidar_scanmatch_imu_heading':
            # Observe mode has no autonomous permit. Manual control availability
            # is independent of scan matching and cannot certify localization.
            control=sim.get('control') or {}
            manual_ready=clock_alive and isinstance(control,dict) and control.get('manual_ready') is True
            contract_valid=(type(odometry.get('localization_valid')) is bool
                            and odometry.get('runtime_authority') is False
                            and odometry.get('autonomy_eligible') is False)
            localized=(clock_alive and contract_valid and not fault
                       and odometry.get('state')=='ready' and odometry['localization_valid'])
            reason=fault or ('simulation_clock_unavailable' if not clock_alive else
                            'invalid_localization_status' if not contract_valid else
                            odometry.get('reason') or 'waiting_for_localization')
            services['simulation']={'ok':clock_alive,'state':'gazebo','domain':42}
            services['rover_bridge'].update(
                # Retain the existing UI field, now explicitly manual-only.
                drive_ready=manual_ready,
                drive_state='ready' if manual_ready else 'waiting_for_simulation',
                drive_fault=fault,
                manual_ready=manual_ready,
                localization_valid=localized,
                localization_state='fault' if fault else 'ready' if localized else 'waiting',
                localization_reason=reason,
                localization_source=odometry['source'],
                autonomy_eligible=False,
                autonomy_reason='encoderless_autonomy_disabled_pending_permit',
                runtime_authority=False,
            )
            return services
        drive_ready=(clock_alive and odometry.get('state')=='ready' and not fault)
        services['simulation']={'ok':clock_alive,'state':'gazebo','domain':42}
        # Connection health is distinct from permission to drive. A latched
        # estimator fault survives STOP/cleanup and must remain visible.
        services['rover_bridge'].update(
            drive_ready=drive_ready,
            drive_state='ready' if drive_ready else 'odometry_fault' if fault else 'waiting_for_simulation',
            drive_fault=fault,
        )
        return services
    def _default_sensors(self):
        result=super()._default_sensors()
        result.pop('mavlink',None);result.pop('encoder',None)
        return result
