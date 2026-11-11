import math
import tempfile
import unittest
from pathlib import Path
import numpy as np
from dashboard_server.live_mapping import LiveMapping, attitude, translation
from dashboard_server.autonomy_settings import AutonomySettings
from dashboard_server.archive_mapping import ArchiveMapping,renderer_snapshot


def frame(stamp=10.,yaw=0.):
    # Unequal radial features yield two-dimensional registration support.
    points=[dict(angle_deg=i,distance_m=4+.5*math.sin(i*.11)+.3*math.cos(i*.17)) for i in range(360)]
    return (dict(stamp=stamp,points=points),stamp),(dict(imu_topic_alive=True,raw=dict(stamp=dict(sec=int(stamp),nanosec=int((stamp-int(stamp))*1e9)),orientation=dict(x=0,y=0,z=math.sin(yaw/2),w=math.cos(yaw/2)))),stamp)


class Tests(unittest.TestCase):
    def test_native_restart_preserves_map_but_requires_new_localization(self):
        with tempfile.TemporaryDirectory() as root:
            mapper=LiveMapping(root,start_worker=False);mapper.action('start')
            scan,imu=frame();mapper.ingest(scan,imu,10.)
            mapper.action('stop');saved=mapper.status();cells=mapper.map.cells.copy()
            restored=LiveMapping(root,start_worker=False)
            self.assertEqual(restored.status()['grid'],saved['grid'])
            self.assertEqual(restored.map.cells,cells)
            self.assertEqual(restored.status()['path'],saved['path'])
            self.assertEqual(restored.status()['map_epoch'],saved['map_epoch'])
            self.assertFalse(restored.tracking);self.assertFalse(restored.status()['active'])
            self.assertFalse(restored.status()['runtime_authority'])
            self.assertTrue(restored.status()['persistence']['native_cells'])
            with self.assertRaises(ValueError):restored.action('start')
            scan,imu=frame(30.);self.assertFalse(restored.ingest(scan,imu,30.))
            self.assertEqual(restored.map.cells,cells)
            restored.action('reset');restored.action('start')
            self.assertTrue(restored.ingest(scan,imu,30.))
            self.assertEqual(restored.status()['scan_count'],1)
            self.assertNotEqual(restored.status()['map_epoch'],saved['map_epoch'])

    def test_legacy_saved_png_survives_restart_without_claiming_native_cells(self):
        from dashboard_server.occupancy_map import atomic_json
        with tempfile.TemporaryDirectory() as root:
            mapper=LiveMapping(root,start_worker=False);mapper.action('start')
            scan,imu=frame();mapper.ingest(scan,imu,10.);saved=mapper.status()
            atomic_json(Path(root)/'last_map.json',saved)
            restored=LiveMapping(root,start_worker=False)
            self.assertEqual(restored.status()['grid'],saved['grid'])
            self.assertEqual(restored.status()['scan_count'],1)
            self.assertTrue(restored.status()['persistence']['legacy_PNG_only'])
            self.assertEqual(restored.map._cell_count,0)
            with self.assertRaises(ValueError):restored.action('start')
            restored.action('stop');again=LiveMapping(root,start_worker=False)
            self.assertEqual(again.status()['grid'],saved['grid'])

    def test_corrupt_native_restart_cannot_install_partial_cells(self):
        import json
        with tempfile.TemporaryDirectory() as root:
            mapper=LiveMapping(root,start_worker=False);mapper.action('start')
            scan,imu=frame();mapper.ingest(scan,imu,10.);mapper.action('stop')
            path=Path(root)/'last_native_map.json';value=json.loads(path.read_text())
            value['cell_count']+=1;path.write_text(json.dumps(value))
            restored=LiveMapping(root,start_worker=False)
            self.assertFalse(restored.status()['persistence']['restored'])
            self.assertEqual(restored.status()['scan_count'],0)
            self.assertEqual(restored.map._cell_count,0)
            self.assertFalse(restored.tracking)

    def test_native_write_failure_still_stops_and_retains_legacy_png(self):
        from unittest.mock import patch
        from dashboard_server.tiled_occupancy import TiledOccupancyMap
        with tempfile.TemporaryDirectory() as root:
            mapper=LiveMapping(root,start_worker=False);mapper.action('start')
            scan,imu=frame();mapper.ingest(scan,imu,10.);saved=mapper.status()
            with patch.object(TiledOccupancyMap,'save_checkpoint',side_effect=OSError('disk failed')):
                stopped=mapper.action('stop')
            self.assertFalse(stopped['active']);self.assertEqual(stopped['state'],'paused')
            self.assertFalse(stopped['persistence']['saved'])
            self.assertEqual(stopped['grid'],saved['grid'])
            restored=LiveMapping(root,start_worker=False)
            self.assertEqual(restored.status()['grid'],saved['grid'])
            self.assertTrue(restored.status()['persistence']['legacy_PNG_only'])

    def test_transient_sensor_pairing_waits_without_insertion_or_reset(self):
        with tempfile.TemporaryDirectory() as root:
            mapper=LiveMapping(root,start_worker=False);mapper.action('start')
            for reason in ('lidar_or_imu_unavailable','stale_lidar_or_imu','unaligned_lidar_imu','imu_orientation_unavailable'):
                mapper.freeze(reason)
                self.assertTrue(mapper.tracking);self.assertTrue(mapper.status()['active'])
                self.assertEqual(mapper.status()['state'],'waiting_for_sensors')
                self.assertEqual(mapper.status()['scan_count'],0)
            scan,imu=frame();self.assertTrue(mapper.ingest(scan,imu,10.))
            saved=mapper.status();mapper.freeze('unaligned_lidar_imu')
            self.assertEqual(mapper.status()['grid'],saved['grid'])
            scan,imu=frame(10.5);self.assertTrue(mapper.ingest(scan,imu,10.5))
            self.assertEqual(mapper.status()['scan_count'],2)
            mapper.freeze('stale_lidar_or_imu');scan,imu=frame(14.)
            with self.assertRaisesRegex(ValueError,'scan_clock_gap'):mapper.ingest(scan,imu,14.)
            mapper.freeze('scan_clock_gap_or_reset')
            self.assertFalse(mapper.tracking);self.assertEqual(mapper.status()['state'],'needs_reset')

    def test_map_stop_preserves_reset_clears_and_repeated_frame_is_not_new_scan(self):
        with tempfile.TemporaryDirectory() as root:
            mapper=LiveMapping(root,start_worker=False);mapper.action('start')
            scan,imu=frame();self.assertTrue(mapper.ingest(scan,imu,10.))
            saved=mapper.status();self.assertFalse(mapper.ingest(scan,imu,10.))
            self.assertEqual(mapper.status()['scan_count'],1)
            mapper.action('stop');self.assertEqual(mapper.status()['grid'],saved['grid'])
            self.assertEqual(mapper.status()['path'],saved['path'])
            mapper.action('reset');self.assertEqual(mapper.status()['scan_count'],0)
            self.assertNotEqual(saved['map_epoch'],mapper.status()['map_epoch'])

    def test_stale_tilt_and_source_clock_gap_rejected_without_adding_cells(self):
        with tempfile.TemporaryDirectory() as root:
            mapper=LiveMapping(root,start_worker=False);mapper.action('start')
            scan,imu=frame();mapper.ingest(scan,imu,10.)
            saved=mapper.status()
            for now in (9.,12.):
                with self.assertRaises(ValueError):mapper.ingest(scan,imu,now)
            scan,imu=frame(14.)
            with self.assertRaisesRegex(ValueError,'scan_clock_gap'):mapper.ingest(scan,imu,14.)
            self.assertEqual(mapper.status(),saved)
            mapper.freeze('uncertain');self.assertFalse(mapper.status()['active'])
            with self.assertRaises(ValueError):mapper.action('start')
            self.assertFalse(mapper.status()['runtime_authority'])

    def test_known_translation_display_estimate_and_degenerate_support_rejection(self):
        rng=np.random.default_rng(7);previous=rng.uniform(-4,4,(400,2))
        shift,quality=translation(previous,previous-[.08,-.04],0)
        np.testing.assert_allclose(shift,[.08,-.04],atol=.01)
        line=np.column_stack((np.linspace(-4,4,200),np.zeros(200)))
        with self.assertRaises(ValueError):translation(line,line,0)

    def test_settings_are_persistent_and_invalid_values_do_not_write(self):
        with tempfile.TemporaryDirectory() as root:
            path=Path(root)/'settings.json';settings=AutonomySettings(path)
            settings.set_speed(45);before=path.read_bytes()
            for value in (True,-1,101,'50',math.nan,math.inf):
                with self.assertRaises(ValueError):settings.set_speed(value)
                self.assertEqual(path.read_bytes(),before)
            status=AutonomySettings(path).status()
            self.assertEqual(status['speed_percent'],45)
            self.assertFalse(status['enabled']);self.assertFalse(status['motor_command'])

    def test_archive_rejects_traversal_and_symlinks(self):
        with tempfile.TemporaryDirectory() as root:
            archives=ArchiveMapping(root)
            for name in ('..','../outside','/outside','missing'):
                with self.assertRaises(ValueError):archives.status(name)

    def test_renderer_xy_bounds_and_y_flip_preserve_geometry(self):
        import base64
        import cv2
        image=np.full((50,100,3),146,dtype=np.uint8);image[0,0]=25;image[-1,-1]=245
        ok,png=cv2.imencode('.png',image);self.assertTrue(ok)
        value=renderer_snapshot(png,{'render_world_bounds_m':[-3.,7.,-2.,3.],'frame_index':4},'fixture')
        self.assertEqual(value['grid']['origin'],{'x':-3.,'y':-2.})
        self.assertEqual(value['grid']['resolution'],.1)
        self.assertEqual(value['grid']['occupied_bounds']['minX'],-3.)
        self.assertAlmostEqual(value['grid']['occupied_bounds']['minY'],2.9)
        self.assertAlmostEqual(value['grid']['occupied_bounds']['maxY'],3.)
        decoded=cv2.imdecode(np.frombuffer(base64.b64decode(value['grid']['image_base64']),dtype=np.uint8),cv2.IMREAD_COLOR)
        self.assertEqual(int(decoded[-1,0,0]),25);self.assertEqual(int(decoded[0,-1,0]),245)


if __name__=='__main__':unittest.main()
