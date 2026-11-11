"""Bounded slow/failing writer behavior without ROS or real runtime files."""
import json
from pathlib import Path
import sys
import threading

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parent))
from snapshot_writer import LatestSnapshotWriter


def test_blocked_writer_keeps_one_latest_bundle_and_original_capture_timestamps():
    entered=threading.Event();release=threading.Event();writes=[]
    def save(name,payload):
        if not writes:entered.set();assert release.wait(2)
        writes.append((name,dict(payload)))
    writer=LatestSnapshotWriter(save)
    try:
        writer.submit({'simulation_status.json':{'updated_at':10.,'generation':0}})
        assert entered.wait(1)
        # These calls must complete while the I/O worker is still blocked.
        for i in range(1,101):writer.submit({'simulation_status.json':{'updated_at':10.+i,'generation':i},
                                            'rover_bridge_status.json':{'updated_at':10.+i,'generation':i}})
        state=writer.status()
        assert state['pending_bundles']==state['inflight_bundles']==1
        assert state['coalesced']==99 and state['submitted']==101
        release.set();assert writer.close(timeout=2)
        assert [(name,p['generation']) for name,p in writes]==[
            ('simulation_status.json',0),('simulation_status.json',100),('rover_bridge_status.json',100)]
        assert [p['updated_at'] for _,p in writes]==[10.,110.,110.]
        assert writer.status()['completed']==2 and writer.status()['failures']==0
        assert writer.submit({'simulation_status.json':{}}) is False
    finally:release.set();writer.close(timeout=2)


def test_file_failure_is_contained_visible_and_does_not_stop_later_files_or_bundles(tmp_path):
    failed=threading.Event();records=[]
    def save(name,payload):
        if payload.get('fail'):
            failed.set();raise OSError('private-filename-or-token')
        records.append((name,payload))
    writer=LatestSnapshotWriter(save)
    try:
        writer.submit({'simulation_status.json':{'fail':True},'rover_bridge_status.json':{'armed':False}})
        assert failed.wait(1)
        writer.submit({'simulation_status.json':{'loop_timing':{'first_autonomy_failure':{'reason':'imu_synchronization_timeout'}}}})
        assert writer.close(timeout=2)
        state=writer.status()
        assert state['failures']==1 and state['last_error']=={'file':'simulation_status.json','error':'OSError'}
        assert 'private-' not in json.dumps(state)
        assert records[-1][1]['loop_timing']['first_autonomy_failure']['reason']=='imu_synchronization_timeout'
        assert any(name=='rover_bridge_status.json' for name,_ in records)
    finally:writer.close(timeout=2)


def test_bounded_close_does_not_wait_forever_on_filesystem_and_rejects_new_work():
    entered=threading.Event();release=threading.Event()
    def save(*args):entered.set();assert release.wait(2)
    writer=LatestSnapshotWriter(save)
    try:
        writer.submit({'simulation_status.json':{'updated_at':1.}})
        assert entered.wait(1)
        assert writer.close(timeout=0) is False
        assert writer.status()['closing']
        assert writer.submit({'simulation_status.json':{}}) is False
    finally:release.set();assert writer.close(timeout=2)


@pytest.mark.parametrize('payload',[{},[],{'mapping_sample.json':{}},{'../escape.json':{}}])
def test_only_bounded_status_file_set_is_accepted(payload):
    writer=LatestSnapshotWriter(lambda *args:None)
    try:
        with pytest.raises(ValueError):writer.submit(payload)
        assert writer.status()['submitted']==0
    finally:assert writer.close(timeout=1)
