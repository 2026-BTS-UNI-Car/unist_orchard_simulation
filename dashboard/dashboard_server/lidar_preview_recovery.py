"""Verified, bounded display-only recovery for missing recorded LiDAR previews.

Build explicitly/offline; HTTP reads finished eight-second chunks only. Original
bags, timelines, mapping inputs and sensor coordinates are never overwritten.
"""
import gzip,hashlib,json,math,re,uuid
from datetime import datetime
from pathlib import Path

SCHEMA='ubts.recovered_lidar_preview.v2'
CHUNK_S=8
MAX_POINTS=1080

def digest(path):
 h=hashlib.sha256()
 with path.open('rb') as stream:
  for block in iter(lambda:stream.read(4*1024*1024),b''):h.update(block)
 return h.hexdigest()

def identity(archive):
 source=archive/'mapping_source/sensors.jsonl.gz'
 manifest=json.loads((archive/'mapping_source/manifest.json').read_text())
 sha=(manifest.get('output_sha256') or {}).get('sensors.jsonl.gz')
 if manifest.get('complete') is not True or not re.fullmatch('[0-9a-f]{64}',str(sha)):
  raise ValueError('verified_native_source_required')
 metadata=archive/'metadata.json'
 return source,sha,digest(metadata)

def folder(archive,cache,sha,metadata_sha):
 return Path(cache)/'recovered-lidar'/archive.name/(sha+'-'+metadata_sha[:16]+'-v2')

def frame(row,start):
 m=row['message'];values=m['ranges'];n=len(values)
 if not 1<=n<=4096:raise ValueError('preview_scan_capacity')
 amin,step,rmin,rmax=(float(m[k]) for k in ('angle_min','angle_increment','range_min','range_max'))
 if not all(math.isfinite(v) for v in (amin,step,rmin,rmax)) or not 0<rmin<rmax<=100 or not 0<step<=2*math.pi:
  raise ValueError('preview_scan_geometry')
 points=[]
 for i in range(0,n,max(1,math.ceil(n/MAX_POINTS))):
  r=values[i]
  if type(r) not in (int,float) or not math.isfinite(r) or not rmin<=r<=rmax:continue
  a=amin+i*step
  # Same bridge XY convention/rounding as the original archive recorder.
  points.append([round(r*math.sin(a),3),round(r*math.cos(a),3),round(r,3),32,round(math.degrees(a)%360,2)])
 return dict(t_s=round(max(0,row['bag_time_ns']/1e9-start),3),frame_id=m['header']['frame_id'],
  stamp=m['header']['stamp'],coordinate_frame='lidar_bridge_xy_v1',points=points,
  metrics=dict(source_points=n,rendered_points=len(points),scan_time_s=m.get('scan_time'),
   time_increment_s=m.get('time_increment'),range_limit_m=rmax),source='verified_raw_preview_recovery')

def build(archive,cache):
 archive=Path(archive).resolve(strict=True);cache=Path(cache)
 source,sha,metadata_sha=identity(archive)
 metadata=json.loads((archive/'metadata.json').read_text())
 if metadata.get('state')!='archived':raise ValueError('finished_recording_required')
 target=folder(archive,cache,sha,metadata_sha)
 if target.exists():
  window(archive,cache,0,.5)
  return json.loads((target/'manifest.json').read_text())
 signature=(source.stat().st_size,source.stat().st_mtime_ns)
 if digest(source)!=sha:raise ValueError('raw_sensor_checksum_mismatch')
 start=datetime.fromisoformat(metadata['started_at']).timestamp()
 temporary=target.with_name(target.name+'.partial-'+uuid.uuid4().hex);temporary.mkdir(parents=True)
 files={};index=None;rows=[];imu_rows=[];count=0;imu_count=0;previous=-1.
 def flush(index,rows,imu_rows):
  name=f'chunk-{index:06d}.json.gz';path=temporary/name
  data=json.dumps(dict(frames=rows,imu_frames=imu_rows),allow_nan=False,separators=(',',':')).encode()
  if len(data)>16*1024*1024:raise ValueError('recovered_chunk_capacity')
  with path.open('wb') as raw:
   with gzip.GzipFile(filename='',mode='wb',fileobj=raw,compresslevel=1,mtime=0) as output:output.write(data)
  files[str(index)]=dict(name=name,bytes=path.stat().st_size,sha256=digest(path),frames=len(rows))
 with gzip.open(source,'rt',encoding='utf8') as stream:
  for line in stream:
   if len(line)>2*1024*1024:raise ValueError('source_row_capacity')
   row=json.loads(line)
   if row.get('topic') not in ('/scan','/imu/data_raw'):continue
   receipt=row['bag_time_ns']
   if type(receipt) is not int or receipt<0:raise ValueError('invalid_recorded_receipt_time')
   t=round(max(0,receipt/1e9-start),3)
   if t<previous:raise ValueError('recorded_receipt_clock_reversed')
   previous=t;new_index=int(t//CHUNK_S)
   if index is not None and new_index!=index:
    flush(index,rows,imu_rows);rows=[];imu_rows=[]
   index=new_index
   if row['topic']=='/scan':rows.append(frame(row,start));count+=1
   else:
    from .imu_processing import quaternion_to_euler_deg
    m=row['message'];cov=m.get('orientation_covariance',[-1])
    euler=quaternion_to_euler_deg(m['orientation'].get(k) for k in ('x','y','z','w')) if cov[0]!=-1 else {}
    raw={k:m[k] for k in ('angular_velocity','linear_acceleration')}
    raw.update(frame_id=m['header']['frame_id'],stamp=m['header']['stamp'],euler_deg=euler)
    imu_rows.append(dict(t_s=t,raw=raw,stamp=raw['stamp']));imu_count+=1
 if rows or imu_rows:flush(index,rows,imu_rows)
 if not count:raise ValueError('no_recorded_scans')
 if signature!=(source.stat().st_size,source.stat().st_mtime_ns) or digest(source)!=sha or digest(archive/'metadata.json')!=metadata_sha:
  raise ValueError('recording_changed_during_recovery')
 value=dict(schema=SCHEMA,complete=True,archive_id=archive.name,source_sensor_sha256=sha,
  metadata_sha256=metadata_sha,frames=count,imu_frames=imu_count,chunk_s=CHUNK_S,files=files,
  source_stat=list(signature),
  duration_s=max(float(metadata.get('duration_s') or 0),previous),display_point_limit=MAX_POINTS,
  original_files_preserved=True,mapping_inputs_changed=False)
 (temporary/'manifest.json').write_text(json.dumps(value,separators=(',',':')),encoding='utf8')
 try:temporary.rename(target)
 except FileExistsError:
  # Another explicit offline builder won. Never replace its published result.
  other=json.loads((target/'manifest.json').read_text())
  if other!=value:raise ValueError('concurrent_recovery_generation_mismatch')
 return value

def window(archive,cache,start_s,duration_s):
 archive=Path(archive);source,sha,metadata_sha=identity(archive)
 target=folder(archive,cache,sha,metadata_sha);manifest_file=target/'manifest.json'
 if not manifest_file.is_file():return None
 if target.is_symlink() or manifest_file.stat().st_size>1024*1024:raise ValueError('invalid_recovery_manifest')
 value=json.loads(manifest_file.read_text())
 if (value.get('schema')!=SCHEMA or value.get('complete') is not True or value.get('archive_id')!=archive.name
  or value.get('source_sensor_sha256')!=sha or value.get('metadata_sha256')!=metadata_sha
  or value.get('chunk_s')!=CHUNK_S
  or value.get('source_stat')!=[source.stat().st_size,source.stat().st_mtime_ns]):raise ValueError('incompatible_recovery_manifest')
 start=float(start_s or 0);duration=float(duration_s or 8)
 if not math.isfinite(start) or not math.isfinite(duration):raise ValueError('invalid_preview_window')
 start=max(0,start);duration=min(24.,max(.5,duration))
 rows=[];imus=[]
 for index in range(max(0,int(start//CHUNK_S)-1),int((start+duration)//CHUNK_S)+1):
  entry=value['files'].get(str(index))
  if entry is None:continue
  name=f'chunk-{index:06d}.json.gz'
  if entry.get('name')!=name:raise ValueError('invalid_recovery_chunk_path')
  path=target/name
  if path.is_symlink() or path.stat().st_size>8*1024*1024 or digest(path)!=entry['sha256']:raise ValueError('recovery_chunk_checksum')
  with gzip.open(path,'rb') as stream:data=stream.read(16*1024*1024+1)
  if len(data)>16*1024*1024:raise ValueError('recovery_chunk_capacity')
  part=json.loads(data);rows.extend(part['frames']);imus.extend(part['imu_frames'])
 def select(items):
  selected=[r for r in items if start<=r['t_s']<=start+duration]
  before=[r for r in items if r['t_s']<start]
  if before:selected.insert(0,before[-1])
  return selected
 selected=select(rows)
 telemetry=dict(duration_s=value['duration_s'],imu=dict(frames=select(imus)),source='verified_raw_preview_recovery')
 metadata=json.loads((archive/'metadata.json').read_text())
 from .motor_pwm import MOTOR_TIMELINE_SCHEMA,pin_metadata,MOTOR_PIN_PROVENANCE
 if (metadata.get('timeline') or {}).get('motor_schema')==MOTOR_TIMELINE_SCHEMA:
  telemetry['motor']=dict(schema=MOTOR_TIMELINE_SCHEMA,enabled=True,frames=[],frames_file='motor.jsonl',
   pins=pin_metadata(),pin_mapping_source=MOTOR_PIN_PROVENANCE,
   sampling='status_snapshots_not_all_serial_edges',reported_pwm_semantics='last_cached_mcu_report_not_voltage_or_measured_speed')
 return dict(frames=selected,source='verified_raw_preview_recovery',source_sensor_sha256=sha,
  original_preview_missing=True,complete=True,total_source_frames=value['frames'],
  telemetry=telemetry)
