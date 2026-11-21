"""Bounded numeric restart state for ONE immutable recorded sensor prefix.

This never restores a live vehicle position. No pickle, external object hooks,
sensor subscriptions or motor commands are used. Invalid state falls back to
replaying the original source; the last complete map remains available.
"""
import gzip,json,math,os,re,uuid,zlib
from dataclasses import asdict
from pathlib import Path
import numpy as np
import scipy
from scipy.spatial import cKDTree
from .playback_mapping import SourceCursorLocalizer,persist_json
from .raw_mapping.raw_localization import Surfaces,source_stamp
from .raw_mapping.gnss_constraints import FixedENU,GNSSFactor
from .integrated_archive import digest,localization_fingerprint,raster_fingerprint
from .tile_transport import StreamingOccupancyMap

SCHEMA='ubts.recorded_prefix_state.v1'
LIMIT=64*1024*1024
SCALARS=('last_seen','gyro_time','gyro_rate','gyro_integral','gyro_valid_s',
         'rejected','accepted','last_geometry_stamp','motion_prior_count','path_break_pending',
         'gyro_time_ns','gyro_valid_ns','heading','heading_time','heading_epoch','heading_gyro','heading_disagreements')
# Since 2026-11-21 the state also holds the heading read from the IMU orientation and the integer gyro clocks.
# Without them a restarted map had no IMU turn for its first fits and differed from an uninterrupted one.
REFERENCE_NUMBERS=('gyro','gyro_valid','heading_slack')

def plain(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    if isinstance(value,dict):return {k:plain(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)):return [plain(v) for v in value]
    return value

def array(value,shape):
    result=np.asarray(value,dtype=float)
    if result.shape!=shape or not np.all(np.isfinite(result)):raise ValueError('invalid_resume_array')
    return result

def number(value,optional=False):
    if optional and value is None:return None
    if type(value) not in (int,float) or not math.isfinite(value):raise ValueError('invalid_resume_number')
    return value

def whole(value,optional=False,limit=2**63-1):
    if optional and value is None:return None
    if type(value) is not int or not 0<=value<=limit:raise ValueError('invalid_resume_integer')
    return value

def reference(value):
    if value is None:return None
    surface=value['surfaces']
    return plain(dict(**{k:value[k] for k in ('stamp','q','pose','gyro','gyro_valid')},
        stamp_ns=value.get('stamp_ns'),gyro_valid_ns=value.get('gyro_valid_ns',0),
        heading=value.get('heading'),heading_epoch=value.get('heading_epoch',0),heading_slack=value.get('heading_slack',0.),
        heading_gyro=value.get('heading_gyro'),
        surfaces=dict(points=surface.points,normals=surface.normals,denominator=surface.denominator,dense=surface.dense)))

def restore_reference(value):
    if value is None:return None
    s=value['surfaces'];n=len(s['points']);m=len(s['dense'])
    if not 60<=n<=4096 or not 120<=m<=4096:raise ValueError('invalid_resume_surface_size')
    points=array(s['points'],(n,2));normals=array(s['normals'],(n,2));dense=array(s['dense'],(m,2))
    if type(s['denominator']) is not int or not 60<=s['denominator']<=4096:raise ValueError('invalid_resume_denominator')
    q=array(value['q'],(4,))
    if abs(np.linalg.norm(q)-1)>1e-5:raise ValueError('invalid_resume_quaternion')
    return dict(stamp=number(value['stamp']),q=q,pose=array(value['pose'],(3,)),
        **{k:number(value[k]) for k in REFERENCE_NUMBERS},
        stamp_ns=whole(value['stamp_ns'],optional=True),gyro_valid_ns=whole(value['gyro_valid_ns']),
        heading=number(value['heading'],optional=True),heading_epoch=whole(value['heading_epoch'],limit=1000000),
        heading_gyro=number(value['heading_gyro'],optional=True),
        surfaces=Surfaces(points,normals,cKDTree(points),s['denominator'],dense,cKDTree(dense)))

def export_localizer(source):
    engine=source.engine;gnss=source.gnss
    indices={factor.receiver_epoch:i for i,factor in enumerate(gnss.accepted)}
    return plain(dict(schema=SCHEMA,libraries=dict(numpy=np.__version__,scipy=scipy.__version__),
        engine=dict(**{k:getattr(engine,k) for k in SCALARS},pose=engine.pose,velocity=engine.velocity,
            direction=engine.direction,position_prior=engine.position_prior,gps_speed=engine.gps_speed,
            last_recovery=engine.last_recovery if math.isfinite(engine.last_recovery) else None,
            heading_q=engine.heading_q,heading_samples=engine.heading_samples,
            previous=reference(engine.previous),anchor=reference(engine.anchor),anchor_is_previous=engine.anchor is engine.previous),
        imus=source.imus,stamps=source.stamps,pending_imus=source.pending_imus,
        input_scans=source.input_scans,note=source.note,basis=source.basis,
        pairs=[dict(pose=p,factor=indices[f.receiver_epoch]) for p,f in source.pairs],
        gnss=dict(origin=None if gnss.origin is None else [gnss.origin.latitude,gnss.origin.longitude],
            seen=sorted(gnss.seen),accepted=[asdict(f) for f in gnss.accepted],rejected=gnss.rejected)))

def restore_localizer(value):
    if not isinstance(value,dict) or value.get('schema')!=SCHEMA or value.get('libraries')!=dict(numpy=np.__version__,scipy=scipy.__version__):
        raise ValueError('incompatible_resume_schema')
    # Parsed input is bounded before this function; only allow finite JSON data.
    def finite_tree(v):
        if isinstance(v,dict):
            for child in v.values():finite_tree(child)
        elif isinstance(v,list):
            for child in v:finite_tree(child)
        elif type(v) is float and not math.isfinite(v):raise ValueError('nonfinite_resume_state')
    finite_tree(value)
    source=SourceCursorLocalizer();engine=source.engine;e=value['engine'];g=value['gnss']
    for key in SCALARS:setattr(engine,key,e[key])
    for key in ('accepted','rejected','motion_prior_count'):
        if type(e[key]) is not int or not 0<=e[key]<=1000000:raise ValueError('invalid_resume_scan_count')
    for key in ('last_seen','gyro_time','gyro_rate','last_geometry_stamp'):number(e[key],optional=True)
    for key in ('gyro_integral','gyro_valid_s','heading'):number(e[key])
    number(e['heading_time'],optional=True);whole(e['gyro_time_ns'],optional=True);whole(e['gyro_valid_ns'])
    whole(e['heading_epoch'],limit=1000000);whole(e['heading_disagreements'],limit=1000000)
    number(e['heading_gyro'],optional=True)
    engine.heading_q=None if e['heading_q'] is None else array(e['heading_q'],(4,))
    if engine.heading_q is not None and abs(np.linalg.norm(engine.heading_q)-1)>1e-5:raise ValueError('invalid_resume_quaternion')
    if (engine.heading_q is None)!=(e['heading_time'] is None):raise ValueError('invalid_resume_heading')
    if type(e['heading_samples']) is not list or len(e['heading_samples'])>8:raise ValueError('invalid_resume_heading_samples')
    engine.heading_samples=[(number(t),number(h)) for t,h in e['heading_samples']]
    if [t for t,_ in engine.heading_samples]!=sorted({t for t,_ in engine.heading_samples}):raise ValueError('invalid_resume_heading_samples')
    if type(e['path_break_pending']) is not bool or type(e['anchor_is_previous']) is not bool:raise ValueError('invalid_resume_path_flag')
    engine.pose=array(e['pose'],(3,));engine.velocity=None if e['velocity'] is None else array(e['velocity'],(2,))
    engine.direction=None if e['direction'] is None else array(e['direction'],(2,))
    engine.position_prior=None if e['position_prior'] is None else (number(e['position_prior'][0]),array(e['position_prior'][1],(2,)),number(e['position_prior'][2]))
    engine.gps_speed=None if e['gps_speed'] is None else tuple(number(v) for v in e['gps_speed'])
    if engine.gps_speed is not None and len(engine.gps_speed)!=2:raise ValueError('invalid_resume_speed')
    engine.last_recovery=-float('inf') if e['last_recovery'] is None else number(e['last_recovery'])
    engine.previous=restore_reference(e['previous'])
    engine.anchor=engine.previous if e['anchor_is_previous'] else restore_reference(e['anchor'])
    if len(value['imus'])>64 or len(value['pending_imus'])>10000:raise ValueError('invalid_resume_IMU_history')
    source.imus=value['imus'];source.stamps=value['stamps'];source.pending_imus=value['pending_imus']
    if source.stamps!=sorted(source.stamps) or source.stamps!=[source_stamp(m) for m in source.imus]:raise ValueError('invalid_resume_IMU_stamps')
    if type(value['input_scans']) is not int or value['input_scans']!=engine.accepted+engine.rejected:raise ValueError('invalid_resume_counters')
    source.input_scans=value['input_scans'];source.note=value['note'];source.basis=value['basis']
    if source.basis:
        source.basis={**source.basis,'rotation':array(source.basis['rotation'],(2,2)),
                      'translation':array(source.basis['translation'],(2,))}
        r=source.basis['rotation']
        if not np.allclose(r.T@r,np.eye(2),atol=1e-10) or not np.isclose(np.linalg.det(r),1,atol=1e-10):raise ValueError('invalid_resume_rigid_basis')
    if len(g['accepted'])>100000 or len(g['seen'])>100000:raise ValueError('invalid_resume_GNSS_history')
    if g['origin'] is not None and (len(g['origin'])!=2 or not -90<=number(g['origin'][0])<=90 or not -180<=number(g['origin'][1])<=180):raise ValueError('invalid_resume_GNSS_origin')
    source.gnss.origin=None if g['origin'] is None else FixedENU(*g['origin'])
    source.gnss.seen=set(g['seen']);source.gnss.accepted=[GNSSFactor(**{**f,'xy':tuple(f['xy'])}) for f in g['accepted']]
    source.gnss.rejected=g['rejected']
    if any(f.receiver_epoch not in source.gnss.seen for f in source.gnss.accepted):raise ValueError('invalid_resume_GNSS_epochs')
    if len(value['pairs'])>len(g['accepted']):raise ValueError('invalid_resume_GNSS_pairs')
    for p in value['pairs']:
        if type(p['factor']) is not int or not 0<=p['factor']<len(g['accepted']):raise ValueError('invalid_resume_factor_index')
    source.pairs=[(array(p['pose'],(2,)),source.gnss.accepted[p['factor']]) for p in value['pairs']]
    return source

def save_state(folder,history,state,grid,offset,t_s):
    if grid.scan_count==0:return
    # Verify state before installing any pointer, including cross-version limits.
    restore_localizer(state)
    token=f'resume-{state["input_scans"]:07d}-{uuid.uuid4().hex}'
    native=folder/(token+'-grid.json');proof=grid.save_checkpoint(native)
    blob=folder/(token+'.json.gz')
    data=json.dumps(state,separators=(',',':'),allow_nan=False).encode()
    if len(data)>LIMIT:raise ValueError('resume_state_size_limit')
    with blob.open('xb') as stream:
        stream.write(gzip.compress(data,compresslevel=3,mtime=0));stream.flush();os.fsync(stream.fileno())
    meta=dict(schema=SCHEMA,epoch=history['epoch'],source_sensor_sha256=history['source_sensor_sha256'],
        localization_fingerprint=localization_fingerprint(),raster_fingerprint=raster_fingerprint(),state=blob.name,state_sha256=digest(blob),
        native=native.name,native_sha256=proof['manifest_sha256'],raw_offset=offset,t_s=t_s,
        drawn_scans=grid.scan_count,input_scans=state['input_scans'],runtime_authority=False)
    previous={}
    pointer=folder/'resume.json'
    if pointer.is_file() and not pointer.is_symlink() and pointer.stat().st_size<=65536:
        try:previous=json.loads(pointer.read_text(encoding='utf-8'))
        except (OSError,ValueError):pass
    if not isinstance(previous,dict):previous={}
    if all(previous.get(k)==meta[k] for k in ('schema','epoch','source_sensor_sha256','localization_fingerprint','raster_fingerprint')):
        meta['previous']={k:v for k,v in previous.items() if k!='previous'}
    persist_json(pointer,meta)
    # Keep the new and previous immutable resume bundles. Derived display
    # history checkpoints, old map epochs, and original sensors are untouched.
    # Cleanup occurs only after the atomic pointer switch; failure is harmless.
    stale=previous.get('previous') if meta.get('previous') else None
    if isinstance(stale,dict):
        try:
            if not isinstance(stale.get('state'),str):return
            stem=stale['state'].removesuffix('.json.gz')
            if not re.fullmatch(r'resume-[0-9]{7}-[0-9a-f]{32}',stem) or stale.get('native')!=stem+'-grid.json':return
            native=folder/stale['native'];files=[folder/stale['state'],native]
            if native.is_file() and not native.is_symlink() and native.stat().st_size<=4*1024*1024:
                saved=json.loads(native.read_text(encoding='utf-8'));blob=saved.get('blob','')
                if re.fullmatch(re.escape(native.stem)+r'-[0-9a-f]{32}\.tiles\.gz',blob):files.append(folder/blob)
            for path in files:
                if path.resolve().parent==folder.resolve() and path.is_file() and not path.is_symlink():path.unlink()
        except (OSError,ValueError,TypeError):pass

def load_state(folder,history):
    pointer=folder/'resume.json'
    if not pointer.is_file():return None
    try:
        if pointer.is_symlink() or pointer.stat().st_size>65536:raise ValueError('invalid_resume_pointer')
        meta=json.loads(pointer.read_text(encoding='utf-8'))
        if not isinstance(meta,dict):return None
        current=load_entry(folder,history,meta)
        if current:return current
        return load_entry(folder,history,meta.get('previous',{}))
    except (OSError,ValueError,KeyError,TypeError,IndexError,EOFError,RecursionError,zlib.error):return None

def load_entry(folder,history,meta):
    try:
        if not isinstance(meta,dict):return None
        if (meta.get('schema')!=SCHEMA or meta.get('epoch')!=history['epoch'] or
            meta.get('source_sensor_sha256')!=history['source_sensor_sha256'] or
            meta.get('localization_fingerprint')!=history.get('localization_fingerprint') or
            meta.get('raster_fingerprint')!=history.get('raster_fingerprint') or
            meta.get('localization_fingerprint')!=localization_fingerprint() or
            meta.get('raster_fingerprint')!=raster_fingerprint() or
            not 0<=number(meta['t_s'])<=history['processed_t_s'] or
            type(meta['raw_offset']) is not int or not 0<=meta['raw_offset']<=16*1024**3 or
            not 1<=meta['drawn_scans']<=len(history['events'])):raise ValueError('incompatible_recorded_prefix')
        for key,suffix in (('state',r'\.json\.gz'),('native',r'-grid\.json')):
            if not re.fullmatch(r'resume-[0-9]{7}-[0-9a-f]{32}'+suffix,meta[key]):raise ValueError('invalid_resume_path')
            file=folder/meta[key]
            if file.is_symlink() or file.stat().st_size>LIMIT or digest(file)!=meta[key+'_sha256']:raise ValueError('resume_hash_mismatch')
        if meta['native']!=meta['state'].removesuffix('.json.gz')+'-grid.json':raise ValueError('resume_bundle_mismatch')
        with gzip.open(folder/meta['state'],'rb') as stream:data=stream.read(LIMIT+1)
        if len(data)>LIMIT:raise ValueError('resume_state_inflate_limit')
        state=json.loads(data);restore_localizer(state)
        grid=StreamingOccupancyMap.load_checkpoint(folder/meta['native'])
        if grid.scan_count!=meta['drawn_scans'] or state['input_scans']!=meta['input_scans']:raise ValueError('resume_native_count_mismatch')
        # Rebuild display caches from exact signed cells. Untouched old tiles
        # retain their occupied bounds and revision after resumed integration.
        for key,tile in grid._tiles.items():
            states=grid.cell_states(tile);grid.tile_classes[key]=states;grid.tile_versions[key]=grid.scan_count
            ys,xs=np.nonzero(states==2)
            if len(xs):grid.object_bounds[key]=(key[0]*64+int(xs.min()),key[1]*64+int(ys.min()),key[0]*64+int(xs.max())+1,key[1]*64+int(ys.max())+1)
        grid.scan_stamps[grid.scan_count]=grid.stamp
        return meta,state,grid
    except (OSError,ValueError,KeyError,TypeError,IndexError,EOFError,RecursionError,zlib.error):return None
