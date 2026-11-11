"""Deterministic 2D-scan orchard; measurements and assumptions are in rover.yaml."""
from pathlib import Path
import math
import xml.etree.ElementTree as E
import yaml
try:
    from .camera_profile import load_rover_config
except ImportError:
    from camera_profile import load_rover_config

ROOT = Path(__file__).resolve().parent

def add(parent, tag, value=None, **attrs):
    el = E.SubElement(parent, tag, attrs)
    if value is not None:
        el.text = str(value)
    return el

def pose(parent, xyz, rpy=(0, 0, 0)):
    add(parent, 'pose', ' '.join(str(v) for v in (*xyz, *rpy)))

def geometry(parent, shape, values):
    g = add(add(parent, 'geometry'), shape)
    if shape == 'box':
        add(g, 'size', ' '.join(map(str, values)))
    elif shape == 'sphere':
        add(g, 'radius', values[0])
    else:
        add(g, 'radius', values[0]); add(g, 'length', values[1])

def solid(link, shape, values, color, name='body', xyz=(0,0,0), rpy=(0,0,0), collision=True):
    vis = add(link, 'visual', name=name)
    pose(vis, xyz, rpy); geometry(vis, shape, values)
    mat = add(vis, 'material')
    add(mat, 'ambient', color); add(mat, 'diffuse', color)
    if collision:
        # SDFormat shares the link-child namespace across visuals and collisions.
        c = add(link, 'collision', name=name+'_collision')
        pose(c, xyz, rpy); geometry(c, shape, values)
        f = add(add(add(c, 'surface'), 'friction'), 'ode')
        add(f, 'mu', 0.8); add(f, 'mu2', 0.15)

def inertia(link, mass, dimensions):
    i = add(link, 'inertial'); add(i, 'mass', mass)
    t = add(i, 'inertia'); x,y,z = dimensions
    for key,val in [('ixx',mass*(y*y+z*z)/12),('iyy',mass*(x*x+z*z)/12),('izz',mass*(x*x+y*y)/12),('ixy',0),('ixz',0),('iyz',0)]:add(t,key,val)

def plugin(parent, short, **params):
    filename=''.join(('-'+c.lower() if c.isupper() else c) for c in short).lstrip('-')
    p=add(parent,'plugin',filename=f'gz-sim-{filename}-system',name=f'gz::sim::systems::{short}')
    for k,v in params.items():add(p,k,v)
    return p

def fixed(model, name, xyz):
    link=add(model,'link',name=name);pose(link,xyz);inertia(link,0.19,(.06,.06,.03))
    j=add(model,'joint',name=name+'_fixed',type='fixed');add(j,'parent','base_link');add(j,'child',name)
    return link

def sensor(link, name, kind, topic, hz):
    s=add(link,'sensor',name=name,type=kind);add(s,'always_on','true');add(s,'update_rate',hz);add(s,'topic',topic)
    return s

def generate(config):
    c=config;v=c['vehicle'];l=c['lidar'];cam=c['camera'];imu=c['imu']
    sdf=E.Element('sdf',version='1.9');w=add(sdf,'world',name='ubts_orchard')
    ph=add(w,'physics',name='1ms',type='ignored');add(ph,'max_step_size',.002);add(ph,'real_time_factor',1.0)
    add(w,'gravity','0 0 -9.80665')
    for p in ['Physics','UserCommands','SceneBroadcaster','Imu']:plugin(w,p)
    plugin(w,'Sensors',render_engine='ogre2')
    scene=add(w,'scene');add(scene,'ambient','.55 .55 .55 1');add(scene,'background','.55 .73 .90 1');add(scene,'shadows','false')
    sun=add(w,'light',name='sun',type='directional');pose(sun,(0,0,10));add(sun,'direction','-.5 .1 -1');add(sun,'diffuse','.9 .9 .9 1');add(sun,'cast_shadows','false')
    ground=add(w,'model',name='ground');add(ground,'static','true');gl=add(ground,'link',name='ground');pose(gl,(0,0,-.1));solid(gl,'box',(80,80,.2),'.29 .36 .17 1')
    # DART combines the two contacting surfaces with min(mu) and min(mu2).
    # The generic solid's mu2=.15 would cap tire rolling traction at .15 too,
    # erasing the wheel's lateral=.15 / longitudinal=.8 skid-steer profile.
    # Keep this flat test floor isotropic; coefficients remain SIM assumptions.
    gl.find('collision/surface/friction/ode/mu2').text = '0.8'
    # Tree trunks cross the 60cm scan. Canopies never supply semantic labels.
    for row,y in enumerate([-4.5,-1.5,1.5,4.5]):
        for col,x in enumerate(range(-4,21,3)):
            t=add(w,'model',name=f'tree_{row}_{col}');add(t,'static','true');pose(t,(x,y,0));lk=add(t,'link',name='trunk')
            solid(lk,'cylinder',(.16,1.8),'.32 .16 .06 1',xyz=(0,0,.9))
            solid(lk,'sphere',(.85,),'.13 .40 .08 1',name='canopy',xyz=(0,0,2.0))
    # Explicit obstacles for scan-plane tests: low object invisible at 0.6m.
    for name,xyz,size,color in [('barrier',(9,0,.6),(.3,1.1,1.2),'.8 .15 .08 1'),('low_obstacle',(3,-.6,.10),(.35,.35,.20),'.9 .7 .1 1'),('end_wall',(24,0,1),(.2,14,2),'.65 .65 .65 1')]:
        m=add(w,'model',name=name);add(m,'static','true');pose(m,xyz);lk=add(m,'link',name='body');solid(lk,'box',size,color)
    model=add(w,'model',name='ubts_rover');pose(model,(0,0,.003));add(model,'self_collide','false')
    base=add(model,'link',name='base_link');pose(base,(0,0,v['body_center_height_m']));inertia(base,v['body_mass_kg'],v['body_size_m']);solid(base,'box',v['body_size_m'],'.14 .25 .40 1')
    # Low door sides indicate the cut passenger body; no guessed roof/cab.
    for side in [-1,1]:solid(base,'box',(.8,.03,.12),'.20 .35 .5 1',name=f'door_{side}',xyz=(0,side*.27,.18))
    mast_bottom=v['body_center_height_m']+v['body_size_m'][2]/2
    mast_height=l['height_above_ground_m']-mast_bottom
    solid(base,'cylinder',(.012,mast_height),'.4 .4 .45 1',name='sensor_mast',
          xyz=(l['x_m'],l['y_m'],mast_bottom+mast_height/2-v['body_center_height_m']))
    wheel_names={'left':[],'right':[]}
    for side,sy in [('left',1),('right',-1)]:
        for axle,sx in [('front',1),('rear',-1)]:
            name=f'{side}_{axle}_wheel';lk=add(model,'link',name=name);pose(lk,(sx*v['wheelbase_m']/2,sy*v['track_m']/2,v['wheel_radius_m']))
            inertia(lk,v['wheel_mass_kg'],(.28,.12,.28));solid(lk,'cylinder',(v['wheel_radius_m'],v['wheel_width_m']),'.045 .05 .065 1',rpy=(math.pi/2,0,0))
            # Collision-local Z is the axle after Rx(pi/2), so wheel spin leaves
            # it invariant. DART's primary friction is lateral; the orthogonal
            # coefficient is longitudinal. Values remain simulation assumptions.
            friction=lk.find('collision/surface/friction/ode')
            friction.find('mu').text='0.15';friction.find('mu2').text='0.8'
            add(friction,'fdir1','0 0 1')
            j=add(model,'joint',name=name+'_joint',type='revolute');add(j,'parent','base_link');add(j,'child',name);axis=add(j,'axis');add(axis,'xyz','0 1 0');limit=add(axis,'limit');add(limit,'lower',-1e16);add(limit,'upper',1e16)
            wheel_names[side].append(name+'_joint')
    laser=fixed(model,'laser',(l['x_m'],l['y_m'],l['height_above_ground_m']))
    solid(laser,'cylinder',(.0385,.0385),'.12 .12 .14 1',collision=False)
    scan=sensor(laser,'s2','gpu_lidar','/sim/scan',l['hz']);ld=add(scan,'lidar');scan_cfg=add(ld,'scan')
    hor=add(scan_cfg,'horizontal');add(hor,'samples',l['samples']);add(hor,'resolution',1);add(hor,'min_angle',-math.pi);add(hor,'max_angle',math.pi-2*math.pi/l['samples'])
    vert=add(scan_cfg,'vertical');add(vert,'samples',1);add(vert,'resolution',1);add(vert,'min_angle',0);add(vert,'max_angle',0)
    rang=add(ld,'range');add(rang,'min',l['min_m']);add(rang,'max',l['max_m']);add(rang,'resolution',.013)
    noise=add(ld,'noise');add(noise,'type','gaussian');add(noise,'mean',0);add(noise,'stddev',l['noise_stddev_m'])
    il=fixed(model,'imu_link',(l['x_m'],l['y_m'],imu['height_above_ground_m']));sensor(il,'bno085','imu','/sim/imu',imu['hz'])
    cl=fixed(model,'camera_link',cam['xyz_m']);solid(cl,'box',(.025,.06,.025),'.05 .05 .05 1',collision=False)
    cs=sensor(cl,'uvc','camera','/sim/camera',cam['hz']);cc=add(cs,'camera');add(cc,'optical_frame_id','camera_link_optical');add(cc,'horizontal_fov',math.radians(cam['horizontal_fov_deg']));im=add(cc,'image');add(im,'width',cam['width']);add(im,'height',cam['height']);add(im,'format','R8G8B8');clip=add(cc,'clip');add(clip,'near',.05);add(clip,'far',40)
    dd=plugin(model,'DiffDrive',wheel_separation=v['track_m'],wheel_radius=v['wheel_radius_m'],topic='/sim/cmd_vel',odom_topic='/sim/wheel_odom',tf_topic='/sim/wheel_tf',odom_publish_frequency=30,frame_id='odom',child_frame_id='base_footprint')
    for side,joints in wheel_names.items():
        for joint in joints:add(dd,side+'_joint',joint)
    # Position evaluation uses independent ground truth, never localization input.
    # Publisher reports the model origin at ground level, not the elevated body link.
    plugin(model,'OdometryPublisher',odom_topic='/sim/ground_truth',odom_frame='world',robot_base_frame='base_footprint',odom_publish_frequency=30,dimensions=3)
    # Native telemetry cap only; wheel dynamics and encoder odometry are
    # independent. Gz 8.15 truncates the 120 Hz step to 8,333,333 ns, so this
    # 30 Hz ceiling should publish every 5 steps (~24 Hz), not exactly 30 Hz.
    plugin(model,'JointStatePublisher',topic='/sim/joint_states',update_rate=30)
    E.indent(sdf)
    return E.tostring(sdf,encoding='unicode')

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--output',default='/runtime/orchard.sdf');args=p.parse_args()
    Path(args.output).write_text(generate(load_rover_config(ROOT/'rover.yaml')),encoding='utf-8')
