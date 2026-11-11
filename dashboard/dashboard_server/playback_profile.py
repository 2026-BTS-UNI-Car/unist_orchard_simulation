"""Loss-bounded display transport; original recording points stay unchanged."""
import math

POLAR_PROFILE='dashboard-polar-v2'
FULL_FIELDS=('x_m','y_m','distance_m','quality','angle_deg')
POLAR_FIELDS=('angle_deg','distance_m','quality')


def point_fields(frames,profile):
    if profile!=POLAR_PROFILE:return FULL_FIELDS
    seen=False
    for frame in frames:
        for p in frame.get('points') or []:
            if not isinstance(p,dict):return FULL_FIELDS
            try:
                a=math.radians(p['angle_deg']);r=p['distance_m'];x=p['x_m'];y=p['y_m']
                if not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in (a,r,x,y)):return FULL_FIELDS
                # The bridge's display convention is x=sin(a)*r,y=cos(a)*r.
                # Some imported archives use another frame; keep explicit XY
                # for those, rather than silently rotating/reprojecting them.
                if abs(x-math.sin(a)*r)>.002 or abs(y-math.cos(a)*r)>.002:return FULL_FIELDS
                seen=True
            except (KeyError,TypeError,ValueError):return FULL_FIELDS
    return POLAR_FIELDS if seen else FULL_FIELDS
