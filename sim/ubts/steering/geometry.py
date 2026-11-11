"""Front-wheel steering geometry for the steering SIM (design 3.1, 3.2 and 7.1).

Frames and signs (design 3.1): body x forward, y left, z up. Angles are in rad unless a
name ends in _deg; positive is to the left (counter-clockwise seen from above). "base"
is the footprint centre (Gazebo base_footprint). The bicycle equivalent angle delta is
the one angle that gives the rear-axle curvature kappa = tan(delta) / L.

Steering types (profile steer_type):
  linkage     each front wheel turns about its own kingpin; kingpin width w_k. The
              Ackermann ratio a in [0, 1] blends parallel (a = 0) and ideal Ackermann
              (a = 1): delta_i = delta + a * (delta_i_ideal - delta).
  axle_pivot  the whole front axle turns about its centre, so both wheels have the angle
              delta. Without slip the rear-axle curvature is still tan(delta) / L, with L
              the distance from the rear axle to the pivot; the wheel positions move.

Limits are defined per wheel and per direction (4 values, WHEEL_LIMIT_KEYS). The
equivalent limit in one direction is the largest |delta| that keeps both wheels inside
their limits (DERIVED, design 3.2).

Bracket margin (steer_hardware_margin_m, design 3.2 "sweep + margin" and 7.1 "the band stays
for small steering"): unknown parts next to a front wheel add to the part of the wheel that
sticks out of the body outline. The margin grows the wheel outline only outward (away from
the centre line) by min(margin, sweep) where sweep is the bare wheel's reach beyond the body
half width: 0 while the wheel is inside the body (straight ahead the footprint is the body),
the full margin once the wheel sticks out by at least the margin (28 deg: 0.043 + 0.02 m).
The ramp keeps the footprint continuous in the steering angle.

Pure Python (math only). No numpy. This is SIM geometry from placeholder values.
"""
from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
import math
from typing import Dict, List, Mapping, Tuple

LINKAGE = 'linkage'
AXLE_PIVOT = 'axle_pivot'
STEER_TYPES = (LINKAGE, AXLE_PIVOT)
LEFT = 'left'
RIGHT = 'right'
SIDES = (LEFT, RIGHT)
WHEEL_LIMIT_KEYS = ('left_wheel_toward_left', 'left_wheel_toward_right',
                    'right_wheel_toward_left', 'right_wheel_toward_right')
# Profile names read by SteeringGeometry.from_profile().
PROFILE_NAMES = ('wheelbase_m', 'track_front_m', 'track_rear_m', 'kingpin_width_m', 'wheel_radius_front_m',
                 'wheel_radius_rear_m', 'wheel_width_m', 'rear_axle_x_in_base_m', 'footprint_length_m',
                 'footprint_width_m', 'front_bumper_x_in_base_m', 'steer_type', 'steer_ackermann_ratio',
                 'steer_mech_limit_deg', 'steer_soft_margin_deg', 'steer_hardware_margin_m',
                 'kingpin_inclination_deg', 'steer_link_table')
# Search interval when the wheel-angle map is inverted (rad).
_SEARCH_LIMIT_RAD = math.radians(80.)

Point = Tuple[float, float]


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('geometry_value_not_finite:' + name)
    return float(value)


def _positive(value, name):
    value = _number(value, name)
    if value <= 0:
        raise ValueError('geometry_value_not_positive:' + name)
    return value


def _angle(value, name='delta_rad'):
    value = _number(value, name)
    if abs(value) >= math.pi / 2:
        raise ValueError('geometry_angle_out_of_range:' + name)
    return value


def _ratio(value):
    value = _number(value, 'ackermann_ratio')
    if not 0. <= value <= 1.:
        raise ValueError('geometry_ackermann_ratio_out_of_range')
    return value


def _steer_type(value):
    if value not in STEER_TYPES:
        raise ValueError('geometry_unknown_steer_type:%r' % (value,))
    return value


def _side(side):
    if side not in SIDES:
        raise ValueError('geometry_unknown_side:%r' % (side,))
    return side


# --- bicycle relations -------------------------------------------------------------------------------
def curvature(delta_rad, wheelbase_m):
    """Rear-axle curvature kappa = tan(delta) / L (1/m, positive to the left)."""
    return math.tan(_angle(delta_rad)) / _positive(wheelbase_m, 'wheelbase_m')


def yaw_rate(speed_m_s, delta_rad, wheelbase_m):
    """Kinematic yaw rate omega = v * tan(delta) / L (rad/s)."""
    return _number(speed_m_s, 'speed_m_s') * curvature(delta_rad, wheelbase_m)


def turn_radius(delta_rad, wheelbase_m):
    """Signed rear-axle turn radius R = L / tan(delta); +inf when delta is 0."""
    kappa = curvature(delta_rad, wheelbase_m)
    return math.inf if kappa == 0. else 1. / kappa


def delta_for_curvature(kappa_1_m, wheelbase_m):
    """Inverse of curvature(): delta = atan(kappa * L)."""
    return math.atan(_number(kappa_1_m, 'kappa_1_m') * _positive(wheelbase_m, 'wheelbase_m'))


# --- equivalent angle <-> wheel angles -----------------------------------------------------------------
def ideal_ackermann_angles(delta_rad, wheelbase_m, kingpin_width_m):
    """Ideal Ackermann (left, right) wheel angles for the equivalent angle delta.

    tan(delta_left) = tan(delta) / (1 - k tan(delta)), tan(delta_right) = tan(delta) / (1 + k tan(delta)),
    k = w_k / (2 L); i.e. delta_in = atan(L / (R - w_k/2)), delta_out = atan(L / (R + w_k/2)).
    """
    delta = _angle(delta_rad)
    k = _positive(kingpin_width_m, 'kingpin_width_m') / (2. * _positive(wheelbase_m, 'wheelbase_m'))
    s, c = math.sin(delta), math.cos(delta)
    return math.atan2(s, c - k * s), math.atan2(s, c + k * s)


def linkage_wheel_angles(delta_rad, wheelbase_m, kingpin_width_m, ackermann_ratio):
    """(left, right) wheel angles of the linkage type: delta_i = delta + a (delta_i_ideal - delta)."""
    a = _ratio(ackermann_ratio)
    delta = _angle(delta_rad)
    ideal_left, ideal_right = ideal_ackermann_angles(delta, wheelbase_m, kingpin_width_m)
    return delta + a * (ideal_left - delta), delta + a * (ideal_right - delta)


def wheel_angles(delta_rad, steer_type, wheelbase_m, kingpin_width_m, ackermann_ratio):
    """(left, right) wheel angles for the equivalent angle delta and the steering type."""
    if _steer_type(steer_type) == AXLE_PIVOT:
        delta = _angle(delta_rad)
        return delta, delta
    return linkage_wheel_angles(delta_rad, wheelbase_m, kingpin_width_m, ackermann_ratio)


def equivalent_from_wheel(wheel_angle_rad, side, steer_type, wheelbase_m, kingpin_width_m, ackermann_ratio):
    """Equivalent angle delta whose `side` wheel has the angle wheel_angle (bisection; the map is monotonic)."""
    _side(side)
    target = _angle(wheel_angle_rad, 'wheel_angle_rad')
    if _steer_type(steer_type) == AXLE_PIVOT or target == 0.:
        return target
    index = 0 if side == LEFT else 1

    def wheel(delta):
        return linkage_wheel_angles(delta, wheelbase_m, kingpin_width_m, ackermann_ratio)[index]

    lo, hi = -_SEARCH_LIMIT_RAD, _SEARCH_LIMIT_RAD
    if not wheel(lo) <= target <= wheel(hi):
        raise ValueError('geometry_wheel_angle_outside_search_range')
    for _ in range(200):
        mid = .5 * (lo + hi)
        if mid <= lo or mid >= hi:
            break
        if wheel(mid) < target:
            lo = mid
        else:
            hi = mid
    return .5 * (lo + hi)


def _wheel_limits(limits_rad):
    if not isinstance(limits_rad, Mapping) or set(limits_rad) != set(WHEEL_LIMIT_KEYS):
        raise ValueError('geometry_wheel_limits_need_keys:' + ','.join(WHEEL_LIMIT_KEYS))
    return {key: _positive(limits_rad[key], key) for key in WHEEL_LIMIT_KEYS}


def equivalent_limits(wheel_limits_rad, margin_rad, steer_type, wheelbase_m, kingpin_width_m, ackermann_ratio):
    """Largest equivalent angle per direction keeping both wheels inside (wheel limit - margin).

    wheel_limits_rad: mapping with the 4 WHEEL_LIMIT_KEYS (positive magnitudes, rad).
    Returns (left_limit, right_limit) as positive magnitudes in rad. With margin 0 this is
    the equivalent mechanical limit; with steer_soft_margin it is the equivalent soft limit.
    """
    limits = _wheel_limits(wheel_limits_rad)
    margin = _number(margin_rad, 'margin_rad')
    if margin < 0:
        raise ValueError('geometry_margin_negative')
    usable = {key: value - margin for key, value in limits.items()}
    if any(value <= 0. for value in usable.values()):
        raise ValueError('geometry_wheel_limit_not_above_margin')
    args = (steer_type, wheelbase_m, kingpin_width_m, ackermann_ratio)
    left = min(equivalent_from_wheel(usable['left_wheel_toward_left'], LEFT, *args),
               equivalent_from_wheel(usable['right_wheel_toward_left'], RIGHT, *args))
    right = min(-equivalent_from_wheel(-usable['left_wheel_toward_right'], LEFT, *args),
                -equivalent_from_wheel(-usable['right_wheel_toward_right'], RIGHT, *args))
    return left, right


def scrub_radius(track_front_m, kingpin_width_m, kingpin_inclination_rad, wheel_radius_front_m):
    """Scrub radius at the ground: (T - w_k) / 2 - r tan(kingpin inclination).

    The kingpin width is taken at wheel-centre height; with inclination the steering axis
    meets the ground further out by r tan(KPI). Meaningful for the linkage type only.
    """
    return ((_positive(track_front_m, 'track_front_m') - _positive(kingpin_width_m, 'kingpin_width_m')) / 2.
            - _positive(wheel_radius_front_m, 'wheel_radius_front_m') * math.tan(
                _number(kingpin_inclination_rad, 'kingpin_inclination_rad')))


# --- front-wheel corners and sweep -----------------------------------------------------------------------
def _rotate(x, y, angle):
    c, s = math.cos(angle), math.sin(angle)
    return x * c - y * s, x * s + y * c


def front_wheel_centre(side, wheel_angle_rad, *, steer_type, front_axle_x_m, track_front_m, kingpin_width_m):
    """Front wheel centre (x, y) in base for the wheel angle (planar; caster and inclination ignored)."""
    sign = 1. if _side(side) == LEFT else -1.
    angle = _angle(wheel_angle_rad, 'wheel_angle_rad')
    axle_x = _number(front_axle_x_m, 'front_axle_x_m')
    track = _positive(track_front_m, 'track_front_m')
    if _steer_type(steer_type) == AXLE_PIVOT:
        ox, oy = _rotate(0., sign * track / 2., angle)
        return axle_x + ox, oy
    kingpin = _positive(kingpin_width_m, 'kingpin_width_m')
    ox, oy = _rotate(0., sign * (track - kingpin) / 2., angle)
    return axle_x + ox, sign * kingpin / 2. + oy


def front_wheel_corners(side, wheel_angle_rad, *, steer_type, front_axle_x_m, track_front_m, kingpin_width_m,
                        wheel_radius_m, wheel_width_m, margin_m=0.):
    """Corners (counter-clockwise) of the top view of a steered front wheel.

    The wheel is a rectangle 2 r long and w wide (4 corners). margin_m > 0 extends it outward
    (away from the centre line, +y for the left wheel, -y for the right) by margin_m: the hull of
    the rectangle and its outward shift (6 corners). The outward reach grows by exactly margin_m;
    the front, rear and inner edges do not move. SteeringGeometry passes the effective bracket
    margin (effective_hardware_margin) here.
    """
    cx, cy = front_wheel_centre(side, wheel_angle_rad, steer_type=steer_type, front_axle_x_m=front_axle_x_m,
                                track_front_m=track_front_m, kingpin_width_m=kingpin_width_m)
    margin = _number(margin_m, 'margin_m')
    if margin < 0:
        raise ValueError('geometry_margin_negative')
    half_length = _positive(wheel_radius_m, 'wheel_radius_m')
    half_width = _positive(wheel_width_m, 'wheel_width_m') / 2.
    corners = []
    for px, py in ((half_length, half_width), (-half_length, half_width), (-half_length, -half_width),
                   (half_length, -half_width)):
        rx, ry = _rotate(px, py, wheel_angle_rad)
        corners.append((cx + rx, cy + ry))
    if margin == 0.:
        return corners
    shift = margin if side == LEFT else -margin
    return convex_hull(corners + [(x, y + shift) for x, y in corners])


def wheel_outer_extent(side, wheel_angle_rad, **wheel):
    """Outward lateral reach of a front wheel from the body centre line (m, positive outward)."""
    corners = front_wheel_corners(side, wheel_angle_rad, **wheel)
    return max(y for _, y in corners) if side == LEFT else max(-y for _, y in corners)


def effective_hardware_margin(bare_sweep_m, margin_m):
    """Bracket margin that applies to a wheel whose bare reach beyond the body is bare_sweep_m: min(margin, sweep),
    0 while the wheel is inside the body outline (see the module docstring)."""
    margin = _number(margin_m, 'margin_m')
    if margin < 0:
        raise ValueError('geometry_margin_negative')
    return min(margin, max(0., _number(bare_sweep_m, 'bare_sweep_m')))


def wheel_sweep_beyond_body(side, wheel_angle_rad, *, body_half_width_m, margin_m=0., **wheel):
    """Outward reach beyond the body half width (m). Negative: the wheel is inside the body outline.

    margin_m is the bracket margin (steer_hardware_margin_m); it adds effective_hardware_margin(bare sweep, margin_m).
    """
    bare = wheel_outer_extent(side, wheel_angle_rad, **wheel) - _positive(body_half_width_m, 'body_half_width_m')
    return bare + effective_hardware_margin(bare, margin_m)


def sweep_at_limits(wheel_limits_rad, *, body_half_width_m, steer_type, track_front_m, kingpin_width_m,
                    wheel_radius_m, wheel_width_m, margin_m=0.):
    """Sweep beyond the body half width with each wheel at each of its 4 limits (m, per WHEEL_LIMIT_KEYS)."""
    limits = _wheel_limits(wheel_limits_rad)
    wheel = dict(steer_type=steer_type, front_axle_x_m=0., track_front_m=track_front_m,
                 kingpin_width_m=kingpin_width_m, wheel_radius_m=wheel_radius_m, wheel_width_m=wheel_width_m,
                 margin_m=margin_m)
    out = {}
    for key in WHEEL_LIMIT_KEYS:
        side = LEFT if key.startswith('left_') else RIGHT
        angle = limits[key] if key.endswith('toward_left') else -limits[key]
        out[key] = wheel_sweep_beyond_body(side, angle, body_half_width_m=body_half_width_m, **wheel)
    return out


# --- polygons ----------------------------------------------------------------------------------------------
def rectangle(x_min, x_max, y_min, y_max):
    """Axis-aligned rectangle corners, counter-clockwise."""
    return [(x_max, y_max), (x_min, y_max), (x_min, y_min), (x_max, y_min)]


def convex_hull(points):
    """Counter-clockwise convex hull (monotone chain); collinear points dropped."""
    pts = sorted(set((float(x), float(y)) for x, y in points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def point_in_convex_polygon(point, polygon, tolerance_m=1e-9):
    """True if point is inside or on a counter-clockwise convex polygon (within tolerance)."""
    px, py = point
    n = len(polygon)
    if n < 3:
        return False
    for i in range(n):
        ax, ay = polygon[i]
        bx, by = polygon[(i + 1) % n]
        edge = math.hypot(bx - ax, by - ay)
        if edge == 0.:
            continue
        if ((bx - ax) * (py - ay) - (by - ay) * (px - ax)) / edge < -tolerance_m:
            return False
    return True


def polygon_support(points, direction):
    """max over the points of the projection on the unit direction (support function)."""
    ux, uy = direction
    norm = math.hypot(ux, uy)
    if norm == 0.:
        raise ValueError('geometry_zero_direction')
    return max((x * ux + y * uy) / norm for x, y in points)


# --- potentiometer-side angle <-> wheel angle table ---------------------------------------------------------
def _interp(xs, ys, x):
    i = bisect_right(xs, x) - 1
    i = min(max(i, 0), len(xs) - 2)
    x0, x1, y0, y1 = xs[i], xs[i + 1], ys[i], ys[i + 1]
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0)


class LinkTable:
    """Static map between the potentiometer-side angle and the equivalent wheel angle.

    Rows are [pot_side_deg, wheel_eq_deg] (profile steer_link_table), strictly increasing
    in both columns, at least 2 rows. The pot-side angle is the pot shaft angle divided by
    pot_to_steer_ratio, i.e. on the steering-angle scale. Backlash is not in this table.
    Linear interpolation; linear extrapolation beyond the end rows. Default: identity.
    """

    def __init__(self, rows_deg):
        rows = []
        for row in rows_deg:
            if len(row) != 2:
                raise ValueError('link_table_row_needs_2_values')
            rows.append((_number(row[0], 'link_table'), _number(row[1], 'link_table')))
        if len(rows) < 2:
            raise ValueError('link_table_needs_2_rows')
        for a, b in zip(rows, rows[1:]):
            if b[0] <= a[0] or b[1] <= a[1]:
                raise ValueError('link_table_not_strictly_increasing')
        self.rows_deg = tuple(rows)
        self._pot = [math.radians(x) for x, _ in rows]
        self._wheel = [math.radians(y) for _, y in rows]

    @classmethod
    def identity(cls, span_deg=40.):
        return cls([(-span_deg, -span_deg), (span_deg, span_deg)])

    def is_identity(self):
        return all(x == y for x, y in self.rows_deg)

    def wheel_from_pot(self, pot_angle_rad):
        return _interp(self._pot, self._wheel, _number(pot_angle_rad, 'pot_angle_rad'))

    def pot_from_wheel(self, wheel_angle_rad):
        return _interp(self._wheel, self._pot, _number(wheel_angle_rad, 'wheel_angle_rad'))

    def rows_rad(self):
        return [[x, y] for x, y in zip(self._pot, self._wheel)]


# --- geometry object ----------------------------------------------------------------------------------------
@dataclass(frozen=True)
class SteeringGeometry:
    """Steering geometry of one vehicle profile. Lengths in m, angles in rad."""

    wheelbase_m: float
    track_front_m: float
    track_rear_m: float
    kingpin_width_m: float
    wheel_radius_front_m: float
    wheel_radius_rear_m: float
    wheel_width_m: float
    rear_axle_x_in_base_m: float
    footprint_length_m: float
    footprint_width_m: float
    front_bumper_x_in_base_m: float
    steer_type: str
    ackermann_ratio: float
    wheel_limits_rad: Tuple[float, float, float, float]
    soft_margin_rad: float
    hardware_margin_m: float
    kingpin_inclination_rad: float = 0.
    link_table_rows_deg: Tuple[Tuple[float, float], ...] = ((-40., -40.), (40., 40.))

    def __post_init__(self):
        for name in ('wheelbase_m', 'track_front_m', 'track_rear_m', 'kingpin_width_m', 'wheel_radius_front_m',
                     'wheel_radius_rear_m', 'wheel_width_m', 'footprint_length_m', 'footprint_width_m'):
            _positive(getattr(self, name), name)
        for name in ('rear_axle_x_in_base_m', 'front_bumper_x_in_base_m', 'kingpin_inclination_rad'):
            _number(getattr(self, name), name)
        _steer_type(self.steer_type)
        _ratio(self.ackermann_ratio)
        if len(self.wheel_limits_rad) != 4:
            raise ValueError('geometry_wheel_limits_need_4_values')
        if _number(self.soft_margin_rad, 'soft_margin_rad') < 0 or _number(self.hardware_margin_m,
                                                                           'hardware_margin_m') < 0:
            raise ValueError('geometry_margin_negative')
        object.__setattr__(self, '_link', LinkTable(self.link_table_rows_deg))
        self.soft_limits_eq()  # validates limits > margin

    @classmethod
    def from_profile(cls, profile):
        """Build from a vehicle_profile.VehicleProfile (reads PROFILE_NAMES)."""
        v = profile.value
        limits = v('steer_mech_limit_deg')
        return cls(
            wheelbase_m=v('wheelbase_m'), track_front_m=v('track_front_m'), track_rear_m=v('track_rear_m'),
            kingpin_width_m=v('kingpin_width_m'), wheel_radius_front_m=v('wheel_radius_front_m'),
            wheel_radius_rear_m=v('wheel_radius_rear_m'), wheel_width_m=v('wheel_width_m'),
            rear_axle_x_in_base_m=v('rear_axle_x_in_base_m'), footprint_length_m=v('footprint_length_m'),
            footprint_width_m=v('footprint_width_m'), front_bumper_x_in_base_m=v('front_bumper_x_in_base_m'),
            steer_type=v('steer_type'), ackermann_ratio=v('steer_ackermann_ratio'),
            wheel_limits_rad=tuple(math.radians(limits[key]) for key in WHEEL_LIMIT_KEYS),
            soft_margin_rad=math.radians(v('steer_soft_margin_deg')),
            hardware_margin_m=v('steer_hardware_margin_m'),
            kingpin_inclination_rad=math.radians(v('kingpin_inclination_deg')),
            link_table_rows_deg=tuple((float(x), float(y)) for x, y in v('steer_link_table')))

    # -- basic relations
    @property
    def front_axle_x_m(self):
        return self.rear_axle_x_in_base_m + self.wheelbase_m

    @property
    def link_table(self):
        return self._link

    def wheel_limits(self) -> Dict[str, float]:
        return dict(zip(WHEEL_LIMIT_KEYS, self.wheel_limits_rad))

    def soft_wheel_limits(self) -> Dict[str, float]:
        return {key: value - self.soft_margin_rad for key, value in self.wheel_limits().items()}

    def _args(self):
        return self.steer_type, self.wheelbase_m, self.kingpin_width_m, self.ackermann_ratio

    def curvature(self, delta_rad):
        return curvature(delta_rad, self.wheelbase_m)

    def yaw_rate(self, speed_m_s, delta_rad):
        return yaw_rate(speed_m_s, delta_rad, self.wheelbase_m)

    def turn_radius(self, delta_rad):
        return turn_radius(delta_rad, self.wheelbase_m)

    def wheel_angles(self, delta_rad):
        """(left, right) wheel angles for the equivalent angle."""
        return wheel_angles(delta_rad, *self._args())

    def equivalent_from_wheel(self, wheel_angle_rad, side):
        return equivalent_from_wheel(wheel_angle_rad, side, *self._args())

    def equivalent_from_wheels(self, left_rad, right_rad):
        """Mean of the two single-wheel inversions (exact when the pair is consistent)."""
        return .5 * (self.equivalent_from_wheel(left_rad, LEFT) + self.equivalent_from_wheel(right_rad, RIGHT))

    def mech_limits_eq(self):
        """(left, right) equivalent mechanical limits, positive magnitudes (rad)."""
        return equivalent_limits(self.wheel_limits(), 0., *self._args())

    def soft_limits_eq(self):
        """(left, right) equivalent soft limits, positive magnitudes (rad); the host API limit."""
        return equivalent_limits(self.wheel_limits(), self.soft_margin_rad, *self._args())

    def clamp_to_soft(self, delta_rad):
        left, right = self.soft_limits_eq()
        return min(left, max(-right, _number(delta_rad, 'delta_rad')))

    def scrub_radius_m(self):
        return scrub_radius(self.track_front_m, self.kingpin_width_m, self.kingpin_inclination_rad,
                            self.wheel_radius_front_m)

    # -- wheels, sweep and footprint
    def _wheel_kwargs(self):
        return dict(steer_type=self.steer_type, front_axle_x_m=self.front_axle_x_m, track_front_m=self.track_front_m,
                    kingpin_width_m=self.kingpin_width_m, wheel_radius_m=self.wheel_radius_front_m,
                    wheel_width_m=self.wheel_width_m)

    def front_wheel_centre(self, side, wheel_angle_rad):
        return front_wheel_centre(side, wheel_angle_rad, steer_type=self.steer_type,
                                  front_axle_x_m=self.front_axle_x_m, track_front_m=self.track_front_m,
                                  kingpin_width_m=self.kingpin_width_m)

    def hardware_margin_at(self, side, wheel_angle_rad):
        """Effective bracket margin of the `side` wheel at this wheel angle (0 while it is inside the body)."""
        bare = self.wheel_sweep_beyond_body(side, wheel_angle_rad, include_hardware_margin=False)
        return effective_hardware_margin(bare, self.hardware_margin_m)

    def front_wheel_corners(self, side, wheel_angle_rad, include_hardware_margin=False):
        margin = self.hardware_margin_at(side, wheel_angle_rad) if include_hardware_margin else 0.
        return front_wheel_corners(side, wheel_angle_rad, margin_m=margin, **self._wheel_kwargs())

    def wheel_sweep_beyond_body(self, side, wheel_angle_rad, include_hardware_margin=False):
        return wheel_sweep_beyond_body(side, wheel_angle_rad, body_half_width_m=self.footprint_width_m / 2.,
                                       margin_m=self.hardware_margin_m if include_hardware_margin else 0.,
                                       **self._wheel_kwargs())

    def sweep_at_mech_limits(self, include_hardware_margin=False):
        return sweep_at_limits(self.wheel_limits(), body_half_width_m=self.footprint_width_m / 2.,
                               steer_type=self.steer_type, track_front_m=self.track_front_m,
                               kingpin_width_m=self.kingpin_width_m, wheel_radius_m=self.wheel_radius_front_m,
                               wheel_width_m=self.wheel_width_m,
                               margin_m=self.hardware_margin_m if include_hardware_margin else 0.)

    def body_polygon(self):
        """Body outline: x from (bumper - length) to bumper, y +-width/2 (counter-clockwise)."""
        front = self.front_bumper_x_in_base_m
        half = self.footprint_width_m / 2.
        return rectangle(front - self.footprint_length_m, front, -half, half)

    def rear_wheel_corners(self, side):
        sign = 1. if _side(side) == LEFT else -1.
        y = sign * self.track_rear_m / 2.
        x = self.rear_axle_x_in_base_m
        return rectangle(x - self.wheel_radius_rear_m, x + self.wheel_radius_rear_m,
                         y - self.wheel_width_m / 2., y + self.wheel_width_m / 2.)

    def footprint_polygons(self, delta_left_rad, delta_right_rad, include_hardware_margin=True) -> Dict[str, List[Point]]:
        """Body, steered front wheels (current wheel angles) and rear wheels as polygons in base."""
        return dict(body=self.body_polygon(),
                    front_left=self.front_wheel_corners(LEFT, delta_left_rad, include_hardware_margin),
                    front_right=self.front_wheel_corners(RIGHT, delta_right_rad, include_hardware_margin),
                    rear_left=self.rear_wheel_corners(LEFT), rear_right=self.rear_wheel_corners(RIGHT))

    def footprint_hull(self, delta_left_rad, delta_right_rad, include_hardware_margin=True) -> List[Point]:
        """Convex hull of footprint_polygons() (counter-clockwise); contains the body."""
        points = []
        for polygon in self.footprint_polygons(delta_left_rad, delta_right_rad, include_hardware_margin).values():
            points.extend(polygon)
        return convex_hull(points)

    def footprint_lateral_extent(self, delta_left_rad, delta_right_rad, include_hardware_margin=True):
        """(left reach, right reach) of the footprint from the body centre line (m, positive outward)."""
        points = self.footprint_hull(delta_left_rad, delta_right_rad, include_hardware_margin)
        return polygon_support(points, (0., 1.)), polygon_support(points, (0., -1.))

    def footprint_for_equivalent(self, delta_rad, include_hardware_margin=True):
        left, right = self.wheel_angles(delta_rad)
        return self.footprint_hull(left, right, include_hardware_margin)
