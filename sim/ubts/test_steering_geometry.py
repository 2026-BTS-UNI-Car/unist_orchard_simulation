"""Steering geometry (design 3.2): bicycle relations, Ackermann blend, per-wheel limits -> equivalent
limits, axle_pivot, front-wheel sweep, footprint polygon and the potentiometer-side table.
Numbers checked against the design text: equivalent soft limit 25 / 23.5 / 22.2 deg for a = 0 / 0.5 / 1
(L 0.76 m, w_k 0.46 m, wheel limits 28 deg, margin 3 deg) and sweep 0.043 / 0.039 / 0.031 m at
28 / 25 / 20 deg (placeholder geometry, SIM values only).
"""
from dataclasses import replace
import math

import pytest

from steering import geometry as g
from steering import vehicle_profile as vp

L, WK = .76, .46
D = math.radians
LIMITS_28 = {key: D(28.) for key in g.WHEEL_LIMIT_KEYS}
WHEEL = dict(steer_type='linkage', front_axle_x_m=.38, track_front_m=.56, kingpin_width_m=.46, wheel_radius_m=.14,
             wheel_width_m=.12)


@pytest.fixture(scope='module')
def geom():
    return g.SteeringGeometry.from_profile(vp.load_profile())


def test_curvature_yaw_rate_and_radius():
    for deg in (-25., -10., -1., 0., 1., 10., 20., 25.):
        delta = D(deg)
        assert g.curvature(delta, L) == pytest.approx(math.tan(delta) / L)
        assert g.yaw_rate(.4, delta, L) == pytest.approx(.4 * math.tan(delta) / L)
        assert g.delta_for_curvature(g.curvature(delta, L), L) == pytest.approx(delta, abs=1e-15)
    assert g.turn_radius(0., L) == math.inf
    # design 3.2 examples: delta 10 deg -> R 4.31 m, 20 deg -> 2.09 m; v 0.4 m/s at 20 deg -> 11.0 deg/s
    assert g.turn_radius(D(10.), L) == pytest.approx(4.31, abs=.005)
    assert g.turn_radius(D(-10.), L) == pytest.approx(-4.31, abs=.005)
    assert g.turn_radius(D(20.), L) == pytest.approx(2.09, abs=.005)
    assert math.degrees(g.yaw_rate(.4, D(20.), L)) == pytest.approx(11.0, abs=.05)
    # curvature authority: delta 20 deg -> 0.48 1/m
    assert g.curvature(D(20.), L) == pytest.approx(.48, abs=.005)
    for bad in (math.pi / 2, float('nan'), True):
        with pytest.raises(ValueError):
            g.curvature(bad, L)
    with pytest.raises(ValueError):
        g.curvature(.1, 0.)


def test_parallel_linkage_gives_equal_angles():
    for deg in (-25., -5., 0., 5., 25.):
        left, right = g.wheel_angles(D(deg), 'linkage', L, WK, 0.)
        assert left == pytest.approx(D(deg)) and right == pytest.approx(D(deg))


@pytest.mark.parametrize('ratio', [.25, .5, 1.])
def test_inner_wheel_turns_more_than_outer(ratio):
    for deg in (2., 10., 25.):
        left, right = g.wheel_angles(D(deg), 'linkage', L, WK, ratio)
        assert left > D(deg) > right > 0.          # left turn: the left wheel is inside
        left, right = g.wheel_angles(-D(deg), 'linkage', L, WK, ratio)
        assert -right > D(deg) > -left > 0.        # right turn: the right wheel is inside


def test_ideal_ackermann_relations():
    for deg in (5., 15., 25.):
        left, right = g.ideal_ackermann_angles(D(deg), L, WK)
        assert 1. / math.tan(right) - 1. / math.tan(left) == pytest.approx(WK / L)
        assert .5 * (1. / math.tan(left) + 1. / math.tan(right)) == pytest.approx(1. / math.tan(D(deg)))
    left, right = g.ideal_ackermann_angles(D(25.), L, WK)
    # design 3.2 / appendix A: equivalent 25 deg with a = 1 -> inner about 28.5, outer about 22.2
    assert math.degrees(left) == pytest.approx(28.5, abs=.05)
    assert math.degrees(right) == pytest.approx(22.2, abs=.05)


@pytest.mark.parametrize('ratio, expected_deg', [(0., 25.), (.5, 23.5), (1., 22.2)])
def test_equivalent_soft_limit_design_numbers(ratio, expected_deg):
    left, right = g.equivalent_limits(LIMITS_28, D(3.), 'linkage', L, WK, ratio)
    assert math.degrees(left) == pytest.approx(expected_deg, abs=.06)
    assert math.degrees(right) == pytest.approx(expected_deg, abs=.06)


@pytest.mark.parametrize('ratio', [0., .5, 1.])
def test_equivalent_limit_is_tight_and_keeps_both_wheels_inside(ratio):
    soft = D(25.)
    left, right = g.equivalent_limits(LIMITS_28, D(3.), 'linkage', L, WK, ratio)
    wheels = g.wheel_angles(left, 'linkage', L, WK, ratio)
    assert max(wheels) == pytest.approx(soft, abs=1e-12) and all(w <= soft + 1e-12 for w in wheels)
    assert max(g.wheel_angles(left + D(.05), 'linkage', L, WK, ratio)) > soft
    wheels = g.wheel_angles(-right, 'linkage', L, WK, ratio)
    assert max(-w for w in wheels) == pytest.approx(soft, abs=1e-12)
    mech_left, _ = g.equivalent_limits(LIMITS_28, 0., 'linkage', L, WK, ratio)
    assert max(g.wheel_angles(mech_left, 'linkage', L, WK, ratio)) == pytest.approx(D(28.), abs=1e-12)
    assert mech_left > left


def test_asymmetric_wheel_limits():
    limits = dict(left_wheel_toward_left=D(24.), left_wheel_toward_right=D(28.), right_wheel_toward_left=D(28.),
                  right_wheel_toward_right=D(30.))
    left, right = g.equivalent_limits(limits, 0., 'linkage', L, WK, 0.)
    assert (left, right) == (pytest.approx(D(24.)), pytest.approx(D(28.)))   # parallel: the smaller wheel limit
    sym_left, sym_right = g.equivalent_limits(LIMITS_28, 0., 'linkage', L, WK, .5)
    left, right = g.equivalent_limits(limits, 0., 'linkage', L, WK, .5)
    assert left < sym_left and right > sym_right
    with pytest.raises(ValueError):
        g.equivalent_limits(LIMITS_28, D(28.), 'linkage', L, WK, .5)
    with pytest.raises(ValueError):
        g.equivalent_limits({'left': D(28.)}, 0., 'linkage', L, WK, .5)


def test_equivalent_from_wheel_round_trip():
    for ratio in (0., .5, 1.):
        for deg in (-25., -7., 0., 3., 25.):
            left, right = g.wheel_angles(D(deg), 'linkage', L, WK, ratio)
            assert g.equivalent_from_wheel(left, 'left', 'linkage', L, WK, ratio) == pytest.approx(D(deg), abs=1e-12)
            assert g.equivalent_from_wheel(right, 'right', 'linkage', L, WK, ratio) == pytest.approx(D(deg), abs=1e-12)
    with pytest.raises(ValueError):
        g.equivalent_from_wheel(.1, 'middle', 'linkage', L, WK, .5)


def test_axle_pivot_basics():
    for deg in (-20., 0., 20.):
        assert g.wheel_angles(D(deg), 'axle_pivot', L, WK, .5) == (D(deg), D(deg))
    limits = dict(left_wheel_toward_left=D(26.), left_wheel_toward_right=D(28.), right_wheel_toward_left=D(27.),
                  right_wheel_toward_right=D(30.))
    left, right = g.equivalent_limits(limits, D(3.), 'axle_pivot', L, WK, .5)
    assert left == pytest.approx(D(23.)) and right == pytest.approx(D(25.))
    kw = dict(WHEEL, steer_type='axle_pivot')
    del kw['wheel_radius_m'], kw['wheel_width_m']
    assert g.front_wheel_centre('left', 0., **kw) == pytest.approx((.38, .28))
    x, y = g.front_wheel_centre('left', D(20.), **kw)
    assert (x, y) == (pytest.approx(.38 - .28 * math.sin(D(20.))), pytest.approx(.28 * math.cos(D(20.))))
    # no slip: the front axle line meets the rear axle line at y = L / tan(delta), so kappa = tan(delta) / L
    theta = D(15.)
    icr_y = (L / math.sin(theta)) * math.cos(theta)
    assert 1. / icr_y == pytest.approx(g.curvature(theta, L))


def test_linkage_wheel_centre_moves_about_the_kingpin():
    kw = dict(WHEEL)
    del kw['wheel_radius_m'], kw['wheel_width_m']
    assert g.front_wheel_centre('left', 0., **kw) == pytest.approx((.38, .28))
    assert g.front_wheel_centre('right', 0., **kw) == pytest.approx((.38, -.28))
    x, y = g.front_wheel_centre('left', D(28.), **kw)
    assert math.hypot(x - .38, y - .23) == pytest.approx(.05)


@pytest.mark.parametrize('deg, expected', [(28., .043), (25., .039), (20., .031)])
def test_sweep_design_numbers(deg, expected):
    for side in ('left', 'right'):
        for sign in (1., -1.):
            sweep = g.wheel_sweep_beyond_body(side, sign * D(deg), body_half_width_m=.35, **WHEEL)
            assert sweep == pytest.approx(expected, abs=.0006)


def test_sweep_corner_and_margin():
    assert g.wheel_outer_extent('left', D(28.), **WHEEL) == pytest.approx(.393, abs=.0005)
    assert g.wheel_sweep_beyond_body('left', 0., body_half_width_m=.35, **WHEEL) == pytest.approx(-.01)
    # inner wheel (left wheel in a left turn): the front corner sticks out; outer wheel: the rear corner
    corners = g.front_wheel_corners('left', D(28.), **WHEEL)
    centre = g.front_wheel_centre('left', D(28.), steer_type='linkage', front_axle_x_m=.38, track_front_m=.56,
                                  kingpin_width_m=.46)
    assert max(corners, key=lambda p: p[1])[0] > centre[0]
    corners = g.front_wheel_corners('right', D(28.), **WHEEL)
    centre = g.front_wheel_centre('right', D(28.), steer_type='linkage', front_axle_x_m=.38, track_front_m=.56,
                                  kingpin_width_m=.46)
    assert min(corners, key=lambda p: p[1])[0] < centre[0]
    bare = g.wheel_sweep_beyond_body('left', D(28.), body_half_width_m=.35, **WHEEL)
    with_margin = g.wheel_sweep_beyond_body('left', D(28.), body_half_width_m=.35, margin_m=.02, **WHEEL)
    assert with_margin == pytest.approx(bare + .02, abs=1e-12)                # design 3.2: sweep + margin
    limits = g.sweep_at_limits(LIMITS_28, body_half_width_m=.35, steer_type='linkage', track_front_m=.56,
                               kingpin_width_m=.46, wheel_radius_m=.14, wheel_width_m=.12)
    assert set(limits) == set(g.WHEEL_LIMIT_KEYS)
    assert all(v == pytest.approx(.0429, abs=.0001) for v in limits.values())


def test_scrub_radius():
    assert g.scrub_radius(.56, .46, 0., .14) == pytest.approx(.05)
    assert g.scrub_radius(.56, .46, D(5.), .14) == pytest.approx(.05 - .14 * math.tan(D(5.)))


def test_geometry_from_profile(geom):
    profile = vp.load_profile()
    assert geom.front_axle_x_m == pytest.approx(.38)
    left, right = geom.soft_limits_eq()
    assert math.degrees(left) == pytest.approx(profile.value('steer_soft_limit_eq_left_deg'))
    assert math.degrees(right) == pytest.approx(profile.value('steer_soft_limit_eq_right_deg'))
    assert geom.scrub_radius_m() == pytest.approx(profile.value('scrub_radius_m'))
    for key, value in geom.sweep_at_mech_limits().items():
        assert value == pytest.approx(profile.value('steer_sweep_m')[key])
    assert geom.clamp_to_soft(D(40.)) == pytest.approx(left)
    assert geom.clamp_to_soft(-D(40.)) == pytest.approx(-right)
    left_wheel, right_wheel = geom.wheel_angles(D(12.))
    assert geom.equivalent_from_wheels(left_wheel, right_wheel) == pytest.approx(D(12.), abs=1e-12)
    assert geom.curvature(D(10.)) == pytest.approx(math.tan(D(10.)) / .76)
    assert geom.link_table.is_identity()


@pytest.mark.parametrize('ratio', [0., .5, 1.])
def test_clamped_commands_never_exceed_wheel_limits(geom, ratio):
    """A21 (geometry part): commands up to +-60 deg, clamped to the equivalent soft limit, keep each wheel
    inside its soft limit; the equivalent mechanical limit keeps each wheel inside its mechanical limit."""
    variant = replace(geom, ackermann_ratio=ratio)
    soft = variant.soft_wheel_limits()
    for deg in range(-60, 61):
        left, right = variant.wheel_angles(variant.clamp_to_soft(D(deg)))
        assert left <= soft['left_wheel_toward_left'] + 1e-12 and -left <= soft['left_wheel_toward_right'] + 1e-12
        assert right <= soft['right_wheel_toward_left'] + 1e-12 and -right <= soft['right_wheel_toward_right'] + 1e-12
    mech = variant.wheel_limits()
    mech_left, mech_right = variant.mech_limits_eq()
    for delta in (mech_left, -mech_right):
        left, right = variant.wheel_angles(delta)
        assert left <= mech['left_wheel_toward_left'] + 1e-12 and -left <= mech['left_wheel_toward_right'] + 1e-12
        assert right <= mech['right_wheel_toward_left'] + 1e-12 and -right <= mech['right_wheel_toward_right'] + 1e-12


def test_footprint_contains_body_and_wheels(geom):
    body = geom.body_polygon()
    xs, ys = sorted({p[0] for p in body}), sorted({p[1] for p in body})
    assert xs == [pytest.approx(-.61), pytest.approx(.61)] and ys == [pytest.approx(-.35), pytest.approx(.35)]
    for deg in (-23., -10., 0., 10., 23.):
        left, right = geom.wheel_angles(D(deg))
        for margin in (False, True):
            hull = geom.footprint_hull(left, right, include_hardware_margin=margin)
            for corner in body:
                assert g.point_in_convex_polygon(corner, hull)
            for polygon in geom.footprint_polygons(left, right, include_hardware_margin=margin).values():
                for point in polygon:
                    assert g.point_in_convex_polygon(point, hull, 1e-9)
    # at zero steer without the bracket margin the wheels stay inside: the hull is the body
    hull = geom.footprint_hull(0., 0., include_hardware_margin=False)
    assert len(hull) == 4
    for (hx, hy), (bx, by) in zip(sorted(hull), sorted(body)):
        assert hx == pytest.approx(bx) and hy == pytest.approx(by)
    assert g.polygon_support(hull, (0., 1.)) == pytest.approx(.35)


def test_footprint_reach_follows_steering(geom):
    left_reach, right_reach = geom.footprint_lateral_extent(D(28.), D(22.), include_hardware_margin=False)
    assert left_reach == pytest.approx(.393, abs=.0005)
    assert right_reach == pytest.approx(.23 + .14 * math.sin(D(22.)) + .11 * math.cos(D(22.)))
    with_margin = geom.footprint_lateral_extent(D(28.), D(22.), include_hardware_margin=True)
    assert with_margin[0] > left_reach and with_margin[1] > right_reach
    hull = geom.footprint_for_equivalent(D(10.))
    assert g.polygon_support(hull, (1., 0.)) == pytest.approx(.61)   # the bumper stays the front edge


def test_link_table_identity_and_inverse():
    table = g.LinkTable.identity()
    assert table.is_identity()
    for x in (-.8, -.5, 0., .3, .8):           # includes linear extrapolation beyond +-40 deg
        assert table.wheel_from_pot(x) == pytest.approx(x) and table.pot_from_wheel(x) == pytest.approx(x)
    bent = g.LinkTable([(-40., -36.), (0., 0.), (40., 44.)])
    assert not bent.is_identity()
    assert math.degrees(bent.wheel_from_pot(D(20.))) == pytest.approx(22.)
    for x in (-.6, -.1, .2, .6, .75):
        assert bent.pot_from_wheel(bent.wheel_from_pot(x)) == pytest.approx(x)
    for rows in ([(0., 0.), (10., -5.)], [(0., 0.)], [(0., 0.), (0., 5.)], [(0., 0., 1.), (1., 1., 1.)]):
        with pytest.raises(ValueError):
            g.LinkTable(rows)


def test_convex_helpers():
    square = g.rectangle(-1., 1., -1., 1.)
    assert g.point_in_convex_polygon((0., 0.), square) and not g.point_in_convex_polygon((1.1, 0.), square)
    hull = g.convex_hull(square + [(0., 0.), (.5, .5)])
    assert sorted(hull) == sorted(square)
    assert g.polygon_support(square, (1., 1.)) == pytest.approx(math.sqrt(2.))


# --- bracket margin (F3: design 3.2 "sweep + margin", design 7.1 "the band stays for small steering") -------------------
def test_margin_is_zero_while_the_wheel_is_inside_the_body(geom):
    """Straight ahead the wheels are inside the body (reach 0.34 < 0.35 m): the footprint is the body itself."""
    assert geom.hardware_margin_m == pytest.approx(.02)
    for side in ('left', 'right'):
        assert geom.hardware_margin_at(side, 0.) == 0.
        assert geom.wheel_sweep_beyond_body(side, 0., include_hardware_margin=True) == pytest.approx(-.01)
    assert geom.footprint_lateral_extent(0., 0., include_hardware_margin=True) == (pytest.approx(.35),
                                                                                   pytest.approx(.35))
    hull = geom.footprint_hull(0., 0., include_hardware_margin=True)
    assert len(hull) == 4 and g.polygon_support(hull, (0., 1.)) == pytest.approx(.35)


def test_margin_at_the_limits_adds_exactly_the_margin(geom):
    bare = geom.sweep_at_mech_limits(include_hardware_margin=False)
    full = geom.sweep_at_mech_limits(include_hardware_margin=True)
    for key in g.WHEEL_LIMIT_KEYS:
        assert bare[key] == pytest.approx(.0429, abs=1e-4)
        assert full[key] == pytest.approx(bare[key] + .02, abs=1e-12)         # 0.043 + 0.02 = 0.063 m (design 3.2)


def test_margin_ramps_with_the_sweep_and_keeps_the_footprint_continuous(geom):
    previous = None
    for k in range(0, 561):                                                  # 0..28 deg in 0.05 deg steps
        left, right = geom.wheel_angles(D(k * .05))
        reach = geom.footprint_lateral_extent(left, right, include_hardware_margin=True)[0]
        bare = geom.wheel_sweep_beyond_body('left', left, include_hardware_margin=False)
        margin = min(.02, max(0., bare))
        assert reach == pytest.approx(max(.35, .35 + bare + margin), abs=1e-12)
        if previous is not None:
            assert abs(reach - previous) < 1e-3                               # no step in the steering angle
        previous = reach
    left, right = geom.wheel_angles(D(5.))
    reach = geom.footprint_lateral_extent(left, right, include_hardware_margin=True)[0]
    assert .35 < reach < .355                                                # design 7.1: the band barely moves


def test_margin_grows_the_wheel_outward_only():
    bare = g.front_wheel_corners('left', D(20.), **WHEEL)
    grown = g.front_wheel_corners('left', D(20.), margin_m=.02, **WHEEL)
    assert max(y for _, y in grown) == pytest.approx(max(y for _, y in bare) + .02)
    assert min(y for _, y in grown) == pytest.approx(min(y for _, y in bare))      # the inner edge stays
    assert max(x for x, _ in grown) == pytest.approx(max(x for x, _ in bare))      # front and rear stay
    assert min(x for x, _ in grown) == pytest.approx(min(x for x, _ in bare))
    right = g.front_wheel_corners('right', D(-20.), margin_m=.02, **WHEEL)
    assert min(y for _, y in right) == pytest.approx(min(y for _, y in g.front_wheel_corners('right', D(-20.), **WHEEL))
                                                     - .02)
    assert g.effective_hardware_margin(-.01, .02) == 0. and g.effective_hardware_margin(.005, .02) == .005
    assert g.effective_hardware_margin(.05, .02) == .02
    with pytest.raises(ValueError):
        g.effective_hardware_margin(.01, -.02)
