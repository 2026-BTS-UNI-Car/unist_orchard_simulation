// Steering geometry used by the native model (protocol section 7, design 3.2).
//
// All angles are on the steering equivalent-angle scale (rad, + left).
//   T(x)      link table: pot-side angle -> wheel-side equivalent angle, piecewise linear with
//             linear extrapolation beyond the end rows (same arithmetic as geometry._interp).
//   delta_eq  = T(link) * (1 + lr_asymmetry * s) + zero_offset, s = sign(T(link)).
//   wheels    linkage: k = w_k / (2 L), ideal_left = atan2(sin d, cos d - k sin d),
//             ideal_right = atan2(sin d, cos d + k sin d), wheel_i = d + a (ideal_i - d);
//             axle_pivot: both wheels = d (same formulas as geometry.wheel_angles).
#ifndef UBTS_STEER_GEOMETRY_H
#define UBTS_STEER_GEOMETRY_H

#include <vector>

namespace ubts_steer {

class PiecewiseLinear {
 public:
  void set(const std::vector<double> &xs, const std::vector<double> &ys) {
    xs_ = xs;
    ys_ = ys;
  }
  double operator()(double x) const;

 private:
  std::vector<double> xs_, ys_;
};

class LinkMap {
 public:
  LinkMap() : asymmetry_(0.0), offset_(0.0) {}
  void set(const std::vector<double> &pot_side, const std::vector<double> &wheel_side, double lr_asymmetry,
           double zero_offset);
  double table(double pot_side) const { return forward_(pot_side); }
  double table_inverse(double wheel_side) const { return inverse_(wheel_side); }
  // Wheel-side equivalent angle of a link-side angle (with left/right gain and zero offset).
  double delta_eq(double link) const;
  // Inverse of delta_eq() (monotonic for |lr_asymmetry| < 1).
  double link_from_delta_eq(double delta) const;

 private:
  PiecewiseLinear forward_, inverse_;
  double asymmetry_, offset_;
};

// Left and right wheel angles of the equivalent angle delta.
void wheel_angles(double delta, bool linkage, double wheelbase_m, double kingpin_width_m, double ackermann_ratio,
                  double *left, double *right);

}  // namespace ubts_steer

#endif  // UBTS_STEER_GEOMETRY_H
