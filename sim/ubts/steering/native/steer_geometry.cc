// Steering geometry used by the native model (see steer_geometry.h).
#include "steer_geometry.h"

#include <algorithm>
#include <cmath>

namespace ubts_steer {

double PiecewiseLinear::operator()(double x) const {
  // i = bisect_right(xs, x) - 1, clamped to [0, n - 2] (geometry._interp).
  long i = static_cast<long>(std::upper_bound(xs_.begin(), xs_.end(), x) - xs_.begin()) - 1;
  const long last = static_cast<long>(xs_.size()) - 2;
  if (i < 0) i = 0;
  if (i > last) i = last;
  const size_t k = static_cast<size_t>(i);
  const double x0 = xs_[k], x1 = xs_[k + 1], y0 = ys_[k], y1 = ys_[k + 1];
  return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

void LinkMap::set(const std::vector<double> &pot_side, const std::vector<double> &wheel_side, double lr_asymmetry,
                  double zero_offset) {
  forward_.set(pot_side, wheel_side);
  inverse_.set(wheel_side, pot_side);
  asymmetry_ = lr_asymmetry;
  offset_ = zero_offset;
}

double LinkMap::delta_eq(double link) const {
  const double t = forward_(link);
  const double s = t > 0.0 ? 1.0 : (t < 0.0 ? -1.0 : 0.0);
  return t * (1.0 + asymmetry_ * s) + offset_;
}

double LinkMap::link_from_delta_eq(double delta) const {
  double y = delta - offset_;
  if (y > 0.0) {
    y /= 1.0 + asymmetry_;
  } else if (y < 0.0) {
    y /= 1.0 - asymmetry_;
  }
  return inverse_(y);
}

void wheel_angles(double delta, bool linkage, double wheelbase_m, double kingpin_width_m, double ackermann_ratio,
                  double *left, double *right) {
  if (!linkage) {
    *left = delta;
    *right = delta;
    return;
  }
  const double k = kingpin_width_m / (2.0 * wheelbase_m);
  const double s = std::sin(delta), c = std::cos(delta);
  const double ideal_left = std::atan2(s, c - k * s);
  const double ideal_right = std::atan2(s, c + k * s);
  *left = delta + ackermann_ratio * (ideal_left - delta);
  *right = delta + ackermann_ratio * (ideal_right - delta);
}

}  // namespace ubts_steer
