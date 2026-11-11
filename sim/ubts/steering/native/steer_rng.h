// Deterministic random streams for the native steering model.
//
// splitmix64 expands (seed, stream tag) into a xoshiro256** state; normals come from
// Box-Muller with one cached spare. Every noise source has its own stream so that
// changing one source does not shift the values of another (protocol section 10).
// Determinism is required within one platform and one build only (decision R1):
// log/sin/cos come from the C library of the build.
#ifndef UBTS_STEER_RNG_H
#define UBTS_STEER_RNG_H

#include <stdint.h>

#include <cmath>

namespace ubts_steer {

// Stream tags (fixed constants; part of the deterministic model definition).
const uint64_t kStreamPotNoise = 0x706f745f6e6f6973ULL;     // ADC sample noise
const uint64_t kStreamBacklashWander = 0x77616e6465725f62ULL;  // link wander inside the play
const uint64_t kStreamReleaseNoise = 0x72656c656173655fULL;   // free-wheeling road noise

inline uint64_t splitmix64_next(uint64_t *state) {
  uint64_t z = (*state += 0x9e3779b97f4a7c15ULL);
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
  return z ^ (z >> 31);
}

class Stream {
 public:
  Stream() { seed(0, 0); }

  void seed(uint64_t seed_value, uint64_t tag) {
    uint64_t sm = seed_value ^ tag;
    for (int i = 0; i < 4; ++i) s_[i] = splitmix64_next(&sm);
    if ((s_[0] | s_[1] | s_[2] | s_[3]) == 0) s_[0] = 1;
    have_spare_ = false;
    spare_ = 0.0;
  }

  uint64_t next() {
    const uint64_t result = rotl(s_[1] * 5, 7) * 9;
    const uint64_t t = s_[1] << 17;
    s_[2] ^= s_[0];
    s_[3] ^= s_[1];
    s_[1] ^= s_[2];
    s_[0] ^= s_[3];
    s_[2] ^= t;
    s_[3] = rotl(s_[3], 45);
    return result;
  }

  // Uniform in [0, 1) with 53 random bits.
  double uniform() { return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0); }

  // Uniform in (0, 1]; safe for log().
  double uniform_open_zero() { return static_cast<double>((next() >> 11) + 1) * (1.0 / 9007199254740992.0); }

  // Standard normal (Box-Muller, pairs; the second value of a pair is cached).
  double normal() {
    if (have_spare_) {
      have_spare_ = false;
      return spare_;
    }
    const double u1 = uniform_open_zero();
    const double u2 = uniform();
    const double r = std::sqrt(-2.0 * std::log(u1));
    const double a = 6.283185307179586 * u2;
    spare_ = r * std::sin(a);
    have_spare_ = true;
    return r * std::cos(a);
  }

 private:
  static uint64_t rotl(uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }

  uint64_t s_[4];
  bool have_spare_;
  double spare_;
};

}  // namespace ubts_steer

#endif  // UBTS_STEER_RNG_H
