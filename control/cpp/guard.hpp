#pragma once
#include <cmath>
#include <stdexcept>

namespace elf {
struct Step { double vx, yaw_rate, seconds; };
inline void validate(const Step& s) {
  if (!std::isfinite(s.vx) || !std::isfinite(s.yaw_rate) || !std::isfinite(s.seconds) ||
      s.vx < 0 || s.vx > .20 || std::abs(s.yaw_rate) > .35 ||
      s.seconds <= 0 || s.seconds > 1.25 ||
      (s.vx == 0 && s.yaw_rate == 0) || (s.vx != 0 && s.yaw_rate != 0))
    throw std::runtime_error("step limits: forward 0..0.20 m/s OR turn +/-0.35 rad/s; duration (0,1.25] s");
}
// Stable diagnostic names; nullptr means every check passed.
inline const char* rejection(double age, unsigned error, unsigned mode,
                             double roll, double pitch, bool starting) {
  if (!std::isfinite(age) || age < 0 || age > .5) return "state_stale";
  // Deployment exception explicitly authorized by the operator; meaning unverified.
  if (error != 0 && error != 2) return "robot_error_code_nonzero";
  if (mode != 1 && (starting || mode != 3)) return "mode_not_allowed";
  if (!std::isfinite(roll) || !std::isfinite(pitch)) return "posture_nonfinite";
  if (std::abs(roll) >= .35 || std::abs(pitch) >= .35) return "excessive_tilt";
  return nullptr;
}
inline bool healthy(double age, unsigned error, unsigned mode,
                    double roll, double pitch, bool starting) {
  return rejection(age, error, mode, roll, pitch, starting) == nullptr;
}
}
