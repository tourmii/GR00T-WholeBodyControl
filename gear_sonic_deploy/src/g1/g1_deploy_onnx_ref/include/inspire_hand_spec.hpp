/**
 * @file inspire_hand_spec.hpp
 * @brief C++ mirror of gear_sonic/utils/inspire_hand_spec.py.
 *
 * Single source of truth for the Inspire RH56 6-DOF hand on the deploy side.
 * Drive order matches the SDK angle_set / angle_act arrays:
 *   0 pinky, 1 ring, 2 middle, 3 index, 4 thumb_bend, 5 thumb_rot.
 *
 * Drive units are integers [0, 1000] where 1000 = fully open, 0 = fully closed
 * (Inspire FTP manual). We model each drive as flexion-from-open: q == 0 (rad)
 * is open, q == Q_CLOSED is closed. Mapping is linear and kept numerically
 * identical to the Python spec.
 */

#ifndef INSPIRE_HAND_SPEC_HPP
#define INSPIRE_HAND_SPEC_HPP

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace inspire_hand_spec {

/// Number of actuated drives on a single Inspire hand.
static constexpr int INSPIRE_DOF = 6;

/// Drive value bounds used by the Inspire SDK (normalized int16).
static constexpr int ANGLE_MIN = 0;
static constexpr int ANGLE_MAX = 1000;

/// Per-joint flexion travel (radians), ordered to match the SDK drive order.
/// Q_OPEN is all zeros; q == Q_CLOSED maps to drive value 0 (fully closed).
static constexpr std::array<double, INSPIRE_DOF> Q_CLOSED = {
    2.72, 2.72, 2.72, 2.72, 1.45, 1.31};

/// Map joint angles (rad, SDK order) to Inspire drive units [0, 1000].
inline std::vector<int16_t> rad_to_drive(const std::array<double, INSPIRE_DOF>& q) {
    std::vector<int16_t> drive(INSPIRE_DOF);
    for (int i = 0; i < INSPIRE_DOF; ++i) {
        const double span = Q_CLOSED[i] != 0.0 ? Q_CLOSED[i] : 1.0;  // Q_OPEN == 0
        double d = static_cast<double>(ANGLE_MAX) * (Q_CLOSED[i] - q[i]) / span;
        d = std::clamp(d, static_cast<double>(ANGLE_MIN), static_cast<double>(ANGLE_MAX));
        drive[i] = static_cast<int16_t>(std::lround(d));
    }
    return drive;
}

/// Map Inspire drive units [0, 1000] (SDK order) to joint angles (rad).
inline std::array<double, INSPIRE_DOF> drive_to_rad(const std::vector<int16_t>& drive) {
    std::array<double, INSPIRE_DOF> q{};
    for (int i = 0; i < INSPIRE_DOF; ++i) {
        double d = (i < static_cast<int>(drive.size())) ? static_cast<double>(drive[i]) : 0.0;
        d = std::clamp(d, static_cast<double>(ANGLE_MIN), static_cast<double>(ANGLE_MAX));
        q[i] = Q_CLOSED[i] - (d / static_cast<double>(ANGLE_MAX)) * Q_CLOSED[i];  // Q_OPEN == 0
    }
    return q;
}

}  // namespace inspire_hand_spec

#endif  // INSPIRE_HAND_SPEC_HPP
