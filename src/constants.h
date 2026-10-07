#ifndef BALLISTICS_MM_CONSTANTS_H
#define BALLISTICS_MM_CONSTANTS_H

namespace Ballistics::MM::Constants {

inline constexpr double G = 9.80665;
inline constexpr double R_DRY = 287.05;          // J/(kg K)
inline constexpr double R_VAPOUR = 461.495;      // J/(kg K)
inline constexpr double GAMMA = 1.4;
inline constexpr double LAPSE = 0.0065;          // K/m
inline constexpr double TROPOPAUSE = 11000.0;    // m
inline constexpr double OMEGA_EARTH = 7.2921159e-5;
inline constexpr double R_EARTH = 6371000.0;
inline constexpr double PI = 3.14159265358979323846;

} // namespace Ballistics::MM::Constants

#endif
