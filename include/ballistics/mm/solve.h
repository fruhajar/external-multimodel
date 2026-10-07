#ifndef BALLISTICS_MM_SOLVE_H
#define BALLISTICS_MM_SOLVE_H

#include "predict.h"

namespace Ballistics::MM {

struct LaunchAim {
    double elevation = 0.0;
    double azimuthOffset = 0.0;
    double tof = 0.0;
    double residual = 0.0;
    int iterations = 0;
};

Result<LaunchAim> solveLaunchDirection(ModelKind model,
                                       const LaunchState& from,
                                       const glm::dvec3& target,
                                       const Projectile& projectile,
                                       const Environment& env,
                                       double minElevation,
                                       double maxElevation,
                                       const SolverConfig& config = SolverConfig::standard());

struct AimPair {
    Result<LaunchAim> direct;
    Result<LaunchAim> indirect;
};

AimPair solveLaunchDirections(ModelKind model,
                              const LaunchState& from,
                              const glm::dvec3& target,
                              const Projectile& projectile,
                              const Environment& env,
                              const SolverConfig& config = SolverConfig::standard());

struct RangeEnvelope {
    double elevation = 0.0;
    double range = 0.0;
};

Result<RangeEnvelope> maxRange(ModelKind model,
                               const LaunchState& from,
                               const Projectile& projectile,
                               const Environment& env,
                               const GroundReference& ground,
                               const SolverConfig& config = SolverConfig::standard());

// One row of a range table: what to dial for a given range, under one met condition.
struct RangeTableRow {
    double range = 0.0;
    double elevation = 0.0;
    double azimuthOffset = 0.0;
    double tof = 0.0;
    double terminalSpeed = 0.0;
    double impactAngle = 0.0;
};

// Sweeps range and solves each row. The mid-range deliverable the solver seam exists for.
std::vector<RangeTableRow> rangeTable(ModelKind model,
                                     const LaunchState& from,
                                     const Projectile& projectile,
                                     const Environment& env,
                                     const GroundReference& ground,
                                     double firstRange,
                                     double lastRange,
                                     double step,
                                     const SolverConfig& config = SolverConfig::standard());

} // namespace Ballistics::MM

#endif
