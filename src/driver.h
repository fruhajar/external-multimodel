#ifndef BALLISTICS_MM_DRIVER_H
#define BALLISTICS_MM_DRIVER_H

#include "ballistics/mm/config.h"
#include "models.h"

#include <cmath>

namespace Ballistics::MM {

// One integration loop for every model. The model supplies the derivative and how to sample
// itself; everything about stepping, impact detection and sampling lives here.
template <TrajectoryModel M>
Result<Trajectory> integrate(const LaunchState& launch, const Projectile& projectile,
                             const Environment& env, const GroundReference& ground,
                             const SolverConfig& config, bool store);

} // namespace Ballistics::MM

#endif
