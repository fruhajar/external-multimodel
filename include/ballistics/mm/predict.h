#ifndef BALLISTICS_MM_PREDICT_H
#define BALLISTICS_MM_PREDICT_H

#include "config.h"
#include "model.h"

namespace Ballistics::MM {

// Non-template entry points, so the library exposes a stable surface and the model templates stay
// inside it. The projectile is validated against the model's data requirements before integrating.
Result<Impact> predictImpact(ModelKind model,
                             const LaunchState& launch,
                             const Projectile& projectile,
                             const Environment& env,
                             const GroundReference& ground,
                             const SolverConfig& config = SolverConfig::standard());

Result<Trajectory> predict(ModelKind model,
                           const LaunchState& launch,
                           const Projectile& projectile,
                           const Environment& env,
                           const GroundReference& ground,
                           const SolverConfig& config = SolverConfig::standard());

} // namespace Ballistics::MM

#endif
