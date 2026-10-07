#include "ballistics/mm/predict.h"

#include "driver.h"

namespace Ballistics::MM {

DataRequirements requirementsOf(ModelKind k) {
    switch (k) {
    case ModelKind::PointMass: return PointMass::requirements();
    case ModelKind::Mpmm:      return Mpmm::requirements();
    case ModelKind::RigidBody: return RigidBody::requirements();
    }
    return 0;
}

Result<Trajectory> predict(ModelKind model, const LaunchState& launch,
                           const Projectile& projectile, const Environment& env,
                           const GroundReference& ground, const SolverConfig& config) {
    switch (model) {
    case ModelKind::PointMass:
        return integrate<PointMass>(launch, projectile, env, ground, config, true);
    case ModelKind::Mpmm:
        return integrate<Mpmm>(launch, projectile, env, ground, config, true);
    case ModelKind::RigidBody:
        return integrate<RigidBody>(launch, projectile, env, ground, config, true);
    }
    return Result<Trajectory>::fail(Status::InvalidInput);
}

Result<Impact> predictImpact(ModelKind model, const LaunchState& launch,
                             const Projectile& projectile, const Environment& env,
                             const GroundReference& ground, const SolverConfig& config) {
    Result<Trajectory> r = [&] {
        switch (model) {
        case ModelKind::PointMass:
            return integrate<PointMass>(launch, projectile, env, ground, config, false);
        case ModelKind::Mpmm:
            return integrate<Mpmm>(launch, projectile, env, ground, config, false);
        case ModelKind::RigidBody:
            return integrate<RigidBody>(launch, projectile, env, ground, config, false);
        }
        return Result<Trajectory>::fail(Status::InvalidInput);
    }();
    return {r.status, r.value.impact};
}

} // namespace Ballistics::MM
