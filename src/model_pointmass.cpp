#include "forces.h"
#include "models.h"

#include "constants.h"

namespace Ballistics::MM {

using namespace Constants;

DataRequirements PointMass::requirements() {
    return DataBlock::Geometry | DataBlock::Drag;
}

PointMass::State PointMass::initialState(const LaunchState& launch, const Projectile&) {
    return {launch.position, launch.velocity};
}

PointMass::State PointMass::derivative(const State& s, const DerivativeContext& ctx) {
    const Projectile& p = *ctx.projectile;
    const Forces::AirState air = Forces::airState(s.pos, s.vel, ctx);

    glm::dvec3 force = p.geometry.mass * Forces::gravity(air.altitudeMsl);
    force += p.geometry.mass * Forces::coriolis(s.vel, glm::radians(ctx.env->latitude),
                                                glm::radians(ctx.env->launchAzimuth));
    force += Forces::drag(air, ctx, 0.0);
    force += Forces::thrust(air, ctx);

    return {s.vel, force / p.geometry.mass};
}

void PointMass::reserve(Trajectory& out, std::size_t n) {
    out.reserveKinematics(n);
}

void PointMass::append(Trajectory& out, double t, const State& s, const DerivativeContext&) {
    out.appendKinematics(t, s.pos, s.vel);
}

void PointMass::fillImpact(Impact& i, const State& s, const DerivativeContext&) {
    i.velocity = s.vel;
    i.terminalSpeed = glm::length(s.vel);
    if (i.terminalSpeed > 1e-9) {
        i.impactAngle = -std::asin(s.vel.y / i.terminalSpeed);
    }
}

} // namespace Ballistics::MM
