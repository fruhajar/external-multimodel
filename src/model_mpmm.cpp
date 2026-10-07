#include "forces.h"
#include "models.h"

#include "constants.h"

namespace Ballistics::MM {

using namespace Constants;

DataRequirements Mpmm::requirements() {
    return DataBlock::Geometry | DataBlock::Drag | DataBlock::Inertia | DataBlock::Moments;
}

Mpmm::State Mpmm::initialState(const LaunchState& launch, const Projectile&) {
    return {launch.position, launch.velocity, launch.spinRate * 2.0 * PI / 60.0};
}

double Mpmm::alpha(const State& s, const DerivativeContext& ctx) {
    const Forces::AirState air = Forces::airState(s.pos, s.vel, ctx);
    return glm::length(Forces::yawOfRepose(air, ctx, s.spin));
}

Mpmm::State Mpmm::derivative(const State& s, const DerivativeContext& ctx) {
    const Projectile& p = *ctx.projectile;
    const Forces::AirState air = Forces::airState(s.pos, s.vel, ctx);

    // The repose angle the current spin sustains against the turning of the velocity vector.
    const glm::dvec3 repose = Forces::yawOfRepose(air, ctx, s.spin);

    glm::dvec3 force = p.geometry.mass * Forces::gravity(air.altitudeMsl);
    force += p.geometry.mass * Forces::coriolis(s.vel, glm::radians(ctx.env->latitude),
                                                glm::radians(ctx.env->launchAzimuth));
    force += Forces::drag(air, ctx, glm::length(repose));
    force += Forces::thrust(air, ctx);
    force += Forces::normalForce(air, ctx, repose);

    State d;
    d.pos = s.vel;
    d.vel = force / p.geometry.mass;
    d.spin = Forces::rollAcceleration(air, ctx, s.spin);
    return d;
}

void Mpmm::reserve(Trajectory& out, std::size_t n) {
    out.reserveKinematics(n);
    out.spin.reserve(n);
    out.alpha.reserve(n);
}

void Mpmm::append(Trajectory& out, double t, const State& s, const DerivativeContext& ctx) {
    out.appendKinematics(t, s.pos, s.vel);
    out.spin.push_back(s.spin);
    out.alpha.push_back(alpha(s, ctx));
}

void Mpmm::fillImpact(Impact& i, const State& s, const DerivativeContext& ctx) {
    i.velocity = s.vel;
    i.terminalSpeed = glm::length(s.vel);
    if (i.terminalSpeed > 1e-9) {
        i.impactAngle = -std::asin(s.vel.y / i.terminalSpeed);
    }
    i.spin = s.spin;
    i.alpha = alpha(s, ctx);
}

} // namespace Ballistics::MM
