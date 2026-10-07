#include "forces.h"

#include "constants.h"

#include <cmath>

namespace Ballistics::MM::Forces {

using namespace Constants;

namespace {

constexpr double STILL_AIR = 0.1;    // m/s below which aerodynamic terms are dropped
constexpr double MIN_MOMENT_SPEED = 1.0;

} // namespace

AirState airState(const glm::dvec3& pos, const glm::dvec3& vel, const DerivativeContext& ctx) {
    AirState air;
    air.altitudeMsl = ctx.baseAltitude + pos.y;

    air.velocityRel = vel - ctx.env->wind->at(pos.y - ctx.groundRel, ctx.env->launchAzimuth);
    air.speed = glm::length(air.velocityRel);
    air.moving = air.speed >= STILL_AIR;

    if (air.moving) {
        air.density = ctx.env->atmosphere->density(air.altitudeMsl);
        air.mach = air.speed / ctx.env->atmosphere->speedOfSound(air.altitudeMsl);
        air.dynamicPressure = 0.5 * air.density * air.speed * air.speed;
    }
    return air;
}

glm::dvec3 gravity(double altitudeMsl) {
    const double ratio = R_EARTH / (R_EARTH + altitudeMsl);
    return glm::dvec3(0.0, -G * ratio * ratio, 0.0);
}

glm::dvec3 coriolis(const glm::dvec3& vel, double latitudeRad, double azimuthRad) {
    const double sinLat = std::sin(latitudeRad);
    const double cosLat = std::cos(latitudeRad);
    const double sinAz = std::sin(azimuthRad);
    const double cosAz = std::cos(azimuthRad);

    const double east = vel.x * sinAz + vel.z * cosAz;
    const double north = vel.x * cosAz - vel.z * sinAz;

    const double aEast = 2.0 * OMEGA_EARTH * (north * sinLat - vel.y * cosLat);
    const double aNorth = -2.0 * OMEGA_EARTH * east * sinLat;
    const double aUp = 2.0 * OMEGA_EARTH * east * cosLat;

    return glm::dvec3(aEast * sinAz + aNorth * cosAz, aUp, aEast * cosAz - aNorth * sinAz);
}

glm::dvec3 drag(const AirState& air, const DerivativeContext& ctx, double alpha) {
    if (!air.moving) {
        return glm::dvec3(0.0);
    }
    const Projectile& p = *ctx.projectile;
    double cd = p.drag->at(air.mach, alpha);
    if (p.motor && ctx.tof < p.motor->burnTime) {
        cd *= p.motor->dragFactorDuringBurn;
    }
    return -air.dynamicPressure * cd * p.geometry.refArea() * (air.velocityRel / air.speed);
}

glm::dvec3 thrust(const AirState& air, const DerivativeContext& ctx) {
    const Projectile& p = *ctx.projectile;
    if (!p.motor || ctx.tof > p.motor->burnTime || !air.moving) {
        return glm::dvec3(0.0);
    }
    return p.motor->thrust * (air.velocityRel / air.speed);
}

glm::dvec3 yawOfRepose(const AirState& air, const DerivativeContext& ctx, double spin) {
    const Projectile& p = *ctx.projectile;
    if (!air.moving || air.speed < MIN_MOMENT_SPEED || spin == 0.0 || !p.inertia || !p.moments ||
        p.moments->C_M_alpha <= 0.0) {
        return glm::dvec3(0.0);
    }

    const double scale = -(8.0 * p.inertia->Ixx * spin) /
                         (PI * air.density * std::pow(p.geometry.diameter, 3.0) *
                          std::pow(air.speed, 4.0) * p.moments->C_M_alpha);

    return scale * glm::cross(air.velocityRel, gravity(air.altitudeMsl));
}

glm::dvec3 normalForce(const AirState& air, const DerivativeContext& ctx,
                       const glm::dvec3& alphaVector) {
    const Projectile& p = *ctx.projectile;
    if (!air.moving || !p.moments) {
        return glm::dvec3(0.0);
    }
    return air.dynamicPressure * p.geometry.refArea() * p.moments->C_L_alpha * alphaVector;
}

glm::dvec3 overturningMoment(const MomentInputs& in, const Projectile& p) {
    if (in.speed < MIN_MOMENT_SPEED || !p.moments) {
        return glm::dvec3(0.0);
    }

    // Linear in the angle of attack, which is the regime a stabilised round stays in: the measured
    // yaw of repose here peaks near one degree. A large-yaw correction would need measured
    // coefficients at those angles, and inventing a rolloff curve would be worse than not having
    // one, because it would look like data.
    const double slope = p.moments->C_M_alpha;

    // Axis perpendicular to the plane of attack. For a body axis displaced towards +z this is
    // -y, which is the nose-down moment a descending trajectory demands of a right-hand spin.
    static const glm::dvec3 bodyX(1.0, 0.0, 0.0);
    return in.q * p.geometry.refArea() * p.geometry.diameter * slope *
           glm::cross(bodyX, in.bodyAlpha);
}

glm::dvec3 magnusMoment(const MomentInputs& in, const Projectile& p) {
    const double alphaMag = glm::length(in.bodyAlpha);
    if (alphaMag < 1e-9 || in.speed < MIN_MOMENT_SPEED || !p.moments) {
        return glm::dvec3(0.0);
    }

    const double spinHat = in.spin * p.geometry.diameter / (2.0 * in.speed);
    return in.q * p.geometry.refArea() * p.geometry.diameter * p.moments->C_M_palpha * spinHat *
           in.bodyAlpha;
}

glm::dvec3 spinDampingMoment(const MomentInputs& in, const Projectile& p) {
    if (in.speed < MIN_MOMENT_SPEED || !p.moments) {
        return glm::dvec3(0.0);
    }
    const double spinHat = in.spin * p.geometry.diameter / (2.0 * in.speed);
    return glm::dvec3(in.q * p.geometry.refArea() * p.geometry.diameter * p.moments->C_l_p *
                          spinHat,
                      0.0, 0.0);
}

glm::dvec3 pitchDampingMoment(const MomentInputs& in, const Projectile& p) {
    const glm::dvec3 transverse(0.0, in.bodyOmega.y, in.bodyOmega.z);
    const double rate = glm::length(transverse);
    if (rate < 1e-9 || in.speed < MIN_MOMENT_SPEED || !p.moments) {
        return glm::dvec3(0.0);
    }
    const double rateHat = rate * p.geometry.diameter / (2.0 * in.speed);
    return in.q * p.geometry.refArea() * p.geometry.diameter * p.moments->C_M_q * rateHat *
           (transverse / rate);
}

glm::dvec3 finRollingMoment(const MomentInputs& in, const Projectile& p) {
    if (!p.fins || !p.moments || std::abs(p.fins->cantAngle) < 1e-12 ||
        in.speed < MIN_MOMENT_SPEED) {
        return glm::dvec3(0.0);
    }
    return glm::dvec3(in.q * p.geometry.refArea() * p.geometry.diameter * p.moments->C_l_delta *
                          p.fins->cantAngle,
                      0.0, 0.0);
}

double rollAcceleration(const AirState& air, const DerivativeContext& ctx, double spin) {
    const Projectile& p = *ctx.projectile;
    if (!air.moving || air.speed < MIN_MOMENT_SPEED || !p.inertia || !p.moments) {
        return 0.0;
    }

    MomentInputs in;
    in.spin = spin;
    in.speed = air.speed;
    in.q = air.dynamicPressure;

    const double moment = spinDampingMoment(in, p).x + finRollingMoment(in, p).x;
    return moment / p.inertia->Ixx;
}

} // namespace Ballistics::MM::Forces
