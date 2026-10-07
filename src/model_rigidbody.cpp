#include "forces.h"
#include "models.h"

#include "constants.h"

namespace Ballistics::MM {

using namespace Constants;

namespace {

// Angle-of-attack vector in body axes: magnitude is the total angle of attack, direction is where
// the axis of symmetry points relative to the relative wind. No x component by construction.
//
// The direction matters and is easy to get backwards. The normal force acts on the side the body
// axis is displaced towards, so bodyAlpha follows the axis, not the wind. In body coordinates the
// wind appears displaced the opposite way, hence the negation.
struct BodyAttitude {
    glm::dvec3 bodyVelocityUnit{1.0, 0.0, 0.0};
    glm::dvec3 bodyAlpha{0.0};
    double total = 0.0;
};

BodyAttitude attitudeOf(const RigidBodyState& s, const Forces::AirState& air) {
    BodyAttitude a;
    if (!air.moving) {
        return a;
    }

    const glm::dvec3 bodyV = glm::conjugate(s.orientation) * air.velocityRel;
    a.bodyVelocityUnit = bodyV / air.speed;

    static const glm::dvec3 bodyX(1.0, 0.0, 0.0);
    a.total = std::acos(glm::clamp(glm::dot(bodyX, a.bodyVelocityUnit), -1.0, 1.0));

    const double transverse = std::sqrt(a.bodyVelocityUnit.y * a.bodyVelocityUnit.y +
                                       a.bodyVelocityUnit.z * a.bodyVelocityUnit.z);
    if (transverse > 1e-12) {
        a.bodyAlpha = glm::dvec3(0.0, -a.total * a.bodyVelocityUnit.y / transverse,
                                 -a.total * a.bodyVelocityUnit.z / transverse);
    }
    return a;
}

} // namespace

DataRequirements RigidBody::requirements() {
    return DataBlock::Geometry | DataBlock::Drag | DataBlock::Inertia | DataBlock::Moments;
}

RigidBody::State RigidBody::initialState(const LaunchState& launch, const Projectile&) {
    State s;
    s.pos = launch.position;
    s.vel = launch.velocity;
    s.omega = glm::dvec3(launch.spinRate * 2.0 * PI / 60.0, 0.0, 0.0);

    // Point the axis of symmetry along the velocity, then apply any initial yaw. Without a
    // nonzero initialYaw the round flies at exactly zero angle of attack and shows no yawing
    // motion at all, which is a degenerate case rather than a useful one.
    const double speed = glm::length(launch.velocity);
    if (speed < 1e-9) {
        return s;
    }

    static const glm::dvec3 bodyX(1.0, 0.0, 0.0);
    const glm::dvec3 direction = launch.velocity / speed;
    glm::dvec3 axis = glm::cross(bodyX, direction);
    const double axisLength = glm::length(axis);

    if (axisLength < 1e-12) {
        s.orientation = glm::dot(bodyX, direction) < 0.0
                            ? glm::angleAxis(PI, glm::dvec3(0.0, 1.0, 0.0))
                            : glm::dquat(1.0, 0.0, 0.0, 0.0);
    } else {
        s.orientation = glm::angleAxis(std::acos(glm::clamp(glm::dot(bodyX, direction), -1.0, 1.0)),
                                       axis / axisLength);
    }

    if (launch.initialYaw != 0.0) {
        glm::dvec3 perpendicular = glm::cross(direction, glm::dvec3(0.0, 1.0, 0.0));
        if (glm::length(perpendicular) < 1e-6) {
            perpendicular = glm::cross(direction, glm::dvec3(1.0, 0.0, 0.0));
        }
        s.orientation = glm::normalize(
            glm::angleAxis(launch.initialYaw, glm::normalize(perpendicular)) * s.orientation);
    }
    return s;
}

double RigidBody::alpha(const State& s, const DerivativeContext& ctx) {
    return attitudeOf(s, Forces::airState(s.pos, s.vel, ctx)).total;
}

RigidBody::State RigidBody::derivative(const State& s, const DerivativeContext& ctx) {
    const Projectile& p = *ctx.projectile;
    const Forces::AirState air = Forces::airState(s.pos, s.vel, ctx);
    const BodyAttitude att = attitudeOf(s, air);

    glm::dvec3 force = p.geometry.mass * Forces::gravity(air.altitudeMsl);
    force += p.geometry.mass * Forces::coriolis(s.vel, glm::radians(ctx.env->latitude),
                                                glm::radians(ctx.env->launchAzimuth));
    force += Forces::drag(air, ctx, att.total);
    force += Forces::thrust(air, ctx);

    // The normal force acts in the world frame, on the side the body axis is displaced towards.
    // Without it a spun shell would have nothing to turn its yaw of repose into drift.
    if (att.total > 1e-12) {
        force += Forces::normalForce(air, ctx, s.orientation * att.bodyAlpha);
    }

    Forces::MomentInputs in;
    in.bodyAlpha = att.bodyAlpha;
    in.bodyVelocityUnit = att.bodyVelocityUnit;
    in.bodyOmega = s.omega;
    in.spin = s.omega.x;
    in.speed = air.speed;
    in.q = air.dynamicPressure;

    glm::dvec3 moment = Forces::spinDampingMoment(in, p) + Forces::finRollingMoment(in, p) +
                        Forces::pitchDampingMoment(in, p);
    if (att.total > 1e-12) {
        moment += Forces::overturningMoment(in, p) + Forces::magnusMoment(in, p);
    }

    State d;
    d.pos = s.vel;
    d.vel = force / p.geometry.mass;

    // Euler's equations for an axisymmetric body, I = diag(C, A, A):
    //   omega x (I omega) = (0, (C - A) wx wz, -(C - A) wx wy)
    //   I omega_dot = M - omega x (I omega)
    // Getting either sign wrong reverses gyroscopic precession, which is the dominant
    // rotational effect for a spun round.
    const double axial = p.inertia->Ixx;
    const double transverse = p.inertia->Iyy;
    const double coupling = axial / transverse - 1.0;

    d.omega = glm::dvec3(moment.x / axial,
                         moment.y / transverse - coupling * s.omega.x * s.omega.z,
                         moment.z / p.inertia->Izz + coupling * s.omega.x * s.omega.y);

    // omega is body-frame, so the kinematic relation is q_dot = 0.5 * q * omega. The reversed
    // order is the world-frame relation and silently conflates the two frames.
    const glm::dquat spinQuat(0.0, s.omega.x, s.omega.y, s.omega.z);
    d.orientation = 0.5 * s.orientation * spinQuat;

    return d;
}

void RigidBody::reserve(Trajectory& out, std::size_t n) {
    out.reserveKinematics(n);
    out.spin.reserve(n);
    out.alpha.reserve(n);
    out.orientation.reserve(n);
    out.omega.reserve(n);
}

void RigidBody::append(Trajectory& out, double t, const State& s, const DerivativeContext& ctx) {
    out.appendKinematics(t, s.pos, s.vel);
    out.spin.push_back(s.omega.x);
    out.alpha.push_back(alpha(s, ctx));
    out.orientation.push_back(s.orientation);
    out.omega.push_back(s.omega);
}

void RigidBody::fillImpact(Impact& i, const State& s, const DerivativeContext& ctx) {
    i.velocity = s.vel;
    i.terminalSpeed = glm::length(s.vel);
    if (i.terminalSpeed > 1e-9) {
        i.impactAngle = -std::asin(s.vel.y / i.terminalSpeed);
    }
    i.spin = s.omega.x;
    i.orientation = s.orientation;
    i.omega = s.omega;
    i.alpha = alpha(s, ctx);
}

} // namespace Ballistics::MM
