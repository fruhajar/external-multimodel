#ifndef BALLISTICS_MM_FORCES_H
#define BALLISTICS_MM_FORCES_H

#include "ballistics/mm/model.h"

namespace Ballistics::MM::Forces {

// Air state at a point, computed once per derivative and shared by every term.
struct AirState {
    glm::dvec3 velocityRel{0.0};
    double speed = 0.0;
    double mach = 0.0;
    double density = 0.0;
    double dynamicPressure = 0.0;   // q
    double altitudeMsl = 0.0;
    bool moving = false;            // false below the threshold where aerodynamics are dropped
};

AirState airState(const glm::dvec3& pos, const glm::dvec3& vel, const DerivativeContext& ctx);

glm::dvec3 gravity(double altitudeMsl);
glm::dvec3 coriolis(const glm::dvec3& vel, double latitudeRad, double azimuthRad);

// Drag along the relative wind. alpha feeds yaw drag for models that track it.
glm::dvec3 drag(const AirState& air, const DerivativeContext& ctx, double alpha);

glm::dvec3 thrust(const AirState& air, const DerivativeContext& ctx);

// Yaw of repose for a spin-stabilised body, the gyroscopic balance between the overturning
// moment and the rate at which the velocity vector is turning. Air density cancels out of the
// drift force this produces, which is why it is robust.
glm::dvec3 yawOfRepose(const AirState& air, const DerivativeContext& ctx, double spin);

// Normal force from an angle of attack, perpendicular to the relative wind, on the side the
// body axis is displaced towards. This is what turns a repose angle into lateral drift.
glm::dvec3 normalForce(const AirState& air, const DerivativeContext& ctx,
                       const glm::dvec3& alphaVector);

// Moments, all in the body frame. bodyAlpha is the angle-of-attack vector in body axes, with
// magnitude equal to the total angle of attack and no x component.
struct MomentInputs {
    glm::dvec3 bodyAlpha{0.0};
    glm::dvec3 bodyVelocityUnit{0.0};
    glm::dvec3 bodyOmega{0.0};
    double spin = 0.0;    // rad/s about the axis of symmetry
    double speed = 0.0;
    double q = 0.0;
};

// Static moment. The axis is perpendicular to the plane of attack, along v_hat x x_b, which is
// what makes it distinct from the Magnus moment lying within that plane.
glm::dvec3 overturningMoment(const MomentInputs& in, const Projectile& p);

// Magnus moment. The axis lies in the plane of attack, along the angle-of-attack direction,
// perpendicular to the overturning moment.
glm::dvec3 magnusMoment(const MomentInputs& in, const Projectile& p);

glm::dvec3 spinDampingMoment(const MomentInputs& in, const Projectile& p);
glm::dvec3 pitchDampingMoment(const MomentInputs& in, const Projectile& p);
glm::dvec3 finRollingMoment(const MomentInputs& in, const Projectile& p);

// Roll acceleration from spin damping and fin cant, for models carrying only a roll degree of
// freedom rather than full attitude.
double rollAcceleration(const AirState& air, const DerivativeContext& ctx, double spin);

} // namespace Ballistics::MM::Forces

#endif
