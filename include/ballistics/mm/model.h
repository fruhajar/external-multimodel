#ifndef BALLISTICS_MM_MODEL_H
#define BALLISTICS_MM_MODEL_H

#include "environment.h"
#include "launch.h"
#include "projectile.h"
#include "types.h"

#include <concepts>

namespace Ballistics::MM {

enum class ModelKind {
    PointMass,   // 3 DOF
    Mpmm,        // 4 DOF, point mass plus roll, STANAG 4355
    RigidBody,   // 6 DOF
};

const char* describe(ModelKind k);
DataRequirements requirementsOf(ModelKind k);

// Everything a derivative needs besides the state. Adding humidity or a motor mass-flow term is
// one member here, not a new parameter on every force function.
struct DerivativeContext {
    const Projectile* projectile = nullptr;
    const Environment* env = nullptr;
    double tof = 0.0;
    double baseAltitude = 0.0;   // MSL altitude of local y == 0
    double groundRel = 0.0;      // ground plane in local y, for wind height above ground
};

// A model is a state type plus five operations. axpy is a free function per state so one
// integrator serves every model; the rigid-body overload renormalises the quaternion.
template <class M>
concept TrajectoryModel = requires(const typename M::State& s, const DerivativeContext& ctx,
                                   const LaunchState& launch, const Projectile& p,
                                   Trajectory& out, double h) {
    typename M::State;
    { M::requirements() } -> std::convertible_to<DataRequirements>;
    { M::initialState(launch, p) } -> std::convertible_to<typename M::State>;
    { M::derivative(s, ctx) } -> std::convertible_to<typename M::State>;
    { M::altitude(s) } -> std::convertible_to<double>;
    { M::position(s) } -> std::convertible_to<glm::dvec3>;
    { M::velocity(s) } -> std::convertible_to<glm::dvec3>;
    { M::spin(s) } -> std::convertible_to<double>;
    { M::alpha(s, ctx) } -> std::convertible_to<double>;
    { M::reserve(out, std::size_t{}) };
    { M::append(out, double{}, s, ctx) };
    { M::fillImpact(out.impact, s, ctx) };
    { axpy(s, s, h) } -> std::convertible_to<typename M::State>;
    { blend(s, s, h) } -> std::convertible_to<typename M::State>;
};

struct PointMassState {
    glm::dvec3 pos{0.0};
    glm::dvec3 vel{0.0};
};

// Point mass plus the roll degree of freedom, so spin is integrated rather than assumed.
struct MpmmState {
    glm::dvec3 pos{0.0};
    glm::dvec3 vel{0.0};
    double spin = 0.0;   // rad/s about the axis of symmetry
};

struct RigidBodyState {
    glm::dvec3 pos{0.0};
    glm::dvec3 vel{0.0};
    glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
    glm::dvec3 omega{0.0};   // body frame
};

// Purely linear: state + h * derivative. Nothing is renormalised here, because the integrator
// accumulates several weighted stages and normalising part way through would distort the sum.
PointMassState axpy(const PointMassState& s, const PointMassState& d, double h);
MpmmState axpy(const MpmmState& s, const MpmmState& d, double h);
RigidBodyState axpy(const RigidBodyState& s, const RigidBodyState& d, double h);

// Called once per completed step. Only the rigid body has anything to do: a linearly stepped
// quaternion leaves the unit sphere and stops being a rotation.
void renormalize(PointMassState&);
void renormalize(MpmmState&);
void renormalize(RigidBodyState& s);

// Linear interpolation between two states, for resolving the impact inside a step.
PointMassState blend(const PointMassState& a, const PointMassState& b, double f);
MpmmState blend(const MpmmState& a, const MpmmState& b, double f);
RigidBodyState blend(const RigidBodyState& a, const RigidBodyState& b, double f);

// Error norm between two states, for adaptive step control.
double errorNorm(const PointMassState& a, const PointMassState& b);
double errorNorm(const MpmmState& a, const MpmmState& b);
double errorNorm(const RigidBodyState& a, const RigidBodyState& b);

} // namespace Ballistics::MM

#endif
