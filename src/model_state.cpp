#include "ballistics/mm/model.h"

#include <algorithm>

namespace Ballistics::MM {

const char* describe(ModelKind k) {
    switch (k) {
    case ModelKind::PointMass: return "point mass (3 DOF)";
    case ModelKind::Mpmm:      return "MPMM (4 DOF)";
    case ModelKind::RigidBody: return "rigid body (6 DOF)";
    }
    return "unknown";
}

PointMassState axpy(const PointMassState& s, const PointMassState& d, double h) {
    return {s.pos + d.pos * h, s.vel + d.vel * h};
}

MpmmState axpy(const MpmmState& s, const MpmmState& d, double h) {
    return {s.pos + d.pos * h, s.vel + d.vel * h, s.spin + d.spin * h};
}

RigidBodyState axpy(const RigidBodyState& s, const RigidBodyState& d, double h) {
    RigidBodyState r;
    r.pos = s.pos + d.pos * h;
    r.vel = s.vel + d.vel * h;
    r.omega = s.omega + d.omega * h;
    r.orientation = s.orientation + d.orientation * h;
    return r;
}

void renormalize(PointMassState&) {}
void renormalize(MpmmState&) {}

void renormalize(RigidBodyState& s) {
    s.orientation = glm::normalize(s.orientation);
}

PointMassState blend(const PointMassState& a, const PointMassState& b, double f) {
    return {a.pos + f * (b.pos - a.pos), a.vel + f * (b.vel - a.vel)};
}

MpmmState blend(const MpmmState& a, const MpmmState& b, double f) {
    return {a.pos + f * (b.pos - a.pos), a.vel + f * (b.vel - a.vel),
            a.spin + f * (b.spin - a.spin)};
}

RigidBodyState blend(const RigidBodyState& a, const RigidBodyState& b, double f) {
    RigidBodyState r;
    r.pos = a.pos + f * (b.pos - a.pos);
    r.vel = a.vel + f * (b.vel - a.vel);
    r.omega = a.omega + f * (b.omega - a.omega);
    r.orientation = glm::slerp(a.orientation, b.orientation, f);
    return r;
}

namespace {

// Scaled to the state's own magnitude so position and velocity contribute comparably.
double relative(const glm::dvec3& diff, const glm::dvec3& scale) {
    const double magnitude = glm::length(scale);
    return glm::length(diff) / (magnitude > 1.0 ? magnitude : 1.0);
}

} // namespace

double errorNorm(const PointMassState& a, const PointMassState& b) {
    return std::max(relative(a.pos - b.pos, a.pos), relative(a.vel - b.vel, a.vel));
}

double errorNorm(const MpmmState& a, const MpmmState& b) {
    const double spinScale = std::abs(a.spin) > 1.0 ? std::abs(a.spin) : 1.0;
    return std::max({relative(a.pos - b.pos, a.pos), relative(a.vel - b.vel, a.vel),
                     std::abs(a.spin - b.spin) / spinScale});
}

double errorNorm(const RigidBodyState& a, const RigidBodyState& b) {
    const glm::dquat dq = a.orientation - b.orientation;
    const double quatError = std::sqrt(dq.w * dq.w + dq.x * dq.x + dq.y * dq.y + dq.z * dq.z);
    return std::max({relative(a.pos - b.pos, a.pos), relative(a.vel - b.vel, a.vel),
                     relative(a.omega - b.omega, a.omega), quatError});
}

} // namespace Ballistics::MM
