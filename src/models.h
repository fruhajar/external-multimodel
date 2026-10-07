#ifndef BALLISTICS_MM_MODELS_H
#define BALLISTICS_MM_MODELS_H

#include "ballistics/mm/model.h"

namespace Ballistics::MM {

// Three degrees of freedom. No attitude, so no drift term: a point mass has no yaw of repose.
struct PointMass {
    using State = PointMassState;
    static constexpr bool tracksAttitude = false;

    static DataRequirements requirements();
    static State initialState(const LaunchState& launch, const Projectile& p);
    static State derivative(const State& s, const DerivativeContext& ctx);
    static double altitude(const State& s) { return s.pos.y; }
    static glm::dvec3 position(const State& s) { return s.pos; }
    static glm::dvec3 velocity(const State& s) { return s.vel; }
    static double spin(const State&) { return 0.0; }
    static double alpha(const State&, const DerivativeContext&) { return 0.0; }
    static void reserve(Trajectory& out, std::size_t n);
    static void append(Trajectory& out, double t, const State& s, const DerivativeContext& ctx);
    static void fillImpact(Impact& i, const State& s, const DerivativeContext& ctx);
};

// Four degrees of freedom: point mass plus roll, STANAG 4355. Spin is integrated, and the yaw of
// repose it sustains is solved analytically rather than being carried as state.
struct Mpmm {
    using State = MpmmState;
    static constexpr bool tracksAttitude = false;

    static DataRequirements requirements();
    static State initialState(const LaunchState& launch, const Projectile& p);
    static State derivative(const State& s, const DerivativeContext& ctx);
    static double altitude(const State& s) { return s.pos.y; }
    static glm::dvec3 position(const State& s) { return s.pos; }
    static glm::dvec3 velocity(const State& s) { return s.vel; }
    static double spin(const State& s) { return s.spin; }
    static double alpha(const State& s, const DerivativeContext& ctx);
    static void reserve(Trajectory& out, std::size_t n);
    static void append(Trajectory& out, double t, const State& s, const DerivativeContext& ctx);
    static void fillImpact(Impact& i, const State& s, const DerivativeContext& ctx);
};

// Six degrees of freedom. Attitude is integrated, so the yaw of repose emerges from the moment
// balance rather than being imposed.
struct RigidBody {
    using State = RigidBodyState;
    static constexpr bool tracksAttitude = true;

    static DataRequirements requirements();
    static State initialState(const LaunchState& launch, const Projectile& p);
    static State derivative(const State& s, const DerivativeContext& ctx);
    static double altitude(const State& s) { return s.pos.y; }
    static glm::dvec3 position(const State& s) { return s.pos; }
    static glm::dvec3 velocity(const State& s) { return s.vel; }
    static double spin(const State& s) { return s.omega.x; }
    static double alpha(const State& s, const DerivativeContext& ctx);
    static void reserve(Trajectory& out, std::size_t n);
    static void append(Trajectory& out, double t, const State& s, const DerivativeContext& ctx);
    static void fillImpact(Impact& i, const State& s, const DerivativeContext& ctx);
};

} // namespace Ballistics::MM

#endif
