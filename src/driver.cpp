#include "driver.h"

#include "constants.h"
#include "forces.h"

#include <algorithm>

namespace Ballistics::MM {

namespace {

// Dormand-Prince 5(4) coefficients. The fifth-order result is propagated and the fourth-order
// one is used only to size the next step.
constexpr double A21 = 1.0 / 5.0;
constexpr double A31 = 3.0 / 40.0, A32 = 9.0 / 40.0;
constexpr double A41 = 44.0 / 45.0, A42 = -56.0 / 15.0, A43 = 32.0 / 9.0;
constexpr double A51 = 19372.0 / 6561.0, A52 = -25360.0 / 2187.0, A53 = 64448.0 / 6561.0,
                 A54 = -212.0 / 729.0;
constexpr double A61 = 9017.0 / 3168.0, A62 = -355.0 / 33.0, A63 = 46732.0 / 5247.0,
                 A64 = 49.0 / 176.0, A65 = -5103.0 / 18656.0;
constexpr double B1 = 35.0 / 384.0, B3 = 500.0 / 1113.0, B4 = 125.0 / 192.0,
                 B5 = -2187.0 / 6784.0, B6 = 11.0 / 84.0;
constexpr double C1 = 5179.0 / 57600.0, C3 = 7571.0 / 16695.0, C4 = 393.0 / 640.0,
                 C5 = -92097.0 / 339200.0, C6 = 187.0 / 2100.0, C7 = 1.0 / 40.0;

template <class State>
State combine(const State& base, std::initializer_list<std::pair<const State*, double>> terms) {
    State out = base;
    for (const auto& [d, weight] : terms) {
        out = axpy(out, *d, weight);
    }
    return out;
}

} // namespace

template <TrajectoryModel M>
Result<Trajectory> integrate(const LaunchState& launch, const Projectile& projectile,
                             const Environment& env, const GroundReference& ground,
                             const SolverConfig& config, bool store) {
    using State = typename M::State;

    if (!env.valid()) {
        return Result<Trajectory>::fail(Status::InvalidInput);
    }
    if (const Result<MissingData> v = validate(projectile, M::requirements()); !v) {
        return Result<Trajectory>::fail(v.status);
    }
    if (config.dt <= 0.0 || config.maxFlightTime <= 0.0) {
        return Result<Trajectory>::fail(Status::InvalidInput);
    }

    DerivativeContext ctx;
    ctx.projectile = &projectile;
    ctx.env = &env;
    ctx.baseAltitude = launch.altitudeMsl - launch.position.y;
    ctx.groundRel = ground.altitudeRel;

    Trajectory out;
    if (store) {
        const double span = config.sampleInterval > 0.0 ? config.sampleInterval : config.dt;
        M::reserve(out, static_cast<std::size_t>(config.maxFlightTime / span) + 2);
    }

    const bool adaptive = config.integrator == Integrator::DOPRI5 &&
                          (config.absTol > 0.0 || config.relTol > 0.0);

    // A model that integrates attitude has to resolve the rotation. Refuse rather than return a
    // numerically tumbling round that looks like a real answer.
    if constexpr (M::tracksAttitude) {
        const double spin = std::abs(launch.spinRate) * 2.0 * Constants::PI / 60.0;
        const double step = adaptive ? config.dtMax : config.dt;
        if (spin * step > SolverConfig::MAX_SPIN_PHASE_PER_STEP * 1.000001) {
            return Result<Trajectory>::fail(Status::StepTooLarge);
        }
    }

    State s = M::initialState(launch, projectile);
    double t = 0.0;
    double dt = adaptive ? std::clamp(config.dt, config.dtMin, config.dtMax) : config.dt;
    double nextSample = 0.0;

    const auto derivativeAt = [&](const State& state, double time) {
        ctx.tof = time;
        return M::derivative(state, ctx);
    };

    while (t < config.maxFlightTime) {
        ctx.tof = t;

        if (store && t >= nextSample) {
            M::append(out, t, s, ctx);
            nextSample += config.sampleInterval;
        }

        const State previous = s;
        const double previousTime = t;
        State next;
        double appliedDt = dt;

        if (adaptive) {
            // Retry the step until the embedded error estimate is inside tolerance.
            for (int attempt = 0; attempt < 24; ++attempt) {
                const State k1 = derivativeAt(s, t);
                const State k2 = derivativeAt(axpy(s, k1, A21 * dt), t + 0.2 * dt);
                const State s3 = combine(s, {{&k1, A31 * dt}, {&k2, A32 * dt}});
                const State k3 = derivativeAt(s3, t + 0.3 * dt);
                const State s4 = combine(s, {{&k1, A41 * dt}, {&k2, A42 * dt}, {&k3, A43 * dt}});
                const State k4 = derivativeAt(s4, t + 0.8 * dt);
                const State s5 = combine(s, {{&k1, A51 * dt}, {&k2, A52 * dt}, {&k3, A53 * dt},
                                             {&k4, A54 * dt}});
                const State k5 = derivativeAt(s5, t + (8.0 / 9.0) * dt);
                const State s6 = combine(s, {{&k1, A61 * dt}, {&k2, A62 * dt}, {&k3, A63 * dt},
                                             {&k4, A64 * dt}, {&k5, A65 * dt}});
                const State k6 = derivativeAt(s6, t + dt);

                const State fifth = combine(s, {{&k1, B1 * dt}, {&k3, B3 * dt}, {&k4, B4 * dt},
                                                {&k5, B5 * dt}, {&k6, B6 * dt}});
                const State k7 = derivativeAt(fifth, t + dt);
                const State fourth = combine(s, {{&k1, C1 * dt}, {&k3, C3 * dt}, {&k4, C4 * dt},
                                                 {&k5, C5 * dt}, {&k6, C6 * dt}, {&k7, C7 * dt}});

                const double error = errorNorm(fifth, fourth);
                const double tolerance = config.absTol + config.relTol * 1.0;

                if (error <= tolerance || dt <= config.dtMin * 1.0000001) {
                    next = fifth;
                    appliedDt = dt;
                    // Grow the next step, capped so it cannot leap past a feature.
                    const double growth = error > 0.0
                        ? std::clamp(0.9 * std::pow(tolerance / error, 0.2), 0.2, 4.0)
                        : 4.0;
                    dt = std::clamp(dt * growth, config.dtMin, config.dtMax);
                    break;
                }
                dt = std::clamp(dt * std::max(0.2, 0.9 * std::pow(tolerance / error, 0.25)),
                                config.dtMin, config.dtMax);
            }
        } else if (config.integrator == Integrator::RK2) {
            const State k1 = derivativeAt(s, t);
            const State k2 = derivativeAt(axpy(s, k1, dt * 0.5), t + dt * 0.5);
            next = axpy(s, k2, dt);
        } else {
            const State k1 = derivativeAt(s, t);
            const State k2 = derivativeAt(axpy(s, k1, dt * 0.5), t + dt * 0.5);
            const State k3 = derivativeAt(axpy(s, k2, dt * 0.5), t + dt * 0.5);
            const State k4 = derivativeAt(axpy(s, k3, dt), t + dt);
            next = combine(s, {{&k1, dt / 6.0}, {&k2, dt / 3.0}, {&k3, dt / 3.0}, {&k4, dt / 6.0}});
        }

        s = next;
        renormalize(s);
        t += appliedDt;

        if (config.diagnostics) {
            ctx.tof = t;
            StepReport report;
            report.t = t;
            report.dt = appliedDt;
            report.speed = glm::length(M::velocity(s));
            report.altitude = M::altitude(s);
            report.alpha = M::alpha(s, ctx);
            report.spin = M::spin(s);
            if (projectile.inertia) {
                report.rotationalEnergy = 0.5 * projectile.inertia->Ixx * report.spin *
                                          report.spin;
            }
            config.diagnostics(report);
        }

        // Descending through the ground plane is the only impact condition, so a vertical drop
        // registers the same way a lobbed shot does.
        const bool descending = M::velocity(s).y < 0.0;
        const bool crossed = M::altitude(previous) >= ground.altitudeRel &&
                             M::altitude(s) < ground.altitudeRel;

        if (descending && crossed) {
            const double drop = M::altitude(s) - M::altitude(previous);
            double fraction = 0.0;
            if (std::abs(drop) > 1e-12) {
                fraction = (ground.altitudeRel - M::altitude(previous)) / drop;
            }

            State at = blend(previous, s, fraction);
            renormalize(at);
            const double impactTime = previousTime + fraction * appliedDt;
            ctx.tof = impactTime;

            out.impact.tof = impactTime;
            out.impact.range = M::position(at).x;
            out.impact.crossrange = M::position(at).z;
            out.impact.altitudeRel = M::altitude(at);
            M::fillImpact(out.impact, at, ctx);

            if (store) {
                M::append(out, impactTime, at, ctx);
            }
            return Result<Trajectory>::ok(std::move(out));
        }

        if (glm::length(M::velocity(s)) < 0.01) {
            break;
        }
    }

    return Result<Trajectory>::fail(Status::NoImpact);
}

// Explicit instantiations: the template stays inside the library, and the public entry points
// in predict.cpp are plain functions.
template Result<Trajectory> integrate<PointMass>(const LaunchState&, const Projectile&,
                                                 const Environment&, const GroundReference&,
                                                 const SolverConfig&, bool);
template Result<Trajectory> integrate<Mpmm>(const LaunchState&, const Projectile&,
                                            const Environment&, const GroundReference&,
                                            const SolverConfig&, bool);
template Result<Trajectory> integrate<RigidBody>(const LaunchState&, const Projectile&,
                                                 const Environment&, const GroundReference&,
                                                 const SolverConfig&, bool);

} // namespace Ballistics::MM
