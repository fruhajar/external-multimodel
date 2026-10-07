#include "ballistics/mm/solve.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Ballistics::MM {

namespace {

double horizontalMiss(const Impact& impact, const glm::dvec3& target) {
    const double dx = impact.range - target.x;
    const double dz = impact.crossrange - target.z;
    return std::sqrt(dx * dx + dz * dz);
}

double signedRangeError(const Impact& impact, const glm::dvec3& target) {
    const double reached = std::sqrt(impact.range * impact.range +
                                    impact.crossrange * impact.crossrange);
    const double wanted = std::sqrt(target.x * target.x + target.z * target.z);
    return reached - wanted;
}

constexpr double NEAR_VERTICAL = 1.5533430342749532;   // 89 degrees


// NoImpact is an ordinary outcome while sweeping elevations: some of them genuinely do not come
// down. Anything else means the request itself was wrong, and collapsing it into a range of zero
// would report a bad timestep or absent data as an unreachable target.
bool isConfigurationFailure(Status s) {
    return s != Status::Ok && s != Status::NoImpact;
}

} // namespace

Result<LaunchAim> solveLaunchDirection(ModelKind model, const LaunchState& from,
                                      const glm::dvec3& target, const Projectile& projectile,
                                      const Environment& env, double minElevation,
                                      double maxElevation, const SolverConfig& config) {
    const GroundReference ground{target.y};
    const double speed = glm::length(from.velocity) > 0.0 ? glm::length(from.velocity)
                                                          : projectile.muzzle.speed;
    if (speed <= 0.0 || minElevation >= maxElevation) {
        return Result<LaunchAim>::fail(Status::InvalidInput);
    }

    double azimuthOffset = 0.0;
    LaunchAim best;
    int totalIterations = 0;

    for (int azIter = 0; azIter < config.maxAzimuthIterations; ++azIter) {
        double low = minElevation;
        double high = maxElevation;

        const auto impactAt = [&](double elevation) {
            LaunchState trial = fromGround(speed, elevation, azimuthOffset, from.altitudeMsl,
                                           from.spinRate);
            trial.position = from.position;
            trial.initialYaw = from.initialYaw;
            return predictImpact(model, trial, projectile, env, ground, config);
        };

        const Result<Impact> atLow = impactAt(low);
        const Result<Impact> atHigh = impactAt(high);
        if (!atLow || !atHigh) {
            const Status why = isConfigurationFailure(atLow.status) ? atLow.status
                               : isConfigurationFailure(atHigh.status) ? atHigh.status
                                                                      : Status::Unreachable;
            return Result<LaunchAim>::fail(why);
        }

        double errLow = signedRangeError(*atLow, target);
        const double errHigh = signedRangeError(*atHigh, target);
        if (errLow * errHigh > 0.0) {
            return Result<LaunchAim>::fail(Status::Unreachable);
        }

        Impact converged{};
        double elevation = 0.0;
        double bestError = std::numeric_limits<double>::max();

        for (int i = 0; i < config.maxElevationIterations; ++i) {
            ++totalIterations;
            const double mid = 0.5 * (low + high);
            const Result<Impact> res = impactAt(mid);
            if (!res) {
                return Result<LaunchAim>::fail(Status::Unreachable);
            }

            const double err = signedRangeError(*res, target);
            if (std::abs(err) < std::abs(bestError)) {
                bestError = err;
                elevation = mid;
                converged = *res;
            }
            if (std::abs(err) < config.rangeTolerance) {
                break;
            }

            if (err * errLow < 0.0) {
                high = mid;
            } else {
                low = mid;
                errLow = err;
            }
            if (high - low < 1e-7) {
                break;
            }
        }

        if (std::abs(bestError) > config.rangeTolerance) {
            return Result<LaunchAim>::fail(Status::NotConverged);
        }

        best = {elevation, azimuthOffset, converged.tof, horizontalMiss(converged, target),
                totalIterations};

        // Crossrange comes from drift, wind and Coriolis, so the aim point is walked back
        // against the miss rather than solved in one pass.
        const double lateral = converged.crossrange - target.z;
        if (std::abs(lateral) <= config.crossrangeTolerance) {
            return Result<LaunchAim>::ok(best);
        }

        const double reached = std::sqrt(converged.range * converged.range +
                                        converged.crossrange * converged.crossrange);
        if (reached < 1e-6) {
            return Result<LaunchAim>::fail(Status::NotConverged);
        }
        azimuthOffset -= config.azimuthDamping *
                         std::asin(std::clamp(lateral / reached, -1.0, 1.0));
    }

    return {Status::NotConverged, best};
}

AimPair solveLaunchDirections(ModelKind model, const LaunchState& from, const glm::dvec3& target,
                              const Projectile& projectile, const Environment& env,
                              const SolverConfig& config) {
    const GroundReference ground{target.y};
    const Result<RangeEnvelope> envelope = maxRange(model, from, projectile, env, ground, config);
    if (!envelope) {
        return {Result<LaunchAim>::fail(envelope.status),
                Result<LaunchAim>::fail(envelope.status)};
    }
    return {solveLaunchDirection(model, from, target, projectile, env, 0.0, envelope->elevation,
                                 config),
            solveLaunchDirection(model, from, target, projectile, env, envelope->elevation,
                                 NEAR_VERTICAL, config)};
}

Result<RangeEnvelope> maxRange(ModelKind model, const LaunchState& from,
                               const Projectile& projectile, const Environment& env,
                               const GroundReference& ground, const SolverConfig& config) {
    const double speed = glm::length(from.velocity) > 0.0 ? glm::length(from.velocity)
                                                          : projectile.muzzle.speed;
    if (speed <= 0.0) {
        return Result<RangeEnvelope>::fail(Status::InvalidInput);
    }

    Status failure = Status::Ok;
    const auto rangeAt = [&](double elevation) {
        LaunchState trial = fromGround(speed, elevation, 0.0, from.altitudeMsl, from.spinRate);
        trial.position = from.position;
        trial.initialYaw = from.initialYaw;
        const Result<Impact> r = predictImpact(model, trial, projectile, env, ground, config);
        if (isConfigurationFailure(r.status)) {
            failure = r.status;
        }
        return r ? r->range : 0.0;
    };

    // Golden section over elevation. Firing downhill moves the optimum below 45 degrees.
    static const double RESPHI = 2.0 - (1.0 + std::sqrt(5.0)) * 0.5;
    const double terrain = std::atan2(ground.altitudeRel,
                                      std::max(1.0, std::abs(from.position.x) + 1000.0));
    const double guess = std::clamp(0.7853981633974483 - terrain * 0.5, 0.0, NEAR_VERTICAL);

    double a = std::max(0.0, guess - 0.4363323129985824);
    double b = std::min(NEAR_VERTICAL, guess + 0.4363323129985824);

    double x1 = a + RESPHI * (b - a);
    double x2 = b - RESPHI * (b - a);
    double f1 = rangeAt(x1);
    double f2 = rangeAt(x2);

    constexpr double TOLERANCE = 1e-4;   // radians
    while (std::abs(b - a) > TOLERANCE) {
        if (f1 > f2) {
            b = x2;
            x2 = x1;
            f2 = f1;
            x1 = a + RESPHI * (b - a);
            f1 = rangeAt(x1);
        } else {
            a = x1;
            x1 = x2;
            f1 = f2;
            x2 = b - RESPHI * (b - a);
            f2 = rangeAt(x2);
        }
    }

    const double elevation = 0.5 * (a + b);
    const double reach = rangeAt(elevation);
    if (failure != Status::Ok) {
        return Result<RangeEnvelope>::fail(failure);
    }
    if (reach <= 0.0) {
        return Result<RangeEnvelope>::fail(Status::NoImpact);
    }
    return Result<RangeEnvelope>::ok({elevation, reach});
}

std::vector<RangeTableRow> rangeTable(ModelKind model, const LaunchState& from,
                                     const Projectile& projectile, const Environment& env,
                                     const GroundReference& ground, double firstRange,
                                     double lastRange, double step, const SolverConfig& config) {
    std::vector<RangeTableRow> rows;
    if (step <= 0.0 || lastRange < firstRange) {
        return rows;
    }
    rows.reserve(static_cast<std::size_t>((lastRange - firstRange) / step) + 1);

    const Result<RangeEnvelope> envelope = maxRange(model, from, projectile, env, ground, config);
    if (!envelope) {
        return rows;
    }

    for (double range = firstRange; range <= lastRange + 1e-9; range += step) {
        const glm::dvec3 target(range, ground.altitudeRel, 0.0);
        const Result<LaunchAim> aim = solveLaunchDirection(model, from, target, projectile, env,
                                                          0.0, envelope->elevation, config);
        if (!aim) {
            continue;
        }

        LaunchState trial = fromGround(
            glm::length(from.velocity) > 0.0 ? glm::length(from.velocity)
                                             : projectile.muzzle.speed,
            aim->elevation, aim->azimuthOffset, from.altitudeMsl, from.spinRate);
        trial.position = from.position;
        trial.initialYaw = from.initialYaw;

        const Result<Impact> impact = predictImpact(model, trial, projectile, env, ground, config);
        if (!impact) {
            continue;
        }
        rows.push_back({range, aim->elevation, aim->azimuthOffset, impact->tof,
                        impact->terminalSpeed, impact->impactAngle});
    }
    return rows;
}

} // namespace Ballistics::MM
