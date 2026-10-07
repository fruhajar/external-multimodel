// Adaptive stepping has to earn its place: equal or better accuracy at fewer derivative
// evaluations than the finest fixed step.

#include "ballistics/mm/ballistics.h"

#include "test_support.h"

using namespace Ballistics::MM;

namespace {

const Projectile& shell() {
    static const Catalogue c = Catalogue::withBuiltins();
    return *c.find("Howitzer155mm_HE");
}

struct Run {
    double range = 0.0;
    double tof = 0.0;
    long steps = 0;
};

Run measure(ModelKind model, const Projectile& p, const Environment& env, SolverConfig cfg,
            double yaw = 0.0) {
    Run out;
    cfg.diagnostics = [&out](const StepReport&) { ++out.steps; };

    LaunchState l = fromGround(p.muzzle.speed, 0.7853981633974483, 0.0, 0.0, p.muzzle.spinRate);
    l.initialYaw = yaw;

    const Result<Impact> r = predictImpact(model, l, p, env, GroundReference{0.0}, cfg);
    if (r) {
        out.range = r->range;
        out.tof = r->tof;
    }
    return out;
}

} // namespace

int main() {
    Environment env = Environment::standard();
    env.latitude = 50.0;
    const Projectile& p = shell();

    // A reference fine enough that both schemes are compared against something better.
    SolverConfig reference = SolverConfig::adaptive();
    reference.absTol = 1e-10;
    reference.relTol = 1e-11;
    const Run truth = measure(ModelKind::Mpmm, p, env, reference);
    CHECK(truth.range > 0.0);

    const Run fixed = measure(ModelKind::Mpmm, p, env, SolverConfig::precise());
    const Run adaptive = measure(ModelKind::Mpmm, p, env, SolverConfig::adaptive());

    CHECK(fixed.range > 0.0);
    CHECK(adaptive.range > 0.0);

    const double fixedError = std::abs(fixed.range - truth.range);
    const double adaptiveError = std::abs(adaptive.range - truth.range);

    std::printf("mpmm: fixed %ld steps, error %.4f m; adaptive %ld steps, error %.4f m\n",
                fixed.steps, fixedError, adaptive.steps, adaptiveError);

    CHECK(fixed.steps > 0 && adaptive.steps > 0);
    CHECK_MSG(adaptive.steps < fixed.steps,
              std::to_string(adaptive.steps) + " vs " + std::to_string(fixed.steps));
    CHECK_MSG(adaptiveError < 1.0, "adaptive error " + std::to_string(adaptiveError) + " m");

    // At the default tolerance the step is capped by dtMax rather than by accuracy: 781 steps
    // over a 78 s flight is exactly 78/0.1. So loosening the tolerance buys nothing, and the
    // meaningful direction is tightening it, which must cost steps.
    {
        SolverConfig tight = SolverConfig::adaptive();
        tight.absTol = 1e-12;
        tight.relTol = 1e-13;
        const Run strict = measure(ModelKind::Mpmm, p, env, tight);
        CHECK_MSG(strict.steps > adaptive.steps,
                  std::to_string(strict.steps) + " vs " + std::to_string(adaptive.steps));
        CHECK(std::abs(strict.range - truth.range) <= adaptiveError + 1e-9);

        // Raising dtMax with a tolerance that still binds keeps the answer while cutting steps.
        SolverConfig roomy = SolverConfig::adaptive();
        roomy.dtMax = 0.5;
        const Run fewer = measure(ModelKind::Mpmm, p, env, roomy);
        CHECK(fewer.steps < adaptive.steps);
        CHECK_REL(fewer.range, truth.range, 1e-5);
    }

    // The step must stay inside its bounds.
    {
        SolverConfig cfg = SolverConfig::adaptive();
        cfg.dtMax = 0.01;
        cfg.dtMin = 1e-5;
        double largest = 0.0;
        double smallest = 1e9;
        cfg.diagnostics = [&](const StepReport& s) {
            largest = std::max(largest, s.dt);
            smallest = std::min(smallest, s.dt);
        };
        const LaunchState l = fromGround(p.muzzle.speed, 0.7853981633974483, 0.0, 0.0,
                                         p.muzzle.spinRate);
        CHECK(static_cast<bool>(predictImpact(ModelKind::Mpmm, l, p, env, GroundReference{0.0},
                                             cfg)));
        CHECK(largest <= cfg.dtMax + 1e-12);
        CHECK(smallest >= cfg.dtMin - 1e-12);
    }

    // The same must hold for the rigid body, where the step is bounded by the spin as well.
    {
        const double spin = p.muzzle.spinRate * 2.0 * 3.14159265358979323846 / 60.0;

        SolverConfig fixedRigid = SolverConfig::forSpinRate(p.muzzle.spinRate);
        fixedRigid.dt *= 0.1;

        SolverConfig adaptiveRigid = SolverConfig::adaptive();
        adaptiveRigid.dtMax = SolverConfig::MAX_SPIN_PHASE_PER_STEP / spin;

        const Run rigidFixed = measure(ModelKind::RigidBody, p, env, fixedRigid, 0.002);
        const Run rigidAdaptive = measure(ModelKind::RigidBody, p, env, adaptiveRigid, 0.002);

        CHECK_MSG(rigidFixed.range > 0.0, "fixed rigid run failed");
        CHECK_MSG(rigidAdaptive.range > 0.0, "adaptive rigid run failed");

        std::printf("rigid body: fixed %ld steps, adaptive %ld steps, range %.1f vs %.1f m\n",
                    rigidFixed.steps, rigidAdaptive.steps, rigidFixed.range,
                    rigidAdaptive.range);

        CHECK(rigidAdaptive.steps < rigidFixed.steps);
        CHECK_REL(rigidAdaptive.range, rigidFixed.range, 1e-3);
    }

    return test::summary("adaptive");
}
