#include "ballistics/mm/ballistics.h"

#include "test_support.h"

using namespace Ballistics::MM;

namespace {

const Projectile& shell() {
    static const Catalogue c = Catalogue::withBuiltins();
    return *c.find("Howitzer155mm_HE");
}

Environment quiet() {
    Environment e = Environment::standard();
    e.latitude = 50.0;
    return e;
}

LaunchState at(double elevation, const Projectile& p, double yaw = 0.0) {
    LaunchState l = fromGround(p.muzzle.speed, elevation, 0.0, 0.0, p.muzzle.spinRate);
    l.initialYaw = yaw;
    return l;
}

} // namespace

int main() {
    const Environment env = quiet();
    const GroundReference flat{0.0};
    const Projectile& p = shell();
    const double elevation = 0.7853981633974483;

    // Each model declares what data it needs, and the entry point refuses rather than reading
    // absent blocks as zeros.
    {
        CHECK(requirementsOf(ModelKind::PointMass) ==
              (DataBlock::Geometry | DataBlock::Drag));
        CHECK(needs(requirementsOf(ModelKind::Mpmm), DataBlock::Inertia));
        CHECK(needs(requirementsOf(ModelKind::Mpmm), DataBlock::Moments));
        CHECK(needs(requirementsOf(ModelKind::RigidBody), DataBlock::Inertia));
        CHECK(!needs(requirementsOf(ModelKind::PointMass), DataBlock::Inertia));

        Projectile bare;
        bare.geometry = {46.5, 0.155, 0.857};
        bare.muzzle = {827.0, 16006.0};
        bare.drag = StandardDrag::forCd(DragFamily::Shell, 0.147);

        // Enough for a point mass, not for the other two.
        CHECK(static_cast<bool>(predictImpact(ModelKind::PointMass, at(elevation, bare), bare,
                                              env, flat)));

        const Result<Impact> mpmm = predictImpact(ModelKind::Mpmm, at(elevation, bare), bare,
                                                  env, flat);
        CHECK(!mpmm);
        CHECK(mpmm.status == Status::MissingData);

        const Result<MissingData> v = validate(bare, requirementsOf(ModelKind::Mpmm));
        CHECK(!v);
        CHECK(v->block == DataBlock::Inertia);

        // Adding inertia moves the complaint on to the next missing block.
        bare.inertia = Inertia{0.1554, 2.9158, 2.9158};
        const Result<MissingData> v2 = validate(bare, requirementsOf(ModelKind::Mpmm));
        CHECK(!v2);
        CHECK(v2->block == DataBlock::Moments);
    }

    // A rigid-body run is refused when the step cannot resolve the spin, rather than silently
    // returning a numerically tumbling round.
    {
        const Result<Impact> coarse = predictImpact(ModelKind::RigidBody, at(elevation, p, 0.002),
                                                    p, env, flat, SolverConfig::standard());
        CHECK(!coarse);
        CHECK_MSG(coarse.status == Status::StepTooLarge, describe(coarse.status));

        // The helper produces a step that is accepted.
        const SolverConfig ok = SolverConfig::forSpinRate(p.muzzle.spinRate);
        CHECK(ok.dt < SolverConfig::standard().dt);
        CHECK(static_cast<bool>(predictImpact(ModelKind::RigidBody, at(elevation, p, 0.002), p,
                                             env, flat, ok)));

        // The reason must survive the solvers. Collapsing a configuration failure into a range
        // of zero made maxRange report NoImpact and solveLaunchDirection report Unreachable,
        // both of which blame the target for a timestep the caller chose.
        const Result<RangeEnvelope> envelope = maxRange(ModelKind::RigidBody, at(0.0, p, 0.002),
                                                        p, env, flat, SolverConfig::standard());
        CHECK(!envelope);
        CHECK_MSG(envelope.status == Status::StepTooLarge, describe(envelope.status));

        const Result<LaunchAim> aim = solveLaunchDirection(
            ModelKind::RigidBody, at(0.0, p, 0.002), glm::dvec3(15000.0, 0.0, 0.0), p, env, 0.0,
            0.8, SolverConfig::standard());
        CHECK(!aim);
        CHECK_MSG(aim.status == Status::StepTooLarge, describe(aim.status));

        // A genuinely unreachable target still reports Unreachable, not a configuration failure.
        const Result<LaunchAim> tooFar = solveLaunchDirection(
            ModelKind::Mpmm, at(0.0, p), glm::dvec3(500000.0, 0.0, 0.0), p, env, 0.0, 1.5,
            SolverConfig::fast());
        CHECK(!tooFar);
        CHECK_MSG(tooFar.status == Status::Unreachable, describe(tooFar.status));

        // A round with no spin has nothing to resolve, so the coarse step is fine.
        Projectile finned = p;
        finned.muzzle.spinRate = 0.0;
        finned.moments->C_M_alpha = -10.0;
        CHECK(static_cast<bool>(predictImpact(ModelKind::RigidBody, at(elevation, finned), finned,
                                             env, flat, SolverConfig::standard())));
    }

    // The three models must agree on range and time of flight: they differ in what they say about
    // attitude and drift, not about where the round broadly goes.
    {
        SolverConfig rigid = SolverConfig::forSpinRate(p.muzzle.spinRate);
        rigid.dt *= 0.1;

        const Result<Impact> pointMass = predictImpact(ModelKind::PointMass, at(elevation, p), p,
                                                       env, flat, SolverConfig::precise());
        const Result<Impact> mpmm = predictImpact(ModelKind::Mpmm, at(elevation, p), p, env, flat,
                                                  SolverConfig::precise());
        const Result<Impact> rigidBody = predictImpact(ModelKind::RigidBody,
                                                        at(elevation, p, 0.002), p, env, flat,
                                                        rigid);
        CHECK_MSG(static_cast<bool>(pointMass), describe(pointMass.status));
        CHECK_MSG(static_cast<bool>(mpmm), describe(mpmm.status));
        CHECK_MSG(static_cast<bool>(rigidBody), describe(rigidBody.status));

        CHECK_REL(mpmm->range, pointMass->range, 5e-3);
        CHECK_REL(rigidBody->range, mpmm->range, 5e-3);
        CHECK_REL(rigidBody->tof, mpmm->tof, 5e-3);

        // Right-hand spin drifts right, and only the models carrying spin produce it.
        CHECK(mpmm->crossrange > 0.0);
        CHECK(rigidBody->crossrange > 0.0);
        CHECK(mpmm->crossrange > 10.0 * std::abs(pointMass->crossrange));

        // MPMM solves the yaw of repose analytically; the rigid body integrates attitude and the
        // repose emerges. That they land within a few percent is the point of having both.
        CHECK_MSG(std::abs(rigidBody->crossrange - mpmm->crossrange) <
                      0.05 * mpmm->crossrange,
                  "mpmm " + std::to_string(mpmm->crossrange) + " rigid " +
                      std::to_string(rigidBody->crossrange));

        // MPMM must be closer to the rigid body than a point mass is, or it earns nothing.
        CHECK(std::abs(mpmm->crossrange - rigidBody->crossrange) <
              std::abs(pointMass->crossrange - rigidBody->crossrange));
    }

    // Spin is integrated, not assumed: it decays, and the decay is what roll damping says.
    {
        SolverConfig cfg = SolverConfig::precise();
        cfg.storeTrajectory = true;
        const Result<Trajectory> t = predict(ModelKind::Mpmm, at(elevation, p), p, env, flat, cfg);
        CHECK(static_cast<bool>(t));
        CHECK(t->hasSpin());
        CHECK(!t->hasAttitude());

        const double initial = p.muzzle.spinRate * 2.0 * 3.14159265358979323846 / 60.0;
        CHECK_REL(t->spin.front(), initial, 1e-9);
        CHECK(t->spin.back() < t->spin.front());
        CHECK(t->spin.back() > 0.5 * t->spin.front());

        for (std::size_t i = 1; i < t->spin.size(); ++i) {
            CHECK(t->spin[i] <= t->spin[i - 1] + 1e-9);
        }

        // The repose angle grows as the round slows, because it goes as 1/v^3.
        CHECK(t->alpha.back() > t->alpha.front());
        CHECK(glm::degrees(t->alpha.back()) < 5.0);
    }

    // A point mass fills no attitude columns at all.
    {
        SolverConfig cfg = SolverConfig::standard();
        cfg.storeTrajectory = true;
        const Result<Trajectory> t = predict(ModelKind::PointMass, at(elevation, p), p, env, flat,
                                             cfg);
        CHECK(static_cast<bool>(t));
        CHECK(!t->hasAttitude());
        CHECK(!t->hasSpin());
        CHECK(t->size() > 100);
    }

    // Gyroscopic stability is reported, and the catalogue's spun rounds are stable.
    {
        CHECK(gyroscopicStability(p) > 1.0);
        CHECK(gyroscopicStability(p) < 3.0);

        Projectile unstable = p;
        unstable.moments->C_M_alpha = 20.0;
        CHECK(gyroscopicStability(unstable) < 1.0);

        Projectile noData = p;
        noData.inertia.reset();
        CHECK_NEAR(gyroscopicStability(noData), 0.0, 1e-12);
    }

    return test::summary("models");
}
