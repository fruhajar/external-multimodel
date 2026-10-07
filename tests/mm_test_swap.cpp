// Swapping any one piece must change the answer in the expected direction, and must not require
// a change anywhere else. That is the whole claim this directory makes over ballistics/pm.

#include "ballistics/mm/ballistics.h"

#include "test_support.h"

using namespace Ballistics::MM;

namespace {

const Projectile& shell() {
    static const Catalogue c = Catalogue::withBuiltins();
    return *c.find("Howitzer155mm_HE");
}

LaunchState at(double elevation, const Projectile& p) {
    return fromGround(p.muzzle.speed, elevation, 0.0, 0.0, p.muzzle.spinRate);
}

double rangeUnder(const Environment& env, const Projectile& p, double elevation = 0.7853981634) {
    const Result<Impact> r = predictImpact(ModelKind::Mpmm, at(elevation, p), p, env,
                                           GroundReference{0.0}, SolverConfig::standard());
    return r ? r->range : 0.0;
}

} // namespace

int main() {
    const Projectile& p = shell();
    const GroundReference flat{0.0};

    Environment base = Environment::standard();
    base.latitude = 50.0;
    const double reference = rangeUnder(base, p);
    CHECK(reference > 0.0);

    // Atmosphere: hot and thin air means less drag and more range; cold dense air the reverse.
    {
        const IsaStation hot(313.15, 95000.0, 0.0);
        const IsaStation cold(253.15, 105000.0, 0.0);
        CHECK(hot.density(0.0) < Isa::standard().density(0.0));
        CHECK(cold.density(0.0) > Isa::standard().density(0.0));

        Environment warm = base;
        warm.atmosphere = &hot;
        Environment chilly = base;
        chilly.atmosphere = &cold;

        const double warmRange = rangeUnder(warm, p);
        const double coldRange = rangeUnder(chilly, p);
        CHECK_MSG(warmRange > reference, std::to_string(warmRange) + " vs " +
                                             std::to_string(reference));
        CHECK_MSG(coldRange < reference, std::to_string(coldRange) + " vs " +
                                             std::to_string(reference));

        // Humidity thins the air further, so a humid day carries slightly further than a dry one
        // at the same temperature and pressure.
        const IsaStation dry(303.15, 101325.0, 0.0, 0.0);
        const IsaStation humid(303.15, 101325.0, 0.0, 1.0);
        CHECK(humid.density(0.0) < dry.density(0.0));
        CHECK(humid.speedOfSound(0.0) > dry.speedOfSound(0.0));

        Environment dryEnv = base;
        dryEnv.atmosphere = &dry;
        Environment humidEnv = base;
        humidEnv.atmosphere = &humid;
        CHECK(rangeUnder(humidEnv, p) > rangeUnder(dryEnv, p));
    }

    // Wind: a tailwind carries further, a headwind less, a crosswind pushes sideways.
    {
        const Uniform tail(15.0, 180.0);
        const Uniform head(15.0, 0.0);
        const Uniform cross(15.0, 270.0);

        Environment withTail = base;
        withTail.wind = &tail;
        Environment withHead = base;
        withHead.wind = &head;
        Environment withCross = base;
        withCross.wind = &cross;

        CHECK(rangeUnder(withTail, p) > reference);
        CHECK(rangeUnder(withHead, p) < reference);

        const Result<Impact> crossShot = predictImpact(ModelKind::Mpmm, at(0.7853981634, p), p,
                                                       withCross, flat);
        const Result<Impact> calmShot = predictImpact(ModelKind::Mpmm, at(0.7853981634, p), p,
                                                      base, flat);
        CHECK(static_cast<bool>(crossShot) && static_cast<bool>(calmShot));
        CHECK(crossShot->crossrange > calmShot->crossrange);
    }

    // A sounding with the same wind at every level must match a uniform field exactly, and one
    // that veers with height must not.
    {
        const Uniform uniform(12.0, 210.0);
        const Sounding flatProfile({{0.0, 12.0, 210.0}, {5000.0, 12.0, 210.0}});
        const Sounding veering({{0.0, 4.0, 180.0}, {2000.0, 14.0, 230.0}, {6000.0, 26.0, 280.0}});

        CHECK(flatProfile.levels() == 2);
        CHECK(veering.levels() == 3);

        Environment a = base;
        a.wind = &uniform;
        Environment b = base;
        b.wind = &flatProfile;
        Environment c = base;
        c.wind = &veering;

        CHECK_REL(rangeUnder(b, p), rangeUnder(a, p), 1e-12);
        CHECK(std::abs(rangeUnder(c, p) - rangeUnder(a, p)) > 1.0);

        // Interpolating speed and bearing keeps the speed through a veer; interpolating vectors
        // would lose it.
        for (double h = 0.0; h <= 6000.0; h += 250.0) {
            const double speed = glm::length(veering.at(h, 0.0));
            CHECK(speed >= 4.0 - 1e-9 && speed <= 26.0 + 1e-9);
        }
    }

    // Drag family: the same Cd at rest but a different transonic shape gives a different range.
    {
        Projectile g7 = p;
        g7.drag = StandardDrag::forCd(DragFamily::G7, p.drag->at(0.0));
        Projectile g1 = p;
        g1.drag = StandardDrag::forCd(DragFamily::G1, p.drag->at(0.0));

        CHECK_REL(g7.drag->at(0.0), p.drag->at(0.0), 1e-2);
        CHECK_REL(g1.drag->at(0.0), p.drag->at(0.0), 1e-9);

        // G7 is nearly flat through the transonic region where a shell's drag peaks, so it
        // under-predicts drag badly and over-predicts range.
        CHECK(rangeUnder(base, g7) > reference * 1.1);
        CHECK(std::abs(rangeUnder(base, g1) - reference) > 100.0);
    }

    // A measured table stands in for a family without touching anything else.
    {
        Projectile measured = p;
        measured.drag = std::make_shared<const CustomDrag>(
            std::vector<double>{0.0, 0.8, 1.0, 1.2, 2.0, 3.0},
            std::vector<double>{0.15, 0.16, 0.30, 0.38, 0.31, 0.26});
        const double range = rangeUnder(base, measured);
        CHECK(range > 0.0);
        CHECK(std::abs(range - reference) > 1.0);
    }

    // Yaw drag raises Cd with the square of the angle of attack, so it can only shorten a shot.
    {
        auto yawing = std::make_shared<StandardDrag>(DragFamily::Shell, 1.1302);
        yawing->setYawDrag(4.0);
        Projectile withYawDrag = p;
        withYawDrag.drag = yawing;

        CHECK_NEAR(yawing->at(1.5, 0.0), yawing->at(1.5), 1e-12);
        CHECK(yawing->at(1.5, 0.1) > yawing->at(1.5, 0.0));
        CHECK(rangeUnder(base, withYawDrag) <= reference);
    }

    // Integrator: RK2, RK4 and the adaptive scheme must agree, and refining must converge.
    {
        SolverConfig rk2 = SolverConfig::standard();
        rk2.integrator = Integrator::RK2;
        SolverConfig rk4 = SolverConfig::standard();
        SolverConfig fine = SolverConfig::precise();

        const auto rangeWith = [&](const SolverConfig& cfg) {
            const Result<Impact> r = predictImpact(ModelKind::Mpmm, at(0.7853981634, p), p, base,
                                                   flat, cfg);
            return r ? r->range : 0.0;
        };

        const double truth = rangeWith(SolverConfig::adaptive());
        CHECK(truth > 0.0);
        CHECK_REL(rangeWith(rk4), truth, 1e-4);
        CHECK_REL(rangeWith(rk2), truth, 1e-2);
        CHECK_REL(rangeWith(fine), truth, 1e-5);

        // RK4 must beat RK2 at the same step.
        CHECK(std::abs(rangeWith(rk4) - truth) < std::abs(rangeWith(rk2) - truth));
    }

    return test::summary("swap");
}
