#include "ballistics/mm/ballistics.h"

#include "test_support.h"

using namespace Ballistics::MM;

int main() {
    const Catalogue builtins = Catalogue::withBuiltins();
    CHECK(builtins.size() == 9);
    CHECK(builtins.find("no such round") == nullptr);

    int spun = 0;
    for (const std::string& id : builtins.ids()) {
        const Projectile* p = builtins.find(id);
        CHECK_MSG(p != nullptr, id);

        // Every entry must satisfy all three models' data requirements: that is the point of
        // carrying the inertia and moment blocks here rather than only in the 3 DOF library.
        for (ModelKind kind : {ModelKind::PointMass, ModelKind::Mpmm, ModelKind::RigidBody}) {
            const Result<MissingData> v = validate(*p, requirementsOf(kind));
            CHECK_MSG(static_cast<bool>(v),
                      id + " for " + describe(kind) + ": missing " + describe(v->block));
        }

        CHECK_MSG(p->inertia->Ixx > 0.0, id);
        CHECK_MSG(p->inertia->Iyy > p->inertia->Ixx, id);   // long and thin, not a disc
        CHECK_MSG(p->inertia->Iyy == p->inertia->Izz, id);  // axisymmetric

        // Sign convention: overturning is positive, so a spin-stabilised round is positive and
        // a fin-stabilised one negative, since a positive static moment means the round is
        // held only by its spin.
        if (p->stabilisation == Stabilisation::Spin) {
            ++spun;
            CHECK_MSG(p->moments->C_M_alpha > 0.0, id);
            CHECK_MSG(p->muzzle.spinRate > 0.0, id);

            const double sg = gyroscopicStability(*p);
            CHECK_MSG(sg > 1.0 && sg < 3.0, id + " Sg " + std::to_string(sg));
        } else {
            CHECK_MSG(p->moments->C_M_alpha < 0.0, id);
            CHECK_MSG(p->muzzle.spinRate == 0.0, id);
        }

        // Damping coefficients must damp.
        CHECK_MSG(p->moments->C_M_q < 0.0, id);
        CHECK_MSG(p->moments->C_l_p < 0.0, id);
    }

    // Only the rifled 155mm family is spin-stabilised; mortars and the 120mm smoothbore rounds
    // are fin-stabilised.
    CHECK(spun == 2);
    CHECK(builtins.find("Howitzer155mm_HE")->stabilisation == Stabilisation::Spin);
    
    CHECK(builtins.find("Mortar120mm_HE")->stabilisation == Stabilisation::Fin);

    // Axial inertia from the measured M107 ratio, above what a uniform cylinder gives.
    {
        const Projectile* p = builtins.find("Howitzer155mm_HE");
        const double uniformCylinder = 0.5 * p->geometry.mass *
                                      (p->geometry.diameter / 2.0) * (p->geometry.diameter / 2.0);
        CHECK(p->inertia->Ixx > uniformCylinder);
        CHECK_REL(p->inertia->Ixx,
                  0.1391 * p->geometry.mass * p->geometry.diameter * p->geometry.diameter, 1e-3);
    }

    // Calibrated rounds reproduce the published maximum range they were fitted to.
    {
        struct Anchor { const char* id; double range; };
        const Anchor anchors[] = {
            {"Mortar60mm_HE", 3490.0},
            {"Mortar60mm_SMK", 3200.0},
            {"Mortar81mm_HE", 5650.0},
            {"Mortar81mm_SMK", 4900.0},
            {"Mortar81mm_ILL", 5050.0},
            {"Mortar120mm_HE", 7200.0},
            {"Mortar120mm_SMK", 7200.0},
            {"Howitzer155mm_HE", 22400.0},
            {"Howitzer155mm_SMK", 18000.0},
        };

        Environment env = Environment::standard();
        env.latitude = 45.0;

        for (const Anchor& a : anchors) {
            const Projectile* p = builtins.find(a.id);
            CHECK_MSG(p->quality == DataQuality::Calibrated, a.id);

            // Drag and gravity only, as the fit was made.
            Projectile bare = *p;
            bare.muzzle.spinRate = 0.0;
            const Result<RangeEnvelope> e = maxRange(
                ModelKind::PointMass, fromGround(p->muzzle.speed, 0.0, 0.0, 0.0), bare, env,
                GroundReference{0.0});
            CHECK_MSG(static_cast<bool>(e), a.id);
            if (e) {
                CHECK_MSG(std::abs(e->range - a.range) < 0.01 * a.range,
                          std::string(a.id) + " " +
                              test::values(e->range, a.range, 0.01 * a.range));
            }
        }
    }

    // Callers can extend and shrink the set.
    Catalogue mine = builtins;
    Projectile custom;
    custom.id = "custom";
    custom.geometry = {5.0, 0.08, 0.4};
    custom.muzzle = {300.0, 0.0};
    custom.drag = StandardDrag::forCd(DragFamily::G7, 0.3);
    mine.add(std::move(custom));

    CHECK(mine.size() == 10);
    CHECK(builtins.size() == 9);
    CHECK(mine.remove("custom"));
    CHECK(!mine.remove("custom"));

    return test::summary("catalogue");
}
