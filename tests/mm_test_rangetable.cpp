// A range table is the concrete mid-range deliverable the solver seam exists to produce.

#include "ballistics/mm/ballistics.h"

#include "test_support.h"

using namespace Ballistics::MM;

int main() {
    const Catalogue c = Catalogue::withBuiltins();
    const Projectile* p = c.find("Howitzer155mm_HE");

    Environment env = Environment::standard();
    env.latitude = 50.0;

    const GroundReference flat{0.0};
    const LaunchState gun = fromGround(p->muzzle.speed, 0.0, 0.0, 0.0, p->muzzle.spinRate);

    SolverConfig cfg = SolverConfig::fast();
    cfg.rangeTolerance = 15.0;
    cfg.crossrangeTolerance = 5.0;

    const std::vector<RangeTableRow> rows =
        rangeTable(ModelKind::Mpmm, gun, *p, env, flat, 5000.0, 20000.0, 2500.0, cfg);

    CHECK_MSG(rows.size() == 7, "rows " + std::to_string(rows.size()));
    if (rows.empty()) {
        return test::summary("rangetable");
    }

    std::printf("%9s %10s %9s %7s %8s %9s\n",
                "range", "elevation", "aim off", "tof", "term v", "impact");
    for (const RangeTableRow& r : rows) {
        std::printf("%9.0f %9.2f d %8.3f d %6.1f %8.0f %8.1f d\n",
                    r.range, glm::degrees(r.elevation), glm::degrees(r.azimuthOffset), r.tof,
                    r.terminalSpeed, glm::degrees(r.impactAngle));
    }

    // Monotonic in every column that has to be: further means more elevation, longer flight,
    // steeper arrival, and a larger left correction against the rightward drift.
    for (std::size_t i = 1; i < rows.size(); ++i) {
        CHECK(rows[i].range > rows[i - 1].range);
        CHECK(rows[i].elevation > rows[i - 1].elevation);
        CHECK(rows[i].tof > rows[i - 1].tof);
        CHECK(rows[i].impactAngle > rows[i - 1].impactAngle);
        CHECK(rows[i].azimuthOffset < rows[i - 1].azimuthOffset);
    }

    // All below the range-maximising elevation, since that is the bracket used.
    const Result<RangeEnvelope> envelope = maxRange(ModelKind::Mpmm, gun, *p, env, flat, cfg);
    CHECK(static_cast<bool>(envelope));
    for (const RangeTableRow& r : rows) {
        CHECK(r.elevation < envelope->elevation);
        CHECK(r.terminalSpeed > 0.0);
        CHECK(r.azimuthOffset < 0.0);   // aim left of a target that the drift carries right of
    }

    // Each row must actually hit what it claims.
    for (const RangeTableRow& r : rows) {
        LaunchState l = fromGround(p->muzzle.speed, r.elevation, r.azimuthOffset, 0.0,
                                   p->muzzle.spinRate);
        const Result<Impact> impact = predictImpact(ModelKind::Mpmm, l, *p, env, flat, cfg);
        CHECK(static_cast<bool>(impact));
        if (impact) {
            const double reached = std::sqrt(impact->range * impact->range +
                                            impact->crossrange * impact->crossrange);
            CHECK_NEAR(reached, r.range, cfg.rangeTolerance);
            CHECK_NEAR(impact->crossrange, 0.0, cfg.crossrangeTolerance);
        }
    }

    // An empty or reversed sweep produces nothing rather than misbehaving.
    CHECK(rangeTable(ModelKind::Mpmm, gun, *p, env, flat, 5000.0, 4000.0, 500.0, cfg).empty());
    CHECK(rangeTable(ModelKind::Mpmm, gun, *p, env, flat, 5000.0, 9000.0, 0.0, cfg).empty());

    return test::summary("rangetable");
}
