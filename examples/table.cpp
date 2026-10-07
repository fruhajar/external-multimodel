// A range table under two met conditions, which is what the solver seam is for.

#include "ballistics/mm/ballistics.h"

#include <cstdio>

using namespace Ballistics::MM;

int main() {
    const Catalogue c = Catalogue::withBuiltins();
    const Projectile* shell = c.find("Howitzer155mm_HE");
    if (shell == nullptr) {
        return 1;
    }

    const GroundReference flat{0.0};
    const LaunchState gun = fromGround(shell->muzzle.speed, 0.0, 0.0, 0.0,
                                       shell->muzzle.spinRate);

    SolverConfig cfg = SolverConfig::fast();
    cfg.rangeTolerance = 15.0;
    cfg.crossrangeTolerance = 5.0;

    Environment standard = Environment::standard();
    standard.latitude = 50.0;

    // A cold, dense, windy day.
    const IsaStation cold(263.15, 103000.0, 200.0, 0.8);
    const Sounding wind({{0.0, 6.0, 250.0}, {2000.0, 16.0, 270.0}, {6000.0, 28.0, 290.0}});
    Environment rough = standard;
    rough.atmosphere = &cold;
    rough.wind = &wind;

    const auto rows = rangeTable(ModelKind::Mpmm, gun, *shell, standard, flat,
                                6000.0, 18000.0, 3000.0, cfg);
    const auto roughRows = rangeTable(ModelKind::Mpmm, gun, *shell, rough, flat,
                                     6000.0, 18000.0, 3000.0, cfg);

    std::printf("%-9s %-24s %-24s\n", "", "standard day", "cold, dense, windy");
    std::printf("%-9s %10s %12s %10s %12s\n", "range", "elevation", "aim off", "elevation",
                "aim off");

    for (std::size_t i = 0; i < rows.size() && i < roughRows.size(); ++i) {
        std::printf("%9.0f %9.2f d %10.3f d %9.2f d %10.3f d\n", rows[i].range,
                    glm::degrees(rows[i].elevation), glm::degrees(rows[i].azimuthOffset),
                    glm::degrees(roughRows[i].elevation),
                    glm::degrees(roughRows[i].azimuthOffset));
    }
    return 0;
}
