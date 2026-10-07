#include "ballistics/mm/atmosphere.h"

#include "test_support.h"

using namespace Ballistics::MM;

int main() {
    const Atmosphere& isa = Isa::standard();

    CHECK_REL(isa.density(0.0), 1.2250, 1e-3);
    CHECK_REL(isa.density(5000.0), 0.73643, 2e-3);
    CHECK_REL(isa.density(11000.0), 0.36480, 3e-3);
    CHECK_REL(isa.density(20000.0), 0.08891, 1e-2);

    CHECK_REL(isa.speedOfSound(0.0), 340.29, 1e-3);
    CHECK_REL(isa.speedOfSound(11000.0), 295.07, 2e-3);

    CHECK_NEAR(isa.temperature(0.0), 288.15, 1e-9);
    CHECK_NEAR(isa.temperature(11000.0), 216.65, 1e-9);
    CHECK_NEAR(isa.temperature(20000.0), 216.65, 1e-9);

    double previous = isa.density(0.0);
    for (double h = 250.0; h <= 25000.0; h += 250.0) {
        const double d = isa.density(h);
        CHECK(d < previous);
        previous = d;
    }

    // A station reading reproduces itself at its own altitude.
    const IsaStation station(303.15, 95000.0, 500.0);
    CHECK_NEAR(station.temperature(500.0), 303.15, 1e-9);
    CHECK_REL(station.density(500.0), 95000.0 / (287.05 * 303.15), 1e-9);
    CHECK(station.density(500.0) < isa.density(500.0));

    // Standard conditions given as a station reading must reproduce the ISA.
    const IsaStation asStandard(288.15, 101325.0, 0.0);
    for (double h = 0.0; h <= 20000.0; h += 1000.0) {
        CHECK_REL(asStandard.density(h), isa.density(h), 1e-12);
        CHECK_REL(asStandard.speedOfSound(h), isa.speedOfSound(h), 1e-12);
    }

    // Humidity lowers density and raises the speed of sound, and saturated air more than dry.
    const IsaStation dry(303.15, 101325.0, 0.0, 0.0);
    const IsaStation half(303.15, 101325.0, 0.0, 0.5);
    const IsaStation saturated(303.15, 101325.0, 0.0, 1.0);

    CHECK(half.density(0.0) < dry.density(0.0));
    CHECK(saturated.density(0.0) < half.density(0.0));
    CHECK(saturated.speedOfSound(0.0) > dry.speedOfSound(0.0));

    // The effect is real but small: under 2% in density at 30 C and saturation.
    CHECK((dry.density(0.0) - saturated.density(0.0)) / dry.density(0.0) < 0.02);
    CHECK((dry.density(0.0) - saturated.density(0.0)) / dry.density(0.0) > 0.0);

    // Vapour thins with altitude rather than being carried up unchanged, so the humid and dry
    // profiles converge.
    const double lowGap = (dry.density(0.0) - saturated.density(0.0)) / dry.density(0.0);
    const double highGap = (dry.density(10000.0) - saturated.density(10000.0)) /
                           dry.density(10000.0);
    CHECK(highGap < lowGap);

    return test::summary("atmosphere");
}
