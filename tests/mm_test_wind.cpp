#include "ballistics/mm/wind.h"

#include "test_support.h"

using namespace Ballistics::MM;

int main() {
    CHECK(WindField::calm().at(0.0, 0.0) == glm::dvec3(0.0));
    CHECK(WindField::calm().at(5000.0, 90.0) == glm::dvec3(0.0));

    // Wind from 180 is a tailwind for a shot fired on bearing 0, so it pushes along +x.
    const Uniform tail(10.0, 180.0);
    CHECK_NEAR(tail.at(0.0, 0.0).x, 10.0, 1e-9);
    CHECK_NEAR(tail.at(0.0, 0.0).z, 0.0, 1e-9);

    const Uniform head(10.0, 0.0);
    CHECK_NEAR(head.at(0.0, 0.0).x, -10.0, 1e-9);

    // Wind from the west pushes east, which is +z when firing north.
    const Uniform cross(10.0, 270.0);
    CHECK_NEAR(cross.at(0.0, 0.0).z, 10.0, 1e-9);
    CHECK_NEAR(cross.at(0.0, 0.0).x, 0.0, 1e-9);

    // Turning the launch azimuth onto the wind makes the crosswind a tailwind.
    CHECK_NEAR(cross.at(0.0, 90.0).x, 10.0, 1e-9);
    CHECK_NEAR(cross.at(0.0, 90.0).z, 0.0, 1e-9);

    // A uniform field ignores height.
    for (double h = 0.0; h <= 10000.0; h += 500.0) {
        CHECK_NEAR(tail.at(h, 0.0).x, 10.0, 1e-9);
    }

    const Sounding shear({{250.0, 5.0, 180.0}, {500.0, 15.0, 180.0}, {750.0, 25.0, 180.0}});
    CHECK_NEAR(shear.at(0.0, 0.0).x, 5.0, 1e-9);
    CHECK_NEAR(shear.at(250.0, 0.0).x, 5.0, 1e-9);
    CHECK_NEAR(shear.at(375.0, 0.0).x, 10.0, 1e-9);
    CHECK_NEAR(shear.at(500.0, 0.0).x, 15.0, 1e-9);
    CHECK_NEAR(shear.at(750.0, 0.0).x, 25.0, 1e-9);
    CHECK_NEAR(shear.at(9000.0, 0.0).x, 25.0, 1e-9);

    // Levels given out of order are sorted.
    const Sounding jumbled({{750.0, 25.0, 180.0}, {250.0, 5.0, 180.0}, {500.0, 15.0, 180.0}});
    CHECK_NEAR(jumbled.at(375.0, 0.0).x, 10.0, 1e-9);
    CHECK(jumbled.levels() == 3);

    // A veer through north takes the short way, and speed survives the turn. Interpolating the
    // vectors instead would lose about 0.9 m/s of a 5-to-12 m/s layer.
    const Sounding veer({{0.0, 10.0, 350.0}, {1000.0, 10.0, 10.0}});
    CHECK_NEAR(veer.at(500.0, 0.0).x, -10.0, 1e-9);
    CHECK_NEAR(veer.at(500.0, 0.0).z, 0.0, 1e-9);
    for (double h = 0.0; h <= 1000.0; h += 50.0) {
        CHECK_NEAR(glm::length(veer.at(h, 0.0)), 10.0, 1e-9);
    }

    const Sounding turning({{0.0, 5.0, 180.0}, {1000.0, 12.0, 240.0}});
    for (double h = 0.0; h <= 1000.0; h += 50.0) {
        const double speed = glm::length(turning.at(h, 0.0));
        const double expected = 5.0 + (12.0 - 5.0) * h / 1000.0;
        CHECK_NEAR(speed, expected, 1e-9);
    }

    CHECK(Sounding({}).at(100.0, 0.0) == glm::dvec3(0.0));

    return test::summary("wind");
}
