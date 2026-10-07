#include "ballistics/mm/drag.h"

#include "test_support.h"

#include <memory>

using namespace Ballistics::MM;

int main() {
    const StandardDrag g1(DragFamily::G1, 1.0);
    const StandardDrag g7(DragFamily::G7, 1.0);
    const StandardDrag shell(DragFamily::Shell, 1.0);

    CHECK_NEAR(g1.at(0.0), 0.2629, 1e-12);
    CHECK_NEAR(g1.at(5.0), 0.4890, 1e-12);
    CHECK_NEAR(g7.at(0.0), 0.1198, 1e-12);
    CHECK_NEAR(shell.at(0.0), 0.130, 1e-12);

    // Clamped outside the table rather than extrapolated.
    CHECK_NEAR(g1.at(-1.0), 0.2629, 1e-12);
    CHECK_NEAR(shell.at(50.0), 0.208, 1e-12);

    // The shell curve peaks just above Mach 1 at roughly 2.6 times subsonic, which is the shape
    // an artillery shell has and neither standard family does.
    const double subsonic = shell.at(0.5);
    CHECK_REL(shell.at(1.2) / subsonic, 2.6, 0.05);
    CHECK(shell.at(1.2) > shell.at(2.0));
    CHECK(shell.at(2.0) > shell.at(4.0));

    // G7 barely rises through the transonic region, G1 peaks much later.
    CHECK(g7.at(1.2) / g7.at(0.5) < 1.05);
    CHECK(g1.at(2.0) > g1.at(1.2));

    for (double m = 0.0; m <= 6.0; m += 0.01) {
        CHECK(shell.at(m) > 0.12 && shell.at(m) < 0.35);
        CHECK(g1.at(m) > 0.15 && g1.at(m) < 0.60);
    }

    // Form factor scales linearly, and forCd inverts the reference value.
    CHECK_NEAR(StandardDrag(DragFamily::Shell, 2.0).at(1.0), 2.0 * shell.at(1.0), 1e-12);
    CHECK_REL(StandardDrag::forCd(DragFamily::Shell, 0.147)->at(0.0), 0.147, 1e-12);
    CHECK_REL(StandardDrag::forCd(DragFamily::G1, 0.20)->at(0.0), 0.20, 1e-12);
    CHECK_REL(referenceCd(DragFamily::G1), 0.2629, 1e-12);
    CHECK_REL(referenceCd(DragFamily::Shell), 0.130, 1e-12);

    // Yaw drag is quadratic in alpha and off by default.
    StandardDrag yawing(DragFamily::Shell, 1.0);
    CHECK_NEAR(yawing.at(1.5, 0.2), yawing.at(1.5), 1e-12);
    yawing.setYawDrag(4.0);
    CHECK_NEAR(yawing.at(1.5, 0.0), yawing.at(1.5), 1e-12);
    CHECK_NEAR(yawing.at(1.5, 0.1), yawing.at(1.5) + 0.04, 1e-12);
    CHECK_NEAR(yawing.at(1.5, 0.2), yawing.at(1.5) + 0.16, 1e-12);

    // A custom table behaves like a family, and a mismatched one is rejected.
    const CustomDrag flat({0.0, 1.0, 2.0}, {0.3, 0.3, 0.3});
    CHECK_NEAR(flat.at(0.5), 0.3, 1e-12);
    CHECK_NEAR(flat.at(1.7), 0.3, 1e-12);
    CHECK(CustomDrag({0.0, 1.0}, {0.3}).empty());

    // Dispatch through the interface must reach the override.
    const std::shared_ptr<const DragModel> model =
        StandardDrag::forCd(DragFamily::Shell, 0.147);
    CHECK_REL(model->at(1.2), 0.147 / 0.130 * 0.337, 1e-9);
    CHECK_NEAR(model->at(1.2, 0.3), model->at(1.2), 1e-12);

    return test::summary("drag");
}
