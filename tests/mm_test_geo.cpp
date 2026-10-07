#include "ballistics/mm/types.h"

#include "test_support.h"

using namespace Ballistics::MM;

int main() {
    const GeoCoordinate origin{0.0, 0.0, 0.0};
    const GeoCoordinate prague{50.0875, 14.4213, 200.0};
    const GeoCoordinate brno{49.1951, 16.6068, 200.0};

    // Great-circle Prague to Brno: 186.03 km on an initial bearing of 121.40 degrees.
    // Feeding degrees to sin and cos here returns 178.65, a 57 degree error.
    CHECK_REL(prague.distanceTo(brno), 186031.7, 1e-4);
    CHECK_REL(prague.azimuthTo(brno), 121.3975, 1e-4);

    // Symmetry and the degenerate case.
    CHECK_REL(brno.distanceTo(prague), prague.distanceTo(brno), 1e-12);
    CHECK_NEAR(prague.distanceTo(prague), 0.0, 1e-9);

    // Cardinal bearings from the equator.
    CHECK_NEAR(origin.azimuthTo({1.0, 0.0, 0.0}), 0.0, 1e-6);
    CHECK_NEAR(origin.azimuthTo({0.0, 1.0, 0.0}), 90.0, 1e-6);
    CHECK_NEAR(origin.azimuthTo({-1.0, 0.0, 0.0}), 180.0, 1e-6);
    CHECK_NEAR(origin.azimuthTo({0.0, -1.0, 0.0}), 270.0, 1e-6);

    // One degree of latitude is about 111.2 km.
    CHECK_REL(origin.distanceTo({1.0, 0.0, 0.0}), 111195.0, 1e-3);

    // Altitude difference enters as a straight leg.
    const GeoCoordinate above{0.0, 0.0, 300.0};
    CHECK_REL(origin.distanceTo(above), 300.0, 1e-9);

    // Projecting a local trajectory back out: a due-north leg must raise latitude only.
    Trajectory local;
    local.appendKinematics(0.0, glm::dvec3(0.0, 0.0, 0.0), glm::dvec3(0.0));
    local.appendKinematics(1.0, glm::dvec3(10000.0, 500.0, 0.0), glm::dvec3(0.0));

    const std::vector<GeoCoordinate> geo = toGeo(local, origin, 0.0);
    CHECK(geo.size() == 2);
    CHECK_NEAR(geo[1].longitude, 0.0, 1e-9);
    CHECK(geo[1].latitude > 0.0);
    const GeoCoordinate projected{geo[1].latitude, geo[1].longitude, 0.0};
    CHECK_REL(origin.distanceTo(projected), 10000.0, 1e-4);
    CHECK_NEAR(geo[1].altitude, 500.0, 1e-9);

    // Firing east moves longitude instead.
    const std::vector<GeoCoordinate> east = toGeo(local, origin, 90.0);
    CHECK_NEAR(east[1].latitude, 0.0, 1e-9);
    CHECK(east[1].longitude > 0.0);

    return test::summary("geo");
}
