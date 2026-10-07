#ifndef BALLISTICS_MM_LAUNCH_H
#define BALLISTICS_MM_LAUNCH_H

#include "types.h"

namespace Ballistics::MM {

struct LaunchState {
    glm::dvec3 position{0.0};
    glm::dvec3 velocity{0.0};
    double altitudeMsl = 0.0;
    double spinRate = 0.0;   // rpm

    // Initial angular offset of the axis of symmetry from the velocity vector, radians.
    // Rigid-body runs need a nonzero value to show any yawing motion at all.
    double initialYaw = 0.0;
};

LaunchState fromGround(double speed, double elevation, double azimuthOffset,
                       double altitudeMsl, double spinRate = 0.0);

LaunchState fromPlatform(const glm::dvec3& platformVelocity,
                         const glm::dvec3& releaseVelocity,
                         const glm::dvec3& position,
                         double altitudeMsl,
                         double spinRate = 0.0);

struct GroundReference {
    double altitudeRel = 0.0;
};

} // namespace Ballistics::MM

#endif
