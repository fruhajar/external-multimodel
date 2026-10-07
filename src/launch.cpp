#include "ballistics/mm/launch.h"

namespace Ballistics::MM {

LaunchState fromGround(double speed, double elevation, double azimuthOffset,
                       double altitudeMsl, double spinRate) {
    const double horizontal = speed * std::cos(elevation);
    LaunchState s;
    s.velocity = glm::dvec3(horizontal * std::cos(azimuthOffset),
                            speed * std::sin(elevation),
                            horizontal * std::sin(azimuthOffset));
    s.altitudeMsl = altitudeMsl;
    s.spinRate = spinRate;
    return s;
}

LaunchState fromPlatform(const glm::dvec3& platformVelocity, const glm::dvec3& releaseVelocity,
                         const glm::dvec3& position, double altitudeMsl, double spinRate) {
    LaunchState s;
    s.position = position;
    s.velocity = platformVelocity + releaseVelocity;
    s.altitudeMsl = altitudeMsl;
    s.spinRate = spinRate;
    return s;
}

} // namespace Ballistics::MM
