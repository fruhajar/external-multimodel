#include "ballistics/mm/wind.h"

#include <algorithm>

namespace Ballistics::MM {

namespace {

// Met bearing is where the wind comes from; add 180 to get where it pushes, then resolve
// relative to the launch azimuth.
glm::dvec3 resolve(double speed, double fromBearing, double launchAzimuthDeg) {
    const double toward = glm::radians(fromBearing + 180.0 - launchAzimuthDeg);
    return glm::dvec3(speed * std::cos(toward), 0.0, speed * std::sin(toward));
}

class Calm final : public WindField {
public:
    glm::dvec3 at(double, double) const override { return glm::dvec3(0.0); }
};

} // namespace

const WindField& WindField::calm() {
    static const Calm calm;
    return calm;
}

Uniform::Uniform(double speed, double fromBearing)
    : m_speed(speed), m_fromBearing(fromBearing) {}

glm::dvec3 Uniform::at(double, double launchAzimuthDeg) const {
    return resolve(m_speed, m_fromBearing, launchAzimuthDeg);
}

Sounding::Sounding(std::vector<WindLayer> layers) : m_layers(std::move(layers)) {
    std::sort(m_layers.begin(), m_layers.end(),
              [](const WindLayer& a, const WindLayer& b) { return a.altitude < b.altitude; });
}

glm::dvec3 Sounding::at(double altitudeAgl, double launchAzimuthDeg) const {
    if (m_layers.empty()) {
        return glm::dvec3(0.0);
    }
    if (m_layers.size() == 1 || altitudeAgl <= m_layers.front().altitude) {
        return resolve(m_layers.front().speed, m_layers.front().fromBearing, launchAzimuthDeg);
    }
    if (altitudeAgl >= m_layers.back().altitude) {
        return resolve(m_layers.back().speed, m_layers.back().fromBearing, launchAzimuthDeg);
    }

    const auto upper = std::lower_bound(
        m_layers.begin(), m_layers.end(), altitudeAgl,
        [](const WindLayer& l, double h) { return l.altitude < h; });
    const auto lower = upper - 1;

    const double span = upper->altitude - lower->altitude;
    const double blend = span > 0.0 ? (altitudeAgl - lower->altitude) / span : 0.0;

    // Take the short way round the compass so a veer through north behaves, and interpolate
    // speed separately so a veering wind does not lose speed through the turn.
    double delta = upper->fromBearing - lower->fromBearing;
    while (delta > 180.0) delta -= 360.0;
    while (delta < -180.0) delta += 360.0;

    return resolve(lower->speed + blend * (upper->speed - lower->speed),
                   lower->fromBearing + blend * delta, launchAzimuthDeg);
}

} // namespace Ballistics::MM
