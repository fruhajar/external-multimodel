#ifndef BALLISTICS_MM_WIND_H
#define BALLISTICS_MM_WIND_H

#include "types.h"

namespace Ballistics::MM {

// One level of a sounding: altitude above the ground, speed, and the bearing the wind blows
// FROM in degrees (met convention).
struct WindLayer {
    double altitude = 0.0;
    double speed = 0.0;
    double fromBearing = 0.0;
};

class WindField {
public:
    virtual ~WindField() = default;

    // Resolved into the local frame. launchAzimuth is the firing bearing in degrees.
    virtual glm::dvec3 at(double altitudeAgl, double launchAzimuthDeg) const = 0;

    static const WindField& calm();
};

class Uniform final : public WindField {
public:
    Uniform(double speed, double fromBearing);
    glm::dvec3 at(double altitudeAgl, double launchAzimuthDeg) const override;

private:
    double m_speed, m_fromBearing;
};

// Levels are interpolated by speed and bearing, not as vectors, so a veering wind keeps its
// speed through the turn. Held constant above the top level and below the bottom.
class Sounding final : public WindField {
public:
    explicit Sounding(std::vector<WindLayer> layers);
    glm::dvec3 at(double altitudeAgl, double launchAzimuthDeg) const override;

    std::size_t levels() const { return m_layers.size(); }

private:
    std::vector<WindLayer> m_layers;
};

} // namespace Ballistics::MM

#endif
