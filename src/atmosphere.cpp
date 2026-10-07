#include "ballistics/mm/atmosphere.h"

#include "constants.h"

#include <algorithm>
#include <cmath>

namespace Ballistics::MM {

using namespace Constants;

namespace {

const double PRESSURE_EXPONENT = G / (R_DRY * LAPSE);

double profileTemperature(double seaLevelT, double altitudeMsl) {
    // The lapse stops at the tropopause; hold what the profile reached there so a non-standard
    // sea-level reading stays self-consistent above it.
    const double h = altitudeMsl < TROPOPAUSE ? altitudeMsl : TROPOPAUSE;
    return seaLevelT - LAPSE * h;
}

double profilePressure(double seaLevelT, double seaLevelP, double altitudeMsl) {
    const double t = profileTemperature(seaLevelT, altitudeMsl);
    double p = seaLevelP * std::pow(t / seaLevelT, PRESSURE_EXPONENT);
    if (altitudeMsl > TROPOPAUSE) {
        p *= std::exp(-G * (altitudeMsl - TROPOPAUSE) / (R_DRY * t));
    }
    return p;
}

// Moist air is lighter than dry air at the same pressure: water vapour has a lower molar mass.
double moistDensity(double pressure, double temperature, double vapourPressure) {
    const double dry = pressure - vapourPressure;
    return dry / (R_DRY * temperature) + vapourPressure / (R_VAPOUR * temperature);
}

// Saturation vapour pressure, Tetens.
double saturationVapourPressure(double temperatureK) {
    const double tc = temperatureK - 273.15;
    return 610.78 * std::exp(17.27 * tc / (tc + 237.3));
}

// Water vapour has a scale height near 2 km against roughly 8 km for air, so the mixing ratio
// falls with altitude rather than holding. Scaling vapour with the pressure ratio instead would
// keep e/p constant, and humidity would never thin out.
inline constexpr double VAPOUR_SCALE_HEIGHT = 2000.0;

double vapourAt(double seaLevelVapour, double altitudeMsl, double pressure) {
    if (seaLevelVapour <= 0.0) {
        return 0.0;
    }
    return std::min(seaLevelVapour * std::exp(-std::max(0.0, altitudeMsl) / VAPOUR_SCALE_HEIGHT),
                    pressure);
}

} // namespace

double Isa::temperature(double altitudeMsl) const {
    return profileTemperature(288.15, altitudeMsl);
}

double Isa::density(double altitudeMsl) const {
    return profilePressure(288.15, 101325.0, altitudeMsl) / (R_DRY * temperature(altitudeMsl));
}

double Isa::speedOfSound(double altitudeMsl) const {
    return std::sqrt(GAMMA * R_DRY * temperature(altitudeMsl));
}

const Isa& Isa::standard() {
    static const Isa isa;
    return isa;
}

IsaStation::IsaStation(double temperatureK, double pressurePa, double stationAltitude,
                       double relativeHumidity) {
    m_seaLevelTemperature = temperatureK + LAPSE * stationAltitude;
    m_seaLevelPressure =
        pressurePa * std::pow(m_seaLevelTemperature / temperatureK, PRESSURE_EXPONENT);
    m_vapourPressure = relativeHumidity * saturationVapourPressure(temperatureK);
}

double IsaStation::temperature(double altitudeMsl) const {
    return profileTemperature(m_seaLevelTemperature, altitudeMsl);
}

double IsaStation::density(double altitudeMsl) const {
    const double t = temperature(altitudeMsl);
    const double p = profilePressure(m_seaLevelTemperature, m_seaLevelPressure, altitudeMsl);
    return moistDensity(p, t, vapourAt(m_vapourPressure, altitudeMsl, p));
}

double IsaStation::speedOfSound(double altitudeMsl) const {
    // Humidity raises the speed of sound slightly by lowering the effective molar mass.
    const double t = temperature(altitudeMsl);
    const double p = profilePressure(m_seaLevelTemperature, m_seaLevelPressure, altitudeMsl);
    const double e = vapourAt(m_vapourPressure, altitudeMsl, p);

    const double effectiveR = p > 0.0 ? R_DRY * (1.0 + (1.0 - R_DRY / R_VAPOUR) * e / p) : R_DRY;
    return std::sqrt(GAMMA * effectiveR * t);
}

} // namespace Ballistics::MM
