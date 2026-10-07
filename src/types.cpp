#include "ballistics/mm/types.h"

#include "constants.h"

namespace Ballistics::MM {

using namespace Constants;

const char* describe(Status s) {
    switch (s) {
    case Status::Ok:           return "ok";
    case Status::MissingData:  return "projectile is missing data the model requires";
    case Status::InvalidInput: return "invalid input";
    case Status::NoImpact:     return "no impact within the maximum flight time";
    case Status::Unreachable:  return "target outside the achievable envelope";
    case Status::NotConverged: return "solver did not converge to tolerance";
    case Status::StepTooLarge: return "timestep too large to resolve the spin; "
                                     "use SolverConfig::forSpinRate or an adaptive integrator";
    }
    return "unknown";
}

void Trajectory::reserveKinematics(std::size_t n) {
    t.reserve(n);
    x.reserve(n);
    y.reserve(n);
    z.reserve(n);
    vx.reserve(n);
    vy.reserve(n);
    vz.reserve(n);
}

void Trajectory::appendKinematics(double time, const glm::dvec3& pos, const glm::dvec3& vel) {
    t.push_back(time);
    x.push_back(pos.x);
    y.push_back(pos.y);
    z.push_back(pos.z);
    vx.push_back(vel.x);
    vy.push_back(vel.y);
    vz.push_back(vel.z);
}

double GeoCoordinate::azimuthTo(const GeoCoordinate& other) const {
    const double lat1 = glm::radians(latitude);
    const double lat2 = glm::radians(other.latitude);
    const double dLon = glm::radians(other.longitude - longitude);

    const double y = std::sin(dLon) * std::cos(lat2);
    const double x = std::cos(lat1) * std::sin(lat2) -
                     std::sin(lat1) * std::cos(lat2) * std::cos(dLon);

    return std::fmod(glm::degrees(std::atan2(y, x)) + 360.0, 360.0);
}

double GeoCoordinate::distanceTo(const GeoCoordinate& other) const {
    const double lat1 = glm::radians(latitude);
    const double lat2 = glm::radians(other.latitude);
    const double dLat = lat2 - lat1;
    const double dLon = glm::radians(other.longitude - longitude);

    const double sinLat = std::sin(dLat * 0.5);
    const double sinLon = std::sin(dLon * 0.5);
    const double a = sinLat * sinLat + std::cos(lat1) * std::cos(lat2) * sinLon * sinLon;
    const double surface = 2.0 * R_EARTH * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

    const double dAlt = other.altitude - altitude;
    return std::sqrt(surface * surface + dAlt * dAlt);
}

std::vector<GeoCoordinate> toGeo(const Trajectory& local, const GeoCoordinate& origin,
                                double azimuthDeg) {
    std::vector<GeoCoordinate> out(local.size());

    const double azimuth = glm::radians(azimuthDeg);
    const double lat1 = glm::radians(origin.latitude);
    const double lon1 = glm::radians(origin.longitude);
    const double sinLat1 = std::sin(lat1);
    const double cosLat1 = std::cos(lat1);

    for (std::size_t i = 0; i < local.size(); ++i) {
        const double dist = std::sqrt(local.x[i] * local.x[i] + local.z[i] * local.z[i]);
        const double bearing = azimuth + std::atan2(local.z[i], local.x[i]);
        const double arc = dist / R_EARTH;

        const double lat2 = std::asin(sinLat1 * std::cos(arc) +
                                      cosLat1 * std::sin(arc) * std::cos(bearing));
        const double lon2 = lon1 + std::atan2(std::sin(bearing) * std::sin(arc) * cosLat1,
                                               std::cos(arc) - sinLat1 * std::sin(lat2));

        out[i] = {glm::degrees(lat2), glm::degrees(lon2), origin.altitude + local.y[i]};
    }
    return out;
}

} // namespace Ballistics::MM
