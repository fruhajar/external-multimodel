#ifndef BALLISTICS_MM_TYPES_H
#define BALLISTICS_MM_TYPES_H

// Local frame: +x downrange along the launch azimuth, +y up, +z crossrange (right of downrange).
// Left-handed, matching GLM_FORCE_LEFT_HANDED. Metres, seconds, kilograms, radians.
// Identical to ballistics/pm, so a caller moving between the two libraries changes the
// namespace and picks a model, nothing else.

#include <glm/ext/vector_double3.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/trigonometric.hpp>

#include <cmath>
#include <vector>

namespace Ballistics::MM {

enum class Status {
    Ok,
    MissingData,     // projectile lacks a block the chosen model needs
    InvalidInput,
    NoImpact,
    Unreachable,
    NotConverged,
    // The timestep cannot resolve the spin. A rigid-body run integrates attitude, so the step
    // has to be small enough to follow the rotation; a spun shell at 1676 rad/s turns 8.4 rad in
    // a 5 ms step, which aliases and produces a numerically tumbling round rather than an answer.
    StepTooLarge,
};

template <class T>
struct Result {
    Status status = Status::Ok;
    T value{};

    explicit operator bool() const { return status == Status::Ok; }
    const T* operator->() const { return &value; }
    const T& operator*() const { return value; }

    static Result ok(T v) { return {Status::Ok, std::move(v)}; }
    static Result fail(Status s) { return {s, T{}}; }
};

const char* describe(Status s);

struct Impact {
    double tof = 0.0;
    double range = 0.0;
    double crossrange = 0.0;
    double altitudeRel = 0.0;
    glm::dvec3 velocity{0.0};
    double impactAngle = 0.0;     // radians below horizontal, positive downward
    double terminalSpeed = 0.0;

    // Filled only by models that carry attitude.
    double spin = 0.0;            // rad/s about the axis of symmetry
    glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
    glm::dvec3 omega{0.0};
    double alpha = 0.0;           // total angle of attack at impact, radians
};

// Columns, with the attitude ones present only when the model produces them. One type serves
// all three models, so a caller's handling code does not change when the model does.
struct Trajectory {
    std::vector<double> t, x, y, z, vx, vy, vz;

    std::vector<double> spin;               // MPMM and rigid body
    std::vector<double> alpha;              // MPMM and rigid body
    std::vector<glm::dquat> orientation;    // rigid body only
    std::vector<glm::dvec3> omega;          // rigid body only

    Impact impact;

    std::size_t size() const { return t.size(); }
    bool empty() const { return t.empty(); }
    bool hasAttitude() const { return !orientation.empty(); }
    bool hasSpin() const { return !spin.empty(); }

    void reserveKinematics(std::size_t n);
    void appendKinematics(double time, const glm::dvec3& pos, const glm::dvec3& vel);
};

struct GeoCoordinate {
    double latitude = 0.0;   // degrees
    double longitude = 0.0;  // degrees
    double altitude = 0.0;   // metres MSL

    double azimuthTo(const GeoCoordinate& other) const;
    double distanceTo(const GeoCoordinate& other) const;
};

std::vector<GeoCoordinate> toGeo(const Trajectory& local,
                                 const GeoCoordinate& origin,
                                 double azimuthDeg);

} // namespace Ballistics::MM

#endif
