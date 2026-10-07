#ifndef BALLISTICS_MM_PROJECTILE_H
#define BALLISTICS_MM_PROJECTILE_H

#include "drag.h"
#include "types.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace Ballistics::MM {

struct MassGeometry {
    double mass = 0.0;
    double diameter = 0.0;
    double length = 0.0;

    double refArea() const;
};

struct MuzzleData {
    double speed = 0.0;
    double spinRate = 0.0;   // rpm at the muzzle
};

struct MotorProfile {
    double burnTime = 0.0;
    double thrust = 0.0;
    double dragFactorDuringBurn = 1.0;
};

struct FinGeometry {
    double area = 0.0;
    double cantAngle = 0.0;   // radians
    double liftCoefficient = 0.0;
};

struct Inertia {
    double Ixx = 0.0;   // axial
    double Iyy = 0.0;   // transverse
    double Izz = 0.0;
};

struct MomentCoefficients {
    double C_L_alpha = 0.0;    // normal force slope
    double C_M_alpha = 0.0;    // overturning moment, positive for a statically unstable body
    double C_M_q = 0.0;        // pitch damping, negative
    double C_M_palpha = 0.0;   // Magnus moment
    double C_l_p = 0.0;        // roll damping, negative
    double C_l_delta = 0.0;    // fin cant roll driving
};

enum class DataBlock : std::uint32_t {
    Geometry = 1u << 0,
    Muzzle   = 1u << 1,
    Drag     = 1u << 2,
    Motor    = 1u << 3,
    Fins     = 1u << 4,
    Inertia  = 1u << 5,
    Moments  = 1u << 6,
};

using DataRequirements = std::uint32_t;

constexpr DataRequirements operator|(DataBlock a, DataBlock b) {
    return static_cast<DataRequirements>(a) | static_cast<DataRequirements>(b);
}
constexpr DataRequirements operator|(DataRequirements a, DataBlock b) {
    return a | static_cast<DataRequirements>(b);
}
constexpr bool needs(DataRequirements r, DataBlock b) {
    return (r & static_cast<DataRequirements>(b)) != 0;
}

const char* describe(DataBlock b);

enum class Stabilisation { Spin, Fin };

enum class DataQuality { Calibrated, Estimated, Placeholder };

const char* describe(DataQuality q);

struct Projectile {
    std::string id;
    std::string name;
    std::string role;

    MassGeometry geometry;
    MuzzleData muzzle;
    std::shared_ptr<const DragModel> drag;

    std::optional<MotorProfile> motor = std::nullopt;
    std::optional<FinGeometry> fins = std::nullopt;
    std::optional<Inertia> inertia = std::nullopt;
    std::optional<MomentCoefficients> moments = std::nullopt;

    Stabilisation stabilisation = Stabilisation::Fin;
    DataQuality quality = DataQuality::Placeholder;
};

// Names the first block the requested model needs and the projectile does not carry.
struct MissingData {
    DataBlock block = DataBlock::Geometry;
};

Result<MissingData> validate(const Projectile& p, DataRequirements required);

// Gyroscopic stability factor at the muzzle. Below 1 a spin-stabilised round tumbles; artillery
// is designed near 1.3 to 2.0. Returns 0 when the projectile lacks the data to say.
double gyroscopicStability(const Projectile& p, double airDensity = 1.225);

} // namespace Ballistics::MM

#endif
