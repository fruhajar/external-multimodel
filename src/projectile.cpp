#include "ballistics/mm/projectile.h"

#include "constants.h"

namespace Ballistics::MM {

double MassGeometry::refArea() const {
    return Constants::PI * diameter * diameter * 0.25;
}

const char* describe(DataBlock b) {
    switch (b) {
    case DataBlock::Geometry: return "mass and geometry";
    case DataBlock::Muzzle:   return "muzzle data";
    case DataBlock::Drag:     return "a drag model";
    case DataBlock::Motor:    return "a motor profile";
    case DataBlock::Fins:     return "fin geometry";
    case DataBlock::Inertia:  return "moments of inertia";
    case DataBlock::Moments:  return "aerodynamic moment coefficients";
    }
    return "unknown";
}

const char* describe(DataQuality q) {
    switch (q) {
    case DataQuality::Calibrated:  return "calibrated against a published maximum range";
    case DataQuality::Estimated:   return "class-typical coefficients, no round-specific source";
    case DataQuality::Placeholder: return "unverified, carries no authority";
    }
    return "unknown";
}

Result<MissingData> validate(const Projectile& p, DataRequirements required) {
    if (needs(required, DataBlock::Geometry) &&
        (p.geometry.mass <= 0.0 || p.geometry.diameter <= 0.0)) {
        return {Status::MissingData, {DataBlock::Geometry}};
    }
    if (needs(required, DataBlock::Muzzle) && p.muzzle.speed <= 0.0) {
        return {Status::MissingData, {DataBlock::Muzzle}};
    }
    if (needs(required, DataBlock::Drag) && !p.drag) {
        return {Status::MissingData, {DataBlock::Drag}};
    }
    if (needs(required, DataBlock::Motor) && !p.motor) {
        return {Status::MissingData, {DataBlock::Motor}};
    }
    if (needs(required, DataBlock::Fins) && !p.fins) {
        return {Status::MissingData, {DataBlock::Fins}};
    }
    if (needs(required, DataBlock::Inertia) &&
        (!p.inertia || p.inertia->Ixx <= 0.0 || p.inertia->Iyy <= 0.0)) {
        return {Status::MissingData, {DataBlock::Inertia}};
    }
    if (needs(required, DataBlock::Moments) &&
        (!p.moments || p.moments->C_M_alpha == 0.0)) {
        return {Status::MissingData, {DataBlock::Moments}};
    }
    return Result<MissingData>::ok({});
}

double gyroscopicStability(const Projectile& p, double airDensity) {
    if (!p.inertia || !p.moments || p.moments->C_M_alpha <= 0.0 || p.muzzle.speed <= 0.0) {
        return 0.0;
    }
    const double spin = p.muzzle.spinRate * 2.0 * Constants::PI / 60.0;
    const double numerator = p.inertia->Ixx * p.inertia->Ixx * spin * spin;
    const double denominator = 2.0 * airDensity * p.inertia->Iyy * p.geometry.refArea() *
                               p.geometry.diameter * p.muzzle.speed * p.muzzle.speed *
                               p.moments->C_M_alpha;
    return denominator > 0.0 ? numerator / denominator : 0.0;
}

} // namespace Ballistics::MM
