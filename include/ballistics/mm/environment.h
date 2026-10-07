#ifndef BALLISTICS_MM_ENVIRONMENT_H
#define BALLISTICS_MM_ENVIRONMENT_H

#include "atmosphere.h"
#include "wind.h"

namespace Ballistics::MM {

// Non-owning: the caller keeps the atmosphere and wind alive for the call. standard() points at
// statics, so it is always safe.
struct Environment {
    const Atmosphere* atmosphere = nullptr;
    const WindField* wind = nullptr;
    double latitude = 0.0;        // degrees, for Coriolis
    double launchAzimuth = 0.0;   // degrees

    static Environment standard();
    bool valid() const { return atmosphere != nullptr && wind != nullptr; }
};

} // namespace Ballistics::MM

#endif
