#include "ballistics/mm/config.h"

#include <cmath>

namespace Ballistics::MM {

SolverConfig SolverConfig::forSpinRate(double spinRpm) {
    SolverConfig c;
    const double spin = std::abs(spinRpm) * 2.0 * 3.14159265358979323846 / 60.0;
    if (spin > 0.0) {
        c.dt = MAX_SPIN_PHASE_PER_STEP / spin;
    }
    c.sampleInterval = 0.01;
    return c;
}

SolverConfig SolverConfig::fast() {
    SolverConfig c;
    c.integrator = Integrator::RK2;
    c.dt = 0.02;
    c.sampleInterval = 0.1;
    return c;
}

SolverConfig SolverConfig::standard() {
    return SolverConfig{};
}

SolverConfig SolverConfig::precise() {
    SolverConfig c;
    c.dt = 0.0005;
    c.sampleInterval = 0.01;
    c.rangeTolerance = 1.0;
    c.crossrangeTolerance = 0.25;
    return c;
}

SolverConfig SolverConfig::adaptive() {
    SolverConfig c;
    c.integrator = Integrator::DOPRI5;
    c.dt = 0.005;             // starting step only
    c.absTol = 1e-7;
    c.relTol = 1e-8;
    c.dtMin = 1e-6;
    c.dtMax = 0.1;
    c.sampleInterval = 0.01;
    c.rangeTolerance = 1.0;
    c.crossrangeTolerance = 0.25;
    return c;
}

} // namespace Ballistics::MM
