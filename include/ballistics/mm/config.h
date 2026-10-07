#ifndef BALLISTICS_MM_CONFIG_H
#define BALLISTICS_MM_CONFIG_H

#include <functional>

namespace Ballistics::MM {

enum class Integrator {
    RK2,
    RK4,
    // Dormand-Prince 5(4) with step control. Selected by setting absTol or relTol above zero.
    DOPRI5,
};

struct StepReport {
    double t = 0.0;
    double dt = 0.0;
    double speed = 0.0;
    double altitude = 0.0;
    double alpha = 0.0;        // total angle of attack, radians
    double spin = 0.0;         // rad/s
    double rotationalEnergy = 0.0;
};

struct SolverConfig {
    Integrator integrator = Integrator::RK4;
    double dt = 0.005;
    double maxFlightTime = 600.0;

    // Seconds between stored samples, so output density does not move when dt does.
    double sampleInterval = 0.025;
    bool storeTrajectory = false;

    // Above zero, DOPRI5 controls its own step within [dtMin, dtMax].
    double absTol = 0.0;
    double relTol = 0.0;
    double dtMin = 1e-6;
    double dtMax = 0.05;

    double rangeTolerance = 10.0;
    double crossrangeTolerance = 1.0;
    int maxElevationIterations = 120;
    int maxAzimuthIterations = 30;
    double azimuthDamping = 0.7;

    // Called once per accepted step when set. Diagnostics belong here rather than printed from inside a derivative.
    std::function<void(const StepReport&)> diagnostics;

    // Largest spin phase a single step may advance, radians. Only models that integrate
    // attitude care. 0.5 rad keeps the impact point inside 0.1% of a converged run; 1.7 rad
    // aliases the rotation and the round tumbles numerically.
    static constexpr double MAX_SPIN_PHASE_PER_STEP = 0.5;

    // A fixed-step config whose dt can resolve the given spin, in rpm.
    static SolverConfig forSpinRate(double spinRpm);

    static SolverConfig fast();
    static SolverConfig standard();
    static SolverConfig precise();    // fixed step, fine
    static SolverConfig adaptive();   // tolerance driven
};

} // namespace Ballistics::MM

#endif
