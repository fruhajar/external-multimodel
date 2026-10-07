// With no aerodynamic moment acting, world-frame angular momentum must be constant. This is the
// invariant that catches a whole class of error: reversed Euler coupling signs, or quaternion
// kinematics using the world-frame relation on body-frame rates. Either rotates L through a large
// fraction of its own magnitude within seconds. Magnitude and energy
// are conserved either way, so only the direction check finds it.
//

#include "ballistics/mm/ballistics.h"

#include "test_support.h"

using namespace Ballistics::MM;

namespace {

// Zero density removes every aerodynamic term, leaving gravity. Gravity acts at the centre of
// mass, so it applies no moment.
class Vacuum final : public Atmosphere {
public:
    double density(double) const override { return 0.0; }
    double speedOfSound(double) const override { return 340.29; }
    double temperature(double) const override { return 288.15; }
};

Projectile spunShell() {
    Projectile p;
    p.id = "shell";
    p.geometry = {46.5, 0.155, 0.857};
    p.muzzle = {827.0, 16006.0};
    p.drag = StandardDrag::forCd(DragFamily::Shell, 0.147);
    p.inertia = Inertia{0.15540, 2.9158, 2.9158};
    p.moments = MomentCoefficients{1.6, 3.7, -8.0, 0.35, -0.015, 0.0};
    p.stabilisation = Stabilisation::Spin;
    return p;
}

} // namespace

int main() {
    const Vacuum vacuum;
    Environment env = Environment::standard();
    env.atmosphere = &vacuum;

    const Projectile shell = spunShell();

    SolverConfig cfg = SolverConfig::forSpinRate(shell.muzzle.spinRate);
    cfg.dt *= 0.2;
    cfg.storeTrajectory = true;
    cfg.sampleInterval = 0.05;
    cfg.maxFlightTime = 60.0;

    LaunchState launch = fromGround(shell.muzzle.speed, 0.3, 0.0, 0.0, shell.muzzle.spinRate);
    launch.initialYaw = 0.02;   // deliberately off-axis, so the transverse rates are nonzero

    const Result<Trajectory> t = predict(ModelKind::RigidBody, launch, shell, env,
                                         GroundReference{0.0}, cfg);
    CHECK_MSG(static_cast<bool>(t), describe(t.status));
    if (!t) {
        return test::summary("conservation");
    }

    CHECK(t->hasAttitude());
    CHECK(t->size() > 100);

    const auto angularMomentum = [&](std::size_t i) {
        const glm::dvec3 body(shell.inertia->Ixx * t->omega[i].x,
                              shell.inertia->Iyy * t->omega[i].y,
                              shell.inertia->Izz * t->omega[i].z);
        return t->orientation[i] * body;
    };

    const glm::dvec3 initial = angularMomentum(0);
    const double initialMagnitude = glm::length(initial);
    CHECK(initialMagnitude > 0.0);

    const auto energy = [&](std::size_t i) {
        return 0.5 * (shell.inertia->Ixx * t->omega[i].x * t->omega[i].x +
                      shell.inertia->Iyy * t->omega[i].y * t->omega[i].y +
                      shell.inertia->Izz * t->omega[i].z * t->omega[i].z);
    };
    const double initialEnergy = energy(0);

    double worstMagnitude = 0.0;
    double worstDirection = 0.0;
    double worstEnergy = 0.0;

    for (std::size_t i = 1; i < t->size(); ++i) {
        const glm::dvec3 L = angularMomentum(i);
        worstMagnitude = std::max(worstMagnitude,
                                  std::abs(glm::length(L) - initialMagnitude) / initialMagnitude);
        worstDirection = std::max(worstDirection, glm::length(L - initial) / initialMagnitude);
        worstEnergy = std::max(worstEnergy, std::abs(energy(i) - initialEnergy) / initialEnergy);

        // Quaternions must stay unit, or the rotation has stopped being a rotation.
        const glm::dquat q = t->orientation[i];
        CHECK_NEAR(std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z), 1.0, 1e-9);
    }

    std::printf("torque-free over %.0f s: |L| drift %.2e, L direction drift %.2e, E drift %.2e\n",
                t->impact.tof, worstMagnitude, worstDirection, worstEnergy);

    // A model that conflates the body and world frames scores around 6e-01 here.
    CHECK_MSG(worstDirection < 1e-4, "L direction drift " + std::to_string(worstDirection));
    CHECK_MSG(worstMagnitude < 1e-6, "|L| drift " + std::to_string(worstMagnitude));
    CHECK_MSG(worstEnergy < 1e-6, "E drift " + std::to_string(worstEnergy));

    // Axial spin is untouched without an axial moment.
    for (std::size_t i = 0; i < t->size(); ++i) {
        CHECK_REL(t->spin[i], t->spin[0], 1e-9);
    }

    return test::summary("conservation");
}
