// The same shot under all three models, and what each costs.

#include "ballistics/mm/ballistics.h"

#include <chrono>
#include <cstdio>

using namespace Ballistics::MM;

int main() {
    const Catalogue c = Catalogue::withBuiltins();
    const Projectile* shell = c.find("Howitzer155mm_HE");
    if (shell == nullptr) {
        return 1;
    }

    Environment env = Environment::standard();
    env.latitude = 50.0;
    const GroundReference flat{0.0};
    const double elevation = glm::radians(45.0);

    std::printf("%s, Sg %.2f at the muzzle\n\n", shell->name.c_str(),
                gyroscopicStability(*shell));
    std::printf("%-22s %9s %8s %9s %10s %8s\n",
                "model", "range", "tof", "drift", "yaw at end", "ms");

    const auto run = [&](ModelKind kind, const SolverConfig& cfg, double yaw) {
        LaunchState l = fromGround(shell->muzzle.speed, elevation, 0.0, 0.0,
                                   shell->muzzle.spinRate);
        l.initialYaw = yaw;

        const auto start = std::chrono::steady_clock::now();
        const Result<Impact> r = predictImpact(kind, l, *shell, env, flat, cfg);
        const double ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();

        if (!r) {
            std::printf("%-22s %s\n", describe(kind), describe(r.status));
            return;
        }
        std::printf("%-22s %9.0f %8.2f %9.1f %9.3f d %8.0f\n", describe(kind), r->range, r->tof,
                    r->crossrange, glm::degrees(r->alpha), ms);
    };

    run(ModelKind::PointMass, SolverConfig::standard(), 0.0);
    run(ModelKind::Mpmm, SolverConfig::adaptive(), 0.0);

    // A rigid-body run has to resolve the spin, which is most of why it costs what it does.
    SolverConfig rigid = SolverConfig::adaptive();
    rigid.dtMax = SolverConfig::MAX_SPIN_PHASE_PER_STEP /
                  (shell->muzzle.spinRate * 2.0 * 3.14159265358979323846 / 60.0);
    run(ModelKind::RigidBody, rigid, 0.002);

    std::printf("\nAsking a rigid body for a 5 ms step: %s\n",
                describe(predictImpact(ModelKind::RigidBody,
                                       fromGround(shell->muzzle.speed, elevation, 0.0, 0.0,
                                                  shell->muzzle.spinRate),
                                       *shell, env, flat, SolverConfig::standard()).status));
    return 0;
}
