# Choosing a model

Measured on a 155mm M795 at 45 degrees, standard day, 50 degrees latitude, on this machine.
Reproduce with `multimodel/examples/compare`.

| model | range | tof | drift | yaw at impact | cost |
|---|---|---|---|---|---|
| point mass (3 DOF) | 22,345 m | 77.88 s | 60 m | n/a | 8 ms |
| MPMM (4 DOF) | 22,318 m | 77.88 s | 842 m | 0.21 deg | **1 ms** |
| rigid body (6 DOF) | 22,308 m | 77.93 s | 844 m | 0.04 deg | 599 ms |

Range and time of flight agree within 0.2% across all three. What separates them is drift and
attitude.

## What each is for

**Point mass.** Where the round goes, with no claim about which way it is pointing. The 60 m of
crossrange here is Coriolis alone. Use it when the projectile is not spin-stabilised, or when
drift does not matter, or inside something that iterates. `ballistics/pm` is this model on its
own, and is the right choice if it is all you need.

**MPMM.** The mid-range workhorse. Point mass plus the roll degree of freedom, with the yaw of
repose solved analytically from the gyroscopic balance (STANAG 4355). It reproduces the rigid
body's drift to 0.2% at a six-hundredth of the cost, and needs only `Ixx` plus `C_M_alpha`,
`C_L_alpha` and `C_l_p` rather than a full coefficient set. This is the default for anything
involving a spun projectile.

**Rigid body.** Integrates attitude, so the repose angle emerges from the moment balance instead
of being imposed. Its job is to validate MPMM and to show transient yawing motion that MPMM
cannot represent at all. It is not the model to reach for by default.

## Why the rigid body costs what it does

It has to resolve the spin. A 155mm at 16,006 rpm turns 1676 rad/s, so a 5 ms step advances the
attitude 8.4 radians and the rotation aliases: the round tumbles numerically and the answer looks
plausible while being meaningless. `SolverConfig::MAX_SPIN_PHASE_PER_STEP` caps the phase a step
may advance at 0.5 rad, and a rigid-body run with a step above it is **refused** with
`Status::StepTooLarge` rather than answered.

Measured, at 45 degrees:

| spin x dt | result |
|---|---|
| 8.4 rad (dt 5 ms) | refused; would tumble |
| 1.7 rad (dt 1 ms) | refused; alpha diverges past 30 deg by t = 0.9 s |
| 0.34 rad (dt 0.2 ms) | 22,327 m |
| 0.08 rad (dt 50 us) | 22,338 m |
| 0.03 rad (dt 20 us) | 22,340 m |

`SolverConfig::forSpinRate(rpm)` picks a step that is accepted. For a rigid-body run, set
`dtMax` the same way when using the adaptive integrator.

## Fixed step against adaptive

Dormand-Prince 5(4), selected by setting `absTol` or `relTol` above zero. Step counts from the
diagnostics callback.

| model | scheme | steps | range error |
|---|---|---|---|
| MPMM | `precise()`, dt 0.5 ms | 155,760 | 0.0065 m |
| MPMM | `adaptive()` | **781** | **0.0025 m** |
| rigid body | fixed, dt 30 us | 2,612,408 | reference |
| rigid body | adaptive, dtMax from spin | **376,966** | 0.1 m |

For MPMM the adaptive scheme is 200 times cheaper and more accurate, because the trajectory is
smooth and a fixed step sized for the worst moment is wasted everywhere else. Use `adaptive()`
unless you need bit-reproducible steps.

At the default tolerance the adaptive step is capped by `dtMax`, not by accuracy: 781 steps over a
78 s flight is exactly 78/0.1. Loosening the tolerance from there buys nothing; raise `dtMax`
instead.

## Accuracy budget

What is trustworthy, and what only looks it.

**Sound.** Range and time of flight, for the four calibrated rounds, to better than 0.5% of the
published maximum range they were fitted to. The integrators, to the tolerance you ask for. The
rigid body's rotational dynamics: torque-free angular momentum is conserved to 5e-14 over a 50 s
flight. An implementation that conflates the body and world frames scores around 6e-01 here.

**Approximate.** Drift. The form is right and MPMM and the rigid body agree on it independently,
but it rests on `C_M_alpha`, `C_L_alpha` and `Ixx`, which are class-typical rather than measured.
`C_M_alpha` is bounded to 3.5 to 4.0 by the gyroscopic stability the round must have, so it is not
free, but bounded is weaker than known. Expect the right order and sign, not a firing-table number.

**Not modelled.** Motor burn, including the mass loss during it, because no catalogue round carries
motor data. Fin aerodynamics as anything other than a static restoring moment. Base bleed except as
a drag multiplier. Any projectile that manoeuvres.

**Untrustworthy.** The three 120mm smoothbore rounds, which have no published maximum range to fit
against. The moment coefficients for every fin-stabilised round, which are sign-correct and
plausible but have no source. See `docs/catalogue-review.md`.
