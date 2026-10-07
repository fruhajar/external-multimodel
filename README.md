# ballistics-multimodel

Trajectory prediction with interchangeable models: point mass, modified point mass (STANAG 4355)
and rigid body, chosen at the call site. C++20, depends on glm and the standard library and nothing
else.

If a point mass is all you need, [ballistics-pointmass](https://github.com/fruhajar/external-pointmass) is the same
design in a third of the code and is the better choice. This library is for when drift and
stability have to be right rather than plausible, when met data is good enough to model properly,
or when you want to trade accuracy against cost deliberately.

## Building

Needs a C++20 compiler and [glm](https://github.com/g-truc/glm), which must be discoverable by
`find_package(glm CONFIG)`. On Debian or Ubuntu that is `apt install libglm-dev`; otherwise build
glm from source and point `CMAKE_PREFIX_PATH` at where you installed it. Without it, configuring
fails with `Could not find a package configuration file provided by "glm"`.

Tested with GCC 12 and Clang 14, shared and static, clean under AddressSanitizer and
UndefinedBehaviorSanitizer.

```sh
cmake -S . -B build -DBALLISTICS_MM_BUILD_TESTS=ON -DBALLISTICS_MM_BUILD_EXAMPLES=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

```cmake
find_package(BallisticsMultiModel REQUIRED)
target_link_libraries(app PRIVATE Ballistics::MultiModel)
```

## Choosing a model

```cpp
predictImpact(ModelKind::Mpmm, launch, projectile, env, ground, SolverConfig::adaptive());
```

Measured on a 155mm M795 at 45 degrees, standard day. Reproduce with `examples/compare`.

| model | range | tof | drift | cost |
|---|---|---|---|---|
| point mass (3 DOF) | 22,345 m | 77.88 s | 60 m, Coriolis only | 8 ms |
| MPMM (4 DOF) | 22,318 m | 77.88 s | 842 m | **1 ms** |
| rigid body (6 DOF) | 22,308 m | 77.93 s | 844 m | 599 ms |

**MPMM is the default for anything spun.** It reproduces the rigid body's drift to 0.2% at a
six-hundredth of the cost, needing only `Ixx` plus three coefficients. The rigid body's job is to
validate it and to show transient yawing motion MPMM cannot represent. `docs/model-comparison.md`
has the full budget, including what is not modelled.

Each model declares its data requirements, and a projectile missing a block is refused by name
rather than read as zeros:

```cpp
Result<MissingData> v = validate(projectile, requirementsOf(ModelKind::Mpmm));
// v->block == DataBlock::Inertia
```

### Before using the rigid body

It integrates attitude, so the step has to resolve the spin. A 155mm at 16,006 rpm turns
1676 rad/s, and a 5 ms step would advance attitude 8.4 radians, alias the rotation, and return a
numerically tumbling round that looks like an answer. That is refused:

```cpp
predictImpact(ModelKind::RigidBody, launch, shell, env, ground, SolverConfig::standard());
// Status::StepTooLarge

SolverConfig cfg = SolverConfig::forSpinRate(shell.muzzle.spinRate);   // a step that works
```

This is most of why 6 DOF costs what it does.

## Interchangeable pieces

Behind interfaces, with the same names `ballistics-pointmass` uses for its concrete types:

- **`Atmosphere`**: `Isa`, `IsaStation` (temperature, pressure and humidity at the gun, with
  vapour on its own 2 km scale height so it thins with altitude rather than tracking pressure).
- **`WindField`**: `Uniform`, `Sounding` with arbitrary levels, interpolating speed and bearing so
  a veering wind keeps its speed.
- **`DragModel`**: `StandardDrag` over G1, G7 and an artillery-shell curve; `CustomDrag` from a
  measured Mach/Cd table; optional quadratic yaw drag.
- **`Integrator`**: RK2, RK4, and Dormand-Prince 5(4) with step control. On MPMM the adaptive
  scheme takes 781 steps against 155,760 for the finest fixed step, and is more accurate with it.

`SolverConfig::diagnostics` is an optional per-step callback. `rangeTable(...)` sweeps range and
solves each row; `examples/table` prints one for a standard day beside a cold windy one.

Swapping any one piece changes results in the expected direction and needs no change elsewhere.
`tests/mm_test_swap.cpp` asserts it.

## Conventions

Identical to `ballistics-pointmass`: local left-handed frame with `+x` downrange, `+y` up, `+z`
crossrange right; metres, seconds, kilograms, radians internally; degrees only for compass
bearings. Moving a caller between the two libraries is a change of namespace and a model choice,
not of physics. `tests/mm_test_crosslibrary.cpp` checks this model against a file recorded by that
library and finds agreement to 7.6e-14 on range and time of flight.

`C_M_alpha` sign convention: positive is overturning, so a spin-stabilised shell is positive and
held by its spin, a fin-stabilised body negative and statically stable.

## Accuracy, honestly

**Sound.** Range and time of flight for all nine catalogue rounds, each reproducing the published
maximum range it was fitted to within 0.5%. The integrators, to the tolerance you ask for. The
rigid body's rotational dynamics: torque-free angular momentum is conserved to 5e-14 over a 50 s
flight, which `tests/mm_test_conservation.cpp` asserts.

**Approximate.** Drift. The form is right and MPMM and the rigid body agree on it independently,
but it rests on class-typical coefficients. `C_M_alpha` is bounded to 3.5 to 4.0 by the gyroscopic
stability the round must have, so it is not a free parameter, but bounded is weaker than known.
Expect the right order and sign, not a firing-table number.

**Not modelled.** Motor burn and its mass loss, so rocket-assisted rounds are absent rather than
present and wrong. Fin aerodynamics beyond a static restoring moment. Base bleed except as a drag
multiplier. Anything that manoeuvres.

**This is not firing-table data and the library is not validated for operational fire control.**

`docs/catalogue-review.md` records every number and its source. `docs/model-comparison.md` carries
the measured cost and accuracy of each model. `docs/extensibility.md` records what a declared input
contract measured against this library.


## Status

Complete and verified: 13 tests, 11,854 assertions, published and frozen. `main` only advances
through a pull request. No further development is planned unless something turns out to be wrong.
