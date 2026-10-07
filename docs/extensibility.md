# Extensibility

This library takes its inputs as C++ structs. That is the right surface for a library this size,
but a struct is not a contract: it cannot be read by another language, it cannot gain a field
without recompiling every caller, and it carries no way to say "this input came from somewhere you
should not trust".

A declared input contract solves all three. This document records that one exists, that it was
built and measured against this library, and what it cost. **The contract and the layer that
consumes it are not part of this repository.** What follows is the result, not the mechanism.

## What was built

A protobuf schema covering everything this library consumes: projectile geometry, muzzle
conditions, drag as either a standard family with a form factor or a measured Mach/Cd table,
optional motor, fin, inertia and moment-coefficient blocks, atmosphere, wind as uniform or as an
arbitrary-level sounding, solver configuration, and the problem to solve. 31 messages and 8 enums.

On top of it, a layer that maps a message onto this library's API, runs it, and returns the result
as a message.

## What it measured

| | |
|---|---|
| Complete fire mission on the wire | **271 bytes** |
| Result on the wire | 134 bytes |
| Parse a mission from wire | 0.74 us |
| Map it onto this library's types | 0.15 us |
| Overhead against a 96.6 ms solve | **0.001%** |

The overhead is noise. Nothing about taking inputs as a declared message costs anything that
matters next to integrating a trajectory.

Three properties were verified rather than assumed:

- **Fidelity.** A mission expressed in the contract produces *bit-identical* results to the same
  inputs passed through this library's own API. Elevation, time of flight, absolute bearing and
  range all compare at exactly zero tolerance, not within a threshold.
- **Forward compatibility.** A reader built against an older copy of the schema ignores fields it
  does not know and parses everything it does. That is the property a C++ header cannot offer, and
  the reason the contract exists at all.
- **Refusal.** This library reports which data block a model requires and a projectile lacks. That
  travels through the contract intact: a mission missing moments of inertia comes back refused,
  naming the inertia block, rather than being solved against zeros.

## What it enables that this library deliberately does not

**A target given as two known positions.** This library solves for a target expressed as an offset
in the launch-centred frame. Turning a gun position and a target position into an elevation and a
true bearing needs the geodesy, the launch azimuth set from the bearing, the surface leg separated
from the slant range, and the solver's azimuth offset converted back to an absolute bearing. That
assembly is roughly fifty lines and it is not here, on purpose: this library stays small. Built on
the contract, it solves both arcs and closes the loop to within 8.2 m of the target, checked by
projecting the trajectory back to geodetic coordinates.

**A catalogue as data.** The eighteen built-in rounds are compiled in. Expressed as messages they
become a file, extensible without touching the library, and each round can carry its own
provenance: where every number came from and how far it can be trusted.

**Changing the model with one field.** Switching a mission between the point mass, MPMM and the
rigid body is a single enumerator, not a recompile. A mission that asks the rigid body for a step
it cannot resolve is refused through the contract with that reason, rather than answered.

## Why it is not in this repository

This library is finished. It is small, self-contained, verified, and frozen, and those properties
are worth more than any feature that could be added to it. The contract and the composition layer
are still moving, and mixing the two would cost the library exactly what makes it useful.

The split is deliberate and it holds for anything added later: this repository receives results and
statements, never the framework.
