# Catalogue

Nine rounds. Every one is calibrated, and a round that could not be anchored to a published figure
was left out rather than shipped with an estimate standing in for data.

## What is measured and what is derived

**Measured**, from published specifications: mass, diameter, length, muzzle velocity, and the
maximum range each round is quoted as reaching.

**Fitted**, once, to that maximum range: the drag form factor, by bisection.

**Derived**, from documented relations in `tools/generate_catalogue.py`: everything else.

| quantity | relation |
|---|---|
| spin rate | `60 V / (20 d)`, for the 1:20 calibre twist of a 155 mm tube |
| axial inertia | `0.1391 m d^2`, the measured M107 ratio |
| transverse inertia | `(m/12)(3 r^2 + L^2)`, uniform cylinder, within 4% of the measured M107 figure |
| spin decay halflife | roll damping at `C_l_p = -0.015`, evaluated at 0.6 of muzzle speed |
| `C_M_alpha`, `C_L_alpha` | 3.7 and 1.6, class-typical for a shell, and bounded as below |

`tools/rounds.csv` holds only the measured and fitted values. The generator produces
`src/catalogue.cpp` from it, and a test asserts the committed file still matches, so no number in
the catalogue is a literal nobody can trace.

## The rounds, and where the range figure comes from

| round | muzzle | published max range | source | fitted Cd at rest |
|---|---|---|---|---|
| 60mm M720 HE | 241 m/s | 3,490 m | M224 mortar | 0.221 |
| 60mm M722 Smoke WP | 225 m/s | 3,200 m | M224, zone 4 | 0.187 |
| 81mm M821 HE | 341 m/s | 5,650 m | M252 / L16 | 0.234 |
| 81mm M819 Smoke RP | 297 m/s | 4,900 m | M252, zone 4 | 0.233 |
| 81mm M853A1 Illumination | 285 m/s | 5,050 m | FM 23-90 | 0.216 |
| 120mm M933 HE | 332 m/s | 7,200 m | M120 mortar | 0.185 |
| 120mm M929 Smoke WP | 340 m/s | 7,200 m | M120, zone 4 | 0.205 |
| 155mm M795 HE | 827 m/s | 22,400 m | 39 calibre tube | 0.147 |
| 155mm M825 Smoke WP | 684 m/s | 18,000 m | effective range | 0.141 |

Each reproduces its figure to better than 0.5%, which `tests/test_catalogue.cpp` asserts.

## The fit checks itself

A form factor fitted to a range can absorb an error in the muzzle velocity instead of describing
the drag, and the result looks fine. The implied Cd at rest is the tell: a mortar bomb sits near
0.15 to 0.30, a shell near 0.13 to 0.20. Anything outside that means the velocity and the range do
not belong to the same charge.

That bound removed a round. A 60 mm illumination cartridge quoted at 3,500 m and 198 m/s fits to a
Cd at rest of **0.042**, about a fifth of anything physical. The quoted figure is an employment
envelope rather than a ballistic maximum at that velocity, so the round is absent.

The 155 mm fit is worth stating as corroboration rather than just a number: it lands at 0.147
subsonic, 0.382 at the transonic peak, and 0.29 at Mach 2.4. All three sit inside the published
band for a 155 mm HE shell, which they would not if the fit were quietly compensating for something.

## Drag shape

The form factor scales a reference curve, and the curve has to have the right shape or the scaling
cannot rescue it. Normalised to each family's own value at Mach 0:

| Mach | shell | G1 | G7 |
|---|---|---|---|
| 0.90 | 1.22 | 1.04 | 1.00 |
| 1.00 | 2.06 | 1.28 | 1.00 |
| 1.20 | **2.59** | 1.70 | 1.00 |
| 2.00 | 2.12 | **2.09** | 1.28 |
| 3.00 | 1.78 | 2.03 | 2.21 |

A shell's drag peaks just above Mach 1 at roughly 2.6 times its subsonic value. G7 is nearly flat
until Mach 2 and G1 peaks around Mach 2, so neither shape suits a shell: `DragFamily::Shell` is an
M107-family curve and the 155 mm rounds use it. Mortar bombs spend their flight subsonic and
transonic, where G1 is adequate.

## Stabilisation

Only the two rifled 155 mm rounds are spin-stabilised, and they alone carry a spin rate and the
drift term. Mortar bombs are fin-stabilised and develop no gyroscopic yaw of repose, so applying a
repose-derived drift to one is not a small error but the wrong physics.

The sign convention follows from that: `C_M_alpha` positive is overturning, so a spin-stabilised
round is positive and held by its spin, a fin-stabilised one negative and statically stable.

## C_M_alpha is bounded, not chosen

Gyroscopic stability pins it. With the derived inertias,

```
Sg = Ix^2 p^2 / (2 rho Iy S d V^2 C_M_alpha)
```

must exceed 1 or the round tumbles, and artillery is designed near 1.3 to 2.0 at the muzzle. Since
spin is proportional to velocity for a fixed twist, `V` cancels and `Sg` depends only on geometry,
inertia and `C_M_alpha`:

| `C_M_alpha` | Sg at the muzzle |
|---|---|
| 2.5 | 1.91 |
| 3.0 | 1.59 |
| **3.7** | **1.29** |
| 4.5 | 1.06 |
| 8.0 | 0.60 — tumbles |

So it is boxed into roughly 3.5 to 4.0. That matters beyond tidiness: it means drift cannot be
tuned to taste. `tests/test_catalogue.cpp` asserts `1 < Sg < 3` for every round carrying spin
aerodynamics.

## What this catalogue is not

- **Not firing-table data.** Range and time of flight are anchored. The aerodynamic coefficients
  are class-typical rather than measured per round, and bounded is a weaker claim than known.
- **No motor is modelled.** Rocket-assisted rounds are absent rather than present and wrong, since
  their published range is achieved with a motor this does not simulate.
- **No round that could not be anchored is present.** Rocket-assisted projectiles, generic
  extended-range designations, smoothbore tank rounds whose quoted figure is an engagement range
  rather than a ballistic one, and the illumination round the plausibility bound rejected.

If you hold measured aeroballistic data, `Projectile::spinAero` and the drag model are per-round
inputs for exactly this reason: replace them without touching the library.

## Sources

- [M795 projectile](https://en.wikipedia.org/wiki/M795_projectile),
  [globalsecurity M795](https://www.globalsecurity.org/military/systems/munitions/m795.htm)
- [M825 155mm projectile](https://www.globalsecurity.org/military/systems/munitions/m825.htm)
- [M224 60mm mortar](https://en.wikipedia.org/wiki/M224_mortar),
  [M722 60mm smoke](https://www.globalsecurity.org/military/systems/munitions/m722.htm)
- [M252 81mm mortar](https://en.wikipedia.org/wiki/M252_mortar),
  [L16 81mm mortar](https://en.wikipedia.org/wiki/L16_81mm_mortar),
  [M821/M889](https://www.globalsecurity.org/military/systems/munitions/m821.htm)
- [M819 81mm smoke](https://man.fas.org/dod-101/sys/land/m819.htm),
  [M853A1 illumination](https://man.fas.org/dod-101/sys/land/m816.htm),
  [FM 23-90 chapter 4](https://www.globalsecurity.org/military/library/policy/army/fm/23-90/ch4.htm)
- [XM929 120mm smoke](https://man.fas.org/dod-101/sys/land/m929.htm),
  [XM931 120mm](https://man.fas.org/dod-101/sys/land/m931.htm)
- [Yaw of repose](http://ffden-2.phys.uaf.edu/212fall2001_Web_projects/Isaac%20Rowland/Ballistics/Bulletflight/longr.htm),
  [external ballistics](https://en.wikipedia.org/wiki/External_ballistics)
