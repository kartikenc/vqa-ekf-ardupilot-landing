# E1 (flow ON) vs E3 (flow OFF) — per-arm comparison (computed 23 Nov 2026 from per_sortie.csv of both frozen runs)

Observational: arms were randomised within each experiment, but E1 and E3 were flown on different days and light.
E1 touchdown dy was recorded WITHOUT SIGN (0 negatives in 45; E3 has 15) -> E1 centroid, spread and precision tests are invalid.
Rotation-free measures below are valid for both.

| Arm | Camera-point radial E1 -> E3 (median, cm) | p | After 11 cm offset | p | Blind drift | p |
|---|---|---|---|---|---|---|
| A 20 cm | 13.66 -> 13.46 | 0.84 | 4.47 -> 3.04 | 0.84 | 4.15 -> 2.41 | 0.062 |
| A' 1.14 cm | 13.65 -> 13.54 | 0.68 | 4.00 -> 3.45 | 0.92 | 3.44 -> 3.06 | 0.74 |
| B adaptive | 12.54 -> 11.77 | 0.078 | 3.16 -> 2.55 | 0.23 | 2.56 -> 1.43 | 0.013 |

- Turning optical flow off did not worsen landing in any arm.
- Position innovation RMS fell in every arm (A 1.63 -> 1.02 cm, p < 0.001).
- Frames accepted rose 93-95 % -> 96-97 % (E3 mostly flown inside the window).
- Interaction (B minus reference, E3 vs E1): B-A change -0.58 cm, 95 % CI [-5.75, +2.61]; B-A' change -0.66, CI [-5.89, +1.95].

## Post-hoc non-inferiority (90 % bootstrap CI of median difference B - A')
- E1 camera point: -1.11 [-1.81, +2.51]; controlled point: -0.84 [-2.67, +1.59]
- E3 camera point: -1.77 [-4.21, +0.11]; controlled point: -0.90 [-4.00, +0.51]
- With a 3 cm margin both experiments pass. The margin was chosen after the data: label as post hoc.
