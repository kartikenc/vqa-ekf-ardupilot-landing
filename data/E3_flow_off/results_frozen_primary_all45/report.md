# VQA-EKF flight experiment E3 — automated report
_analyze_vqa_ekf_final.py v1.1_

## Data
- Sorties found: 45; valid for the tests: 45 (A 15, A′ 15, B 15)
- Script versions: v7.3; camera: rgb; A′ σ: 0.0114 m; VQA models used: 1 (OK, one frozen model)
- DataFlash logs matched and verified: 45/45
- Flight-controller parameters identical in every log: yes
- Pad grid vs EKF frame: auto: R270 (pad X = +world Y(E), pad Y = -world X(N)); residual 1.17 cm vs next best 5.22 cm (F0)
- Common reference offset (pad frame): (+11.0, +0.0) cm — measured (--ref-offset-cm)
- Data-quality flags: E3_S23: EKF failsafe; E3_S35: maneuver frames <90%; E3_S36: maneuver frames <90%; E3_S37: maneuver frames <90%; E3_S38: maneuver frames <90%; E3_S39: maneuver frames <90%; E3_S40: maneuver frames <90%; E3_S41: EKF failsafe; maneuver frames <90%; E3_S44: EKF failsafe; E3_S45: EKF failsafe

## H0 — Manipulation check (did EKF3 use what each arm intended?)
- A: constant 20 cm: median 20.00 cm, p95 20.00 cm in the maneuver [VISP.PErr (what EKF3 used)] → **PASS**
- A': tuned constant: median 1.14 cm, p95 1.14 cm in the maneuver [VISP.PErr (what EKF3 used)] → **PASS**
- B: VQA-EKF: median 1.14 cm, p95 1.34 cm in the maneuver, ρ(σ, tilt) = 0.65, at the 1 cm floor 0% of frames, sent = model on 100.0% of frames [VISP.PErr (what EKF3 used)] → **PASS**
  - Size of the adaptation: B's σ p95/median ×1.18 in the maneuver; B median 1.14 cm vs A′ 1.14 cm

## H1 / H2 — Touchdown (pad-grid truth)
Primary outcome as pre-registered: radial error from the marker centre.
- **H1** B vs A: median diff -1.69 (95% CI -5.00 to +0.85), Cliff's δ -0.33, Mann-Whitney p=0.130, Holm p=0.130, n=15/15 → **NOT SUPPORTED at α=0.05**
- **H2** B vs A': median diff -1.77 (95% CI -4.56 to +0.78), Cliff's δ -0.44, Mann-Whitney p=0.044, Holm p=0.088, n=15/15 → **NOT SUPPORTED at α=0.05**
- Omnibus Kruskal–Wallis across the three arms: p=0.112

Per arm (median [95% bootstrap CI]):
- A: constant 20 cm: CEP50 13.5 [11.5, 17.1] cm, CEP95 18.8 cm; centroid (+13.5, -0.2) cm; spread about centroid RMS 4.5 cm; after reference offset CEP50 3.0 [2.5, 6.3] cm
- A': tuned constant: CEP50 13.5 [12.0, 16.1] cm, CEP95 18.2 cm; centroid (+13.5, -0.8) cm; spread about centroid RMS 4.1 cm; after reference offset CEP50 3.4 [1.1, 6.4] cm
- B: VQA-EKF: CEP50 11.8 [10.0, 13.6] cm, CEP95 15.0 cm; centroid (+11.6, +0.2) cm; spread about centroid RMS 2.6 cm; after reference offset CEP50 2.5 [1.0, 4.0] cm

Offset-free comparisons (a common reference offset cancels in these):
- Distance between 2-D touchdown centroids (offset-free), B vs A: centroid distance 1.90 cm, permutation p=0.159, n=15/15
- Distance between 2-D touchdown centroids (offset-free), B vs A': centroid distance 2.14 cm, permutation p=0.063, n=15/15
- Distance to the arm's own touchdown centroid (precision), B vs A: median diff -1.87 (95% CI -3.14 to +0.67), Cliff's δ -0.46, Mann-Whitney p=0.034, n=15/15
- Distance to the arm's own touchdown centroid (precision), B vs A': median diff -1.13 (95% CI -2.58 to +0.62), Cliff's δ -0.36, Mann-Whitney p=0.097, n=15/15
- Touchdown radial error after the reference offset, B vs A: median diff -0.49 (95% CI -4.52 to +0.70), Cliff's δ -0.32, Mann-Whitney p=0.134, n=15/15
- Touchdown radial error after the reference offset, B vs A': median diff -0.90 (95% CI -4.56 to +1.14), Cliff's δ -0.38, Mann-Whitney p=0.081, n=15/15

## Where the touchdown error comes from
- A: constant 20 cm: at the last camera fix (0.09 m, 0.8 s before touchdown) the EKF-controlled point was 2.4 cm from the marker; blind-segment drift 2.4 cm (median)
- A': tuned constant: at the last camera fix (0.11 m, 0.9 s before touchdown) the EKF-controlled point was 2.2 cm from the marker; blind-segment drift 3.1 cm (median)
- B: VQA-EKF: at the last camera fix (0.12 m, 0.8 s before touchdown) the EKF-controlled point was 2.6 cm from the marker; blind-segment drift 1.4 cm (median)

## H3 — Filter consistency (position NIS, ideal mean 2.0; from the .bin)
- A: constant 20 cm: mean NIS 0.00 (all frames pooled; median of per-sortie means 0.00), fraction > χ²₂(0.95)=5.99: 0.000 (ideal 0.05) — R larger than the real error (conservative)
- A': tuned constant: mean NIS 0.37 (all frames pooled; median of per-sortie means 0.09), fraction > χ²₂(0.95)=5.99: 0.008 (ideal 0.05) — R larger than the real error (conservative)
- B: VQA-EKF: mean NIS 0.10 (all frames pooled; median of per-sortie means 0.08), fraction > χ²₂(0.95)=5.99: 0.001 (ideal 0.05) — R larger than the real error (conservative)

## Secondary outcomes (exploratory, not adjusted for multiplicity)
- Touchdown dx (pad X), B vs A: median diff -1.00 (95% CI -5.00 to +1.00), Cliff's δ -0.32, Mann-Whitney p=0.145, n=15/15
- Touchdown dx (pad X), B vs A': median diff -2.00 (95% CI -4.30 to +0.50), Cliff's δ -0.39, Mann-Whitney p=0.074, n=15/15
- Touchdown dy (pad Y), B vs A: median diff +0.00 (95% CI -1.50 to +1.50), Cliff's δ -0.04, Mann-Whitney p=0.882, n=15/15
- Touchdown dy (pad Y), B vs A': median diff -0.50 (95% CI -1.00 to +3.00), Cliff's δ +0.08, Mann-Whitney p=0.721, n=15/15
- EKF-controlled point vs marker at the last camera fix, B vs A: median diff +0.20 (95% CI -1.44 to +1.87), Cliff's δ +0.06, Mann-Whitney p=0.803, n=15/15
- EKF-controlled point vs marker at the last camera fix, B vs A': median diff +0.41 (95% CI -1.73 to +1.84), Cliff's δ -0.00, Mann-Whitney p=1.000, n=15/15
- Blind-segment drift (last fix -> touchdown), B vs A: median diff -0.98 (95% CI -2.43 to +0.19), Cliff's δ -0.38, Mann-Whitney p=0.081, n=15/15
- Blind-segment drift (last fix -> touchdown), B vs A': median diff -1.63 (95% CI -2.88 to -0.35), Cliff's δ -0.43, Mann-Whitney p=0.046, n=15/15
- Overshoot after the last step, B vs A: median diff -0.28 (95% CI -2.93 to +1.01), Cliff's δ -0.22, Mann-Whitney p=0.320, n=15/15
- Overshoot after the last step, B vs A': median diff -0.06 (95% CI -2.37 to +1.37), Cliff's δ -0.06, Mann-Whitney p=0.803, n=15/15
- Settling time to 5 cm, B vs A: median diff +0.10 (95% CI -7.31 to +1.81), Cliff's δ +0.21, Mann-Whitney p=0.348, n=15/14
- Settling time to 5 cm, B vs A': median diff +0.02 (95% CI -6.28 to +1.71), Cliff's δ -0.02, Mann-Whitney p=0.959, n=15/11
- Hold RMS in CONVERGE, B vs A: median diff -0.51 (95% CI -2.60 to -0.13), Cliff's δ -0.48, Mann-Whitney p=0.028, n=15/15
- Hold RMS in CONVERGE, B vs A': median diff -0.24 (95% CI -1.20 to +0.33), Cliff's δ -0.14, Mann-Whitney p=0.534, n=15/15
- Mean position NIS (ideal 2), B vs A: median diff +0.08 (95% CI +0.07 to +0.09), Cliff's δ +1.00, Mann-Whitney p=0.000, n=15/15
- Mean position NIS (ideal 2), B vs A': median diff -0.01 (95% CI -0.03 to +0.01), Cliff's δ -0.14, Mann-Whitney p=0.534, n=15/15
- Position innovation RMS, B vs A: median diff -0.74 (95% CI -0.77 to -0.61), Cliff's δ -0.98, Mann-Whitney p=0.000, n=15/15
- Position innovation RMS, B vs A': median diff +0.01 (95% CI -0.05 to +0.11), Cliff's δ +0.24, Mann-Whitney p=0.263, n=15/15

## Validity
- Max maneuver tilt, median over sorties: 18.9 deg (range 18.3–19.6; target 15–22)
- Sorties without touchdown truth: 0
- Not completed: 0 (report them; do not silently drop)