# VQA-EKF flight experiment E3 — automated report
_analyze_vqa_ekf_final.py v1.1_

## Data
- Sorties found: 45; valid for the tests: 38 (A 13, A′ 13, B 12)
- Script versions: v7.3; camera: rgb; A′ σ: 0.0114 m; VQA models used: 1 (OK, one frozen model)
- DataFlash logs matched and verified: 45/45
- Flight-controller parameters identical in every log: yes
- Pad grid vs EKF frame: auto: R270 (pad X = +world Y(E), pad Y = -world X(N)); residual 1.24 cm vs next best 5.34 cm (F0)
- Common reference offset (pad frame): (+11.0, +0.0) cm — measured (--ref-offset-cm)
- Excluded: E3_S35 (Excluded-glare), E3_S36 (Excluded-glare), E3_S37 (Excluded-glare), E3_S38 (Excluded-glare), E3_S39 (Excluded-glare), E3_S40 (Excluded-glare), E3_S41 (Excluded-glare)
- Data-quality flags: E3_S23: EKF failsafe; E3_S35: maneuver frames <90%; E3_S36: maneuver frames <90%; E3_S37: maneuver frames <90%; E3_S38: maneuver frames <90%; E3_S39: maneuver frames <90%; E3_S40: maneuver frames <90%; E3_S41: EKF failsafe; maneuver frames <90%; E3_S44: EKF failsafe; E3_S45: EKF failsafe

## H0 — Manipulation check (did EKF3 use what each arm intended?)
- A: constant 20 cm: median 20.00 cm, p95 20.00 cm in the maneuver [VISP.PErr (what EKF3 used)] → **PASS**
- A': tuned constant: median 1.14 cm, p95 1.14 cm in the maneuver [VISP.PErr (what EKF3 used)] → **PASS**
- B: VQA-EKF: median 1.15 cm, p95 1.33 cm in the maneuver, ρ(σ, tilt) = 0.66, at the 1 cm floor 0% of frames, sent = model on 100.0% of frames [VISP.PErr (what EKF3 used)] → **PASS**
  - Size of the adaptation: B's σ p95/median ×1.15 in the maneuver; B median 1.15 cm vs A′ 1.14 cm

## H1 / H2 — Touchdown (pad-grid truth)
Primary outcome as pre-registered: radial error from the marker centre.
- **H1** B vs A: median diff -1.42 (95% CI -5.22 to +1.62), Cliff's δ -0.22, Mann-Whitney p=0.355, Holm p=0.710, n=12/13 → **NOT SUPPORTED at α=0.05**
- **H2** B vs A': median diff -1.46 (95% CI -3.46 to +1.97), Cliff's δ -0.21, Mann-Whitney p=0.384, Holm p=0.710, n=12/13 → **NOT SUPPORTED at α=0.05**
- Omnibus Kruskal–Wallis across the three arms: p=0.560

Per arm (median [95% bootstrap CI]):
- A: constant 20 cm: CEP50 13.5 [11.9, 17.1] cm, CEP95 19.2 cm; centroid (+13.6, -0.2) cm; spread about centroid RMS 4.8 cm; after reference offset CEP50 4.0 [2.5, 6.3] cm
- A': tuned constant: CEP50 13.5 [11.0, 15.2] cm, CEP95 18.3 cm; centroid (+13.2, -0.7) cm; spread about centroid RMS 4.2 cm; after reference offset CEP50 2.7 [1.1, 6.1] cm
- B: VQA-EKF: CEP50 12.0 [11.2, 14.0] cm, CEP95 15.0 cm; centroid (+12.4, +0.2) cm; spread about centroid RMS 2.1 cm; after reference offset CEP50 2.2 [1.0, 3.6] cm

Offset-free comparisons (a common reference offset cancels in these):
- Distance between 2-D touchdown centroids (offset-free), B vs A: centroid distance 1.28 cm, permutation p=0.532, n=12/13
- Distance between 2-D touchdown centroids (offset-free), B vs A': centroid distance 1.25 cm, permutation p=0.491, n=12/13
- Distance to the arm's own touchdown centroid (precision), B vs A: median diff -2.14 (95% CI -3.60 to -0.06), Cliff's δ -0.68, Mann-Whitney p=0.004, n=12/13
- Distance to the arm's own touchdown centroid (precision), B vs A': median diff -1.15 (95% CI -3.10 to +0.72), Cliff's δ -0.44, Mann-Whitney p=0.068, n=12/13
- Touchdown radial error after the reference offset, B vs A: median diff -1.82 (95% CI -5.18 to +0.38), Cliff's δ -0.46, Mann-Whitney p=0.056, n=12/13
- Touchdown radial error after the reference offset, B vs A': median diff -0.52 (95% CI -4.70 to +1.43), Cliff's δ -0.37, Mann-Whitney p=0.127, n=12/13

## Where the touchdown error comes from
- A: constant 20 cm: at the last camera fix (0.09 m, 0.8 s before touchdown) the EKF-controlled point was 2.7 cm from the marker; blind-segment drift 2.6 cm (median)
- A': tuned constant: at the last camera fix (0.09 m, 0.8 s before touchdown) the EKF-controlled point was 2.2 cm from the marker; blind-segment drift 3.1 cm (median)
- B: VQA-EKF: at the last camera fix (0.12 m, 0.9 s before touchdown) the EKF-controlled point was 2.4 cm from the marker; blind-segment drift 1.6 cm (median)

## H3 — Filter consistency (position NIS, ideal mean 2.0; from the .bin)
- A: constant 20 cm: mean NIS 0.00 (all frames pooled; median of per-sortie means 0.00), fraction > χ²₂(0.95)=5.99: 0.000 (ideal 0.05) — R larger than the real error (conservative)
- A': tuned constant: mean NIS 0.08 (all frames pooled; median of per-sortie means 0.08), fraction > χ²₂(0.95)=5.99: 0.000 (ideal 0.05) — R larger than the real error (conservative)
- B: VQA-EKF: mean NIS 0.11 (all frames pooled; median of per-sortie means 0.08), fraction > χ²₂(0.95)=5.99: 0.001 (ideal 0.05) — R larger than the real error (conservative)

## Secondary outcomes (exploratory, not adjusted for multiplicity)
- Touchdown dx (pad X), B vs A: median diff -0.50 (95% CI -5.25 to +1.50), Cliff's δ -0.19, Mann-Whitney p=0.428, n=12/13
- Touchdown dx (pad X), B vs A': median diff -1.50 (95% CI -3.25 to +1.75), Cliff's δ -0.17, Mann-Whitney p=0.494, n=12/13
- Touchdown dy (pad Y), B vs A: median diff +0.00 (95% CI -2.00 to +2.25), Cliff's δ -0.01, Mann-Whitney p=1.000, n=12/13
- Touchdown dy (pad Y), B vs A': median diff -0.50 (95% CI -0.75 to +3.00), Cliff's δ +0.07, Mann-Whitney p=0.784, n=12/13
- EKF-controlled point vs marker at the last camera fix, B vs A: median diff -0.26 (95% CI -2.02 to +1.39), Cliff's δ -0.13, Mann-Whitney p=0.605, n=12/13
- EKF-controlled point vs marker at the last camera fix, B vs A': median diff +0.23 (95% CI -2.27 to +1.73), Cliff's δ -0.10, Mann-Whitney p=0.683, n=12/13
- Blind-segment drift (last fix -> touchdown), B vs A: median diff -0.97 (95% CI -2.65 to +0.28), Cliff's δ -0.35, Mann-Whitney p=0.149, n=12/13
- Blind-segment drift (last fix -> touchdown), B vs A': median diff -1.47 (95% CI -2.91 to -0.22), Cliff's δ -0.41, Mann-Whitney p=0.087, n=12/13
- Overshoot after the last step, B vs A: median diff -0.48 (95% CI -4.72 to +1.77), Cliff's δ -0.32, Mann-Whitney p=0.183, n=12/13
- Overshoot after the last step, B vs A': median diff -0.00 (95% CI -1.51 to +2.31), Cliff's δ +0.08, Mann-Whitney p=0.765, n=12/13
- Settling time to 5 cm, B vs A: median diff +0.08 (95% CI -13.46 to +5.28), Cliff's δ +0.11, Mann-Whitney p=0.665, n=12/12
- Settling time to 5 cm, B vs A': median diff +0.04 (95% CI -6.22 to +5.76), Cliff's δ +0.03, Mann-Whitney p=0.926, n=12/11
- Hold RMS in CONVERGE, B vs A: median diff -0.60 (95% CI -2.97 to -0.11), Cliff's δ -0.50, Mann-Whitney p=0.036, n=12/13
- Hold RMS in CONVERGE, B vs A': median diff -0.09 (95% CI -1.21 to +0.47), Cliff's δ -0.04, Mann-Whitney p=0.892, n=12/13
- Mean position NIS (ideal 2), B vs A: median diff +0.08 (95% CI +0.07 to +0.09), Cliff's δ +1.00, Mann-Whitney p=0.000, n=12/13
- Mean position NIS (ideal 2), B vs A': median diff -0.00 (95% CI -0.02 to +0.01), Cliff's δ +0.00, Mann-Whitney p=1.000, n=12/13
- Position innovation RMS, B vs A: median diff -0.75 (95% CI -0.80 to -0.68), Cliff's δ -0.97, Mann-Whitney p=0.000, n=12/13
- Position innovation RMS, B vs A': median diff +0.03 (95% CI -0.04 to +0.08), Cliff's δ +0.29, Mann-Whitney p=0.221, n=12/13

## Validity
- Max maneuver tilt, median over sorties: 18.9 deg (range 18.3–19.6; target 15–22)
- Sorties without touchdown truth: 0
- Not completed: 0 (report them; do not silently drop)