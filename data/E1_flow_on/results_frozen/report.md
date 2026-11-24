# VQA-EKF flight experiment E1 — automated report
_analyze_vqa_ekf_final.py v1.1_

## Data
- Sorties found: 45; valid for the tests: 45 (A 15, A′ 15, B 15)
- Script versions: v7.1; camera: rgb; A′ σ: 0.0114 m; VQA models used: 1 (OK, one frozen model)
- DataFlash logs matched and verified: 45/45
- Flight-controller parameters identical in every log: yes
- Pad grid vs EKF frame: auto: R270 (pad X = +world Y(E), pad Y = -world X(N)); residual 3.69 cm vs next best 4.50 cm (F90)
- Common reference offset (pad frame): (+11.0, +0.0) cm — measured (--ref-offset-cm)
- Data-quality flags: E1_S28: maneuver frames <90%; E1_S32: maneuver frames <90%; E1_S33: maneuver frames <90%; E1_S34: maneuver frames <90%; E1_S35: maneuver frames <90%; E1_S36: maneuver frames <90%; E1_S37: maneuver frames <90%; E1_S38: maneuver frames <90%; E1_S39: maneuver frames <90%; E1_S40: maneuver frames <90%; E1_S41: maneuver frames <90%; E1_S42: maneuver frames <90%; E1_S43: maneuver frames <90%; E1_S44: maneuver frames <90%; E1_S45: maneuver frames <90%

## H0 — Manipulation check (did EKF3 use what each arm intended?)
- A: constant 20 cm: median 20.00 cm, p95 20.00 cm in the maneuver [VISP.PErr (what EKF3 used)] → **PASS**
- A': tuned constant: median 1.14 cm, p95 1.14 cm in the maneuver [VISP.PErr (what EKF3 used)] → **PASS**
- B: VQA-EKF: median 1.08 cm, p95 1.32 cm in the maneuver, ρ(σ, tilt) = 0.51, at the 1 cm floor 8% of frames, sent = model on 100.0% of frames [VISP.PErr (what EKF3 used)] → **PASS**
  - Size of the adaptation: B's σ p95/median ×1.23 in the maneuver; B median 1.08 cm vs A′ 1.14 cm

## H1 / H2 — Touchdown (pad-grid truth)
Primary outcome as pre-registered: radial error from the marker centre.
- **H1** B vs A: median diff -1.12 (95% CI -2.76 to +3.50), Cliff's δ +0.02, Mann-Whitney p=0.950, Holm p=1.000, n=15/15 → **NOT SUPPORTED at α=0.05**
- **H2** B vs A': median diff -1.11 (95% CI -2.04 to +2.96), Cliff's δ +0.02, Mann-Whitney p=0.950, Holm p=1.000, n=15/15 → **NOT SUPPORTED at α=0.05**
- Omnibus Kruskal–Wallis across the three arms: p=0.993

Per arm (median [95% bootstrap CI]):
- A: constant 20 cm: CEP50 13.7 [11.2, 15.1] cm, CEP95 16.8 cm; centroid (+12.8, +2.6) cm; spread about centroid RMS 3.3 cm; after reference offset CEP50 4.5 [2.5, 6.0] cm
- A': tuned constant: CEP50 13.6 [11.7, 14.1] cm, CEP95 16.0 cm; centroid (+12.9, +2.7) cm; spread about centroid RMS 2.6 cm; after reference offset CEP50 4.0 [3.0, 4.7] cm
- B: VQA-EKF: CEP50 12.5 [12.0, 16.2] cm, CEP95 18.4 cm; centroid (+13.3, +1.6) cm; spread about centroid RMS 3.6 cm; after reference offset CEP50 3.2 [1.4, 5.8] cm

Offset-free comparisons (a common reference offset cancels in these):
- Distance between 2-D touchdown centroids (offset-free), B vs A: centroid distance 1.04 cm, permutation p=0.526, n=15/15
- Distance between 2-D touchdown centroids (offset-free), B vs A': centroid distance 1.13 cm, permutation p=0.408, n=15/15
- Distance to the arm's own touchdown centroid (precision), B vs A: median diff -0.21 (95% CI -1.95 to +1.21), Cliff's δ -0.14, Mann-Whitney p=0.534, n=15/15
- Distance to the arm's own touchdown centroid (precision), B vs A': median diff +0.59 (95% CI -0.61 to +2.22), Cliff's δ +0.29, Mann-Whitney p=0.184, n=15/15
- Touchdown radial error after the reference offset, B vs A: median diff -1.31 (95% CI -3.90 to +1.36), Cliff's δ -0.22, Mann-Whitney p=0.319, n=15/15
- Touchdown radial error after the reference offset, B vs A': median diff -0.84 (95% CI -2.91 to +2.10), Cliff's δ -0.16, Mann-Whitney p=0.480, n=15/15

## Where the touchdown error comes from
- A: constant 20 cm: at the last camera fix (0.15 m, 1.0 s before touchdown) the EKF-controlled point was 3.6 cm from the marker; blind-segment drift 4.1 cm (median)
- A': tuned constant: at the last camera fix (0.12 m, 0.9 s before touchdown) the EKF-controlled point was 2.0 cm from the marker; blind-segment drift 3.4 cm (median)
- B: VQA-EKF: at the last camera fix (0.15 m, 1.1 s before touchdown) the EKF-controlled point was 2.6 cm from the marker; blind-segment drift 2.6 cm (median)

## H3 — Filter consistency (position NIS, ideal mean 2.0; from the .bin)
- A: constant 20 cm: mean NIS 0.01 (all frames pooled; median of per-sortie means 0.01), fraction > χ²₂(0.95)=5.99: 0.000 (ideal 0.05) — R larger than the real error (conservative)
- A': tuned constant: mean NIS 0.12 (all frames pooled; median of per-sortie means 0.11), fraction > χ²₂(0.95)=5.99: 0.002 (ideal 0.05) — R larger than the real error (conservative)
- B: VQA-EKF: mean NIS 0.14 (all frames pooled; median of per-sortie means 0.11), fraction > χ²₂(0.95)=5.99: 0.002 (ideal 0.05) — R larger than the real error (conservative)

## Secondary outcomes (exploratory, not adjusted for multiplicity)
- Touchdown dx (pad X), B vs A: median diff +0.00 (95% CI -3.00 to +4.00), Cliff's δ +0.11, Mann-Whitney p=0.631, n=15/15
- Touchdown dx (pad X), B vs A': median diff -1.00 (95% CI -1.50 to +3.00), Cliff's δ +0.06, Mann-Whitney p=0.803, n=15/15
- Touchdown dy (pad Y), B vs A: median diff -1.00 (95% CI -4.00 to +1.00), Cliff's δ -0.28, Mann-Whitney p=0.198, n=15/15
- Touchdown dy (pad Y), B vs A': median diff -1.50 (95% CI -3.50 to +0.00), Cliff's δ -0.43, Mann-Whitney p=0.044, n=15/15
- EKF-controlled point vs marker at the last camera fix, B vs A: median diff -0.95 (95% CI -2.60 to +0.96), Cliff's δ -0.25, Mann-Whitney p=0.245, n=15/15
- EKF-controlled point vs marker at the last camera fix, B vs A': median diff +0.59 (95% CI -1.57 to +1.81), Cliff's δ +0.05, Mann-Whitney p=0.836, n=15/15
- Blind-segment drift (last fix -> touchdown), B vs A: median diff -1.58 (95% CI -2.61 to +0.61), Cliff's δ -0.32, Mann-Whitney p=0.147, n=15/15
- Blind-segment drift (last fix -> touchdown), B vs A': median diff -0.88 (95% CI -2.45 to +1.50), Cliff's δ -0.13, Mann-Whitney p=0.561, n=15/15
- Overshoot after the last step, B vs A: median diff -1.84 (95% CI -4.89 to +0.40), Cliff's δ -0.37, Mann-Whitney p=0.089, n=15/15
- Overshoot after the last step, B vs A': median diff -0.02 (95% CI -0.78 to +1.44), Cliff's δ +0.08, Mann-Whitney p=0.740, n=15/15
- Settling time to 5 cm, B vs A: median diff -0.19 (95% CI -9.10 to +4.13), Cliff's δ -0.10, Mann-Whitney p=0.647, n=15/14
- Settling time to 5 cm, B vs A': median diff +0.89 (95% CI -0.77 to +4.95), Cliff's δ +0.12, Mann-Whitney p=0.590, n=15/15
- Hold RMS in CONVERGE, B vs A: median diff +0.15 (95% CI -1.45 to +0.98), Cliff's δ -0.03, Mann-Whitney p=0.901, n=15/15
- Hold RMS in CONVERGE, B vs A': median diff +0.15 (95% CI -0.36 to +0.90), Cliff's δ +0.10, Mann-Whitney p=0.648, n=15/15
- Mean position NIS (ideal 2), B vs A: median diff +0.11 (95% CI +0.10 to +0.16), Cliff's δ +1.00, Mann-Whitney p=0.000, n=15/15
- Mean position NIS (ideal 2), B vs A': median diff +0.00 (95% CI -0.02 to +0.04), Cliff's δ +0.17, Mann-Whitney p=0.431, n=15/15
- Position innovation RMS, B vs A: median diff -1.23 (95% CI -1.44 to -0.93), Cliff's δ -1.00, Mann-Whitney p=0.000, n=15/15
- Position innovation RMS, B vs A': median diff +0.03 (95% CI -0.15 to +0.31), Cliff's δ +0.11, Mann-Whitney p=0.619, n=15/15

## Validity
- Max maneuver tilt, median over sorties: 19.0 deg (range 18.3–19.8; target 15–22)
- Sorties without touchdown truth: 0
- Not completed: 0 (report them; do not silently drop)