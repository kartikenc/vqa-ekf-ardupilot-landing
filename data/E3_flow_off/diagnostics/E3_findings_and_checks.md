# E3 (optical flow OFF) — findings and checks, frozen 23 Nov 2026

Analyzer: analyze_vqa_ekf_final.py v1.1 (SHA-256 starts 92bdc6bb5a306b7d). Script: vqa_ekf_e3_V1.py = v7.3 (SHA-256 starts b2b918ce3308c2b0).
Workbook: VQA_EKF_E3_Campaign_final.xlsx (primary). Sensitivity copy: VQA_EKF_E3_Campaign_noglare_ANALYSIS_COPY.xlsx (S35-S41 "Excluded-glare").

## Primary (all 45, A 15 / A' 15 / B 15)
- H0 PASS all arms. B sigma rho(sigma, tilt) 0.65; p95/median x1.18; B median 1.14 cm = A' 1.14 cm.
- H1 B vs A: Holm p 0.130, Cliff's delta -0.33. H2 B vs A': p 0.044, Holm p 0.088, delta -0.44. Not supported.
- CEP50 camera point 13.5 / 13.5 / 11.8 cm; controlled point 3.0 / 3.4 / 2.5 cm; spread RMS 4.5 / 4.1 / 2.6 cm.
- Precision (distance to own centroid) B vs A p 0.034; B vs A' p 0.097. Hold RMS B vs A p 0.028.
- Blind drift 2.4 / 3.1 / 1.4 cm (B vs A' p 0.046).
- NIS mean 0.00 / 0.37 / 0.10 (ideal 2) -> R conservative in every arm.

## Sensitivity (glare sorties S35-S41 excluded; A 13 / A' 13 / B 12)
- H1 Holm 0.71, H2 Holm 0.71 (not supported). Precision B vs A p 0.004, delta -0.68, CI excludes 0. B vs A' p 0.068.

## Data-quality checks
- "EKF failsafe" flags on S23, S41, S44, S45 are FALSE POSITIVES: "EKF variance" (ERR subsys 16) fired 12-15 s AFTER disarm
  (S23 853.7->866.7 s; S41 410.0->425.0; S44 861.7->876.3; S45 1075.3->1090.1), GUIDED only in flight, no LAND takeover.
  The analyzer flag scans the whole log including post-disarm. results_E3_clean (which excluded them) is NOT cited.
- Glare: S35-S41 (16:37-17:23) detection 73-90 %, misses at ~8 deg tilt vs ~18 deg normally; brightness unchanged
  -> specular glare on semi-gloss marker at low sun. Balanced A 2 / A' 2 / B 3.
- S38 (A', glare) is the only sortie in E3 where EKF3 rejected vision (pos test ratio max 1.74, hold RMS 20.6 cm).
  B max test ratio 0.88; A max 0.04.
- 12 sorties started after the 15:30 protocol window (S05R1, S31-S41).
- S05 invalid (telemetry link fault, parameter read failure; FC->Pi worked, intermittent Pi->FC) -> re-flown as S05R1.
  Trial folders E3_S05R and E3_S50 moved to E3_excluded_sorties (they stole logs 148/187 in matching).
- Bin matching: 45/45 content-verified (0.000 cm over 100 % of VISP rows). Logs 142-188.
- Pad frame: auto R270, residual 1.17 cm vs next best 5.22 cm.
