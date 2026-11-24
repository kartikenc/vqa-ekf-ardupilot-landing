# VQA-EKF: quality-adaptive vision noise for marker-based landing in ArduPilot EKF3

Code, parameters and flight data for the article

> K. E. Cholachgudda, S. Patil, M. T. Megharaj, A. Kadam, K. M. Sudharshan, M. Lakshmipathaiah,
> *Quality-Adaptive Vision Noise for Marker-Based UAV Landing in a Production Autopilot: A Randomised
> 90-Sortie Flight Study With and Without Optical Flow* (submitted to *Autonomous Intelligent Systems*, 2026).

VQA-EKF gives every ArUco position fix its own measurement noise, computed from vehicle tilt θ and
image sharpness L (variance of the Laplacian):

```
σ²(θ, L) = σ_b² · (sec²θ)^p · (100 / L)^q        σ_b = 0.0226 m, p = 5.25, q = 0.69
```

The noise is sent in the covariance of the MAVLink `VISION_POSITION_ESTIMATE` message as
`c0 = c6 = c11 = sqrt(σ³/3)`, so that the unmodified ArduPilot EKF3 uses exactly σ
(down to its `VISO_POS_M_NSE` floor). Every analysis reads back the value EKF3 actually used
(`VISP.PErr` in the DataFlash log).

## Main results (median touchdown error, camera point)

| Campaign | A: constant 20 cm | A′: tuned 1.14 cm | B: VQA-EKF |
|---|---|---|---|
| E1, optical flow fused (45 sorties) | 13.7 cm | 13.6 cm | **12.5 cm** |
| E3, vision only (45 sorties) | 13.5 cm | 13.5 cm | **11.8 cm** |

In E3, B's touchdowns were 47 % less dispersed than A's (p = 0.034; p = 0.004 without the
glare-affected sorties) and B drifted least during the blind final descent. All numbers in the
article are produced by the scripts and frozen outputs in this repository.

## Platform

Holybro X500 V2 · Cube Orange+ with ArduCopter 4.6.3 (EKF3) · Raspberry Pi 5 · Intel RealSense D455
(RGB 640×480, 30 Hz, downward) · Holybro H-Flow (DroneCAN, ToF height + optical flow) ·
16 cm ArUco marker (DICT_4X4_1000, ID 0). GNSS and compass disabled.

## Repository layout

| Path | Content |
|---|---|
| `flight_code/vqa_ekf_ab_test_v7.2.py` | Companion-computer script flown in **E1** (optical flow on) |
| `flight_code/vqa_ekf_e3_V1.py` | Script flown in **E3** (`--no-flow`; v7.2 plus an EKF absolute-position take-off gate and in-flight guard) |
| `flight_code/G1_diff_v7.2_vs_e3V1.diff` | Line-by-line difference between the two scripts (gate G1) |
| `flight_code/tools/check_attitude_link.py` | MAVLink attitude-stream check used before each campaign |
| `model_fit/fit_vqa_model_handheld.py` | Truth-free hand-held fit of σ_b, p, q |
| `analysis/analyze_vqa_ekf_final.py` | Analyzer v1.1 (SHA-256 `92bdc6bb5a306b7d…`): log matching, all metrics, tests and figures |
| `analysis/check_maneuver_frames.py` | Manoeuvre-frame check |
| `analysis/paper/` | Scripts for the article's extra statistics (non-inferiority, power, E1 vs E3) and Figs. 2, 4, 6 |
| `params/` | Flight-controller parameters: E1 (first and last log of the campaign) and E3 |
| `data/E1_flow_on/`, `data/E3_flow_off/` | Campaign workbooks (randomised schedule, status, touchdown truth), companion-computer logs, frozen analyzer outputs, diagnostics |
| `data/dataflash_logs/` | Pointer to the ArduPilot DataFlash logs (.bin) archived in Zenodo |

Frozen outputs per campaign (`results_frozen*/`): `per_sortie.csv` (one row per sortie, 82 columns),
`summary_by_arm.csv`, `tests.csv`, `bin_match.csv`, `report.md`, LaTeX tables and all analyzer figures.
`results_frozen_sensitivity_noglare` is the sensitivity analysis without the seven
low-sun sorties of 22 November (E3_S35–S41).

## Reproducing the analysis

Python ≥ 3.10 with `numpy pandas scipy matplotlib openpyxl pymavlink`.

```bash
# 1. unpack the companion-computer logs and the DataFlash logs (from Zenodo)
tar xzf data/E3_flow_off/companion_logs/vqa_paper_E3_20261123.tar.gz
# 2. match each .bin to its sortie by content (vision positions logged on both sides)
python3 analysis/analyze_vqa_ekf_final.py match-bins --logs vqa_paper_E3 --experiment E3 --bins E3_bins --copy
# 3. full analysis
python3 analysis/analyze_vqa_ekf_final.py analyze --logs vqa_paper_E3 --experiment E3 \
        --bins E3_bins --workbook data/E3_flow_off/workbook/VQA_EKF_E3_Campaign_final.xlsx --out results_E3
# 4. article statistics and figures from the frozen outputs (no logs needed)
python3 analysis/paper/paper_stats.py   --e1 data/E1_flow_on/results_frozen --e3 data/E3_flow_off/results_frozen_primary_all45 --out analysis/paper/out
python3 analysis/paper/paper_figures.py --e1 data/E1_flow_on/results_frozen --e3 data/E3_flow_off/results_frozen_primary_all45 --out analysis/paper/out
```

E1 is analysed the same way with `--experiment E1` and the E1 workbook. Step 4 reproduces
`analysis/paper/out/paper_numbers.json` exactly (bootstrap seed 20261124).

## Running the flight script

Read the header of the script first: it lists every option, the pre-flight bench check
(`--bench`), the camera check (`--cam-check`) and the safety behaviour. Example (E3, arm B):

```bash
python3 flight_code/vqa_ekf_e3_V1.py --mode adaptive --experiment E3 --no-flow \
        --log-root ~/vqa_paper_E3 --sortie-id E3_S03
```

Fly only with a safety pilot holding a STABILIZE takeover switch.

## Citation and licences

Please cite the article and the archived release (see `CITATION.cff`; DOI on the Zenodo badge).
Code: MIT licence (`LICENSE`). Data, workbooks and figures: CC BY 4.0 (`DATA_LICENSE.md`).

Funding: REVA University seed funding, Ref. REVA/IRoC-U/2026/DAC-001.
Contact: Kartik E. Cholachgudda, REVA University, Bengaluru — kartik.c@reva.edu.in
