#!/usr/bin/env python3
"""
analyze_vqa_ekf_final.py — final analysis of the VQA-EKF flight campaign (Paper 1, IEEE RA-L)
=============================================================================================
Pairs with vqa_ekf_ab_test_v7.x.py. Runs on the LAPTOP after the campaign (not on the Pi).

Sub-commands
------------
  analyze     all sorties of one experiment -> tables, figures, hypothesis report
                python analyze_vqa_ekf_final.py analyze --logs vqa_paper_E1 --experiment E1 \
                       --bins MS_bin --workbook VQA_EKF_E1_Campaign_final.xlsx --out results_E1
  match-bins  show which DataFlash .bin belongs to which sortie (and optionally copy it in)
                python analyze_vqa_ekf_final.py match-bins --logs vqa_paper_E1 --experiment E1 \
                       --bins MS_bin [--copy]
  tune        Arm A' sigma from pilot sorties (median model sigma in HOVER_BASE)
  schedule    randomised, blocked flight order (same seed/algorithm as the campaign workbook)

Inputs
------
  --logs      the paper log root copied from the Pi (one folder per sortie with
              sortie_summary.json + vision_frames.csv; a .bin inside the folder is used if present)
  --bins      folder with the DataFlash logs downloaded with Mission Planner. They are matched to
              the sorties in flight order and every match is VERIFIED from the log itself (Arm A:
              EKF used 20 cm, Arm A': the tuned value, Arm B: a varying sigma). Any mismatch stops.
  --workbook  the campaign workbook: Status (only "Valid" sorties enter the tests) and the pad-grid
              touchdown dx/dy (sheets "E1 Schedule" and "Re-fly Log"). Overrides the values typed at
              the prompt. Without it, the truth stored in sortie_summary.json is used.
  --pad-map   how the pad grid axes relate to the EKF/marker frame. "auto" (default) picks the
              mapping that best explains the touchdowns from the EKF positions and prints it.
              Explicit: R0 R90 R180 R270 (rotation) or F0 F90 F180 F270 (mirror, then rotation).
  --ref-offset-cm DX DY
              physically measured offset (pad frame, cm) between the point measured on the grid
              (landing-gear centroid) and the point the EKF steers (the VISO_POS reference).
              Used for the SECONDARY "corrected" touchdown analysis. Without it the common offset
              is estimated from the data and reported as descriptive only.

Outputs (in --out)
------------------
  per_sortie.csv, summary_by_arm.csv, tests.csv, data_quality.csv, bin_match.csv,
  results_table.tex, results_tests.tex, report.md, fig1, fig3-fig7, figS1 (.png + .pdf)

Needs: numpy pandas scipy matplotlib pymavlink openpyxl
    pip install numpy pandas scipy matplotlib pymavlink openpyxl
"""

import argparse
import glob
import hashlib
import json
import math
import os
import random
import re
import shutil
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np
import pandas as pd
from scipy import stats

ARMS = ["constant", "tuned", "adaptive"]
ARM_NAME = {"constant": "A: constant 20 cm", "tuned": "A': tuned constant", "adaptive": "B: VQA-EKF"}
ARM_SHORT = {"constant": "A", "tuned": "A'", "adaptive": "B"}
COLOR = {"constant": "#2a78d6", "tuned": "#eb6834", "adaptive": "#1baf7a"}
MARK = {"constant": "o", "tuned": "s", "adaptive": "^"}
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"
SETTLE_RADIUS_M = 0.05
SIGMA_FLOOR_M, SIGMA_CEIL_M, SIGMA_DEFAULT_M = 0.01, 0.50, 0.20
SMOOTH_FRAMES = 15
ALPHA = 0.05
ANALYZER_VERSION = "1.1"      # 1.1: figures/tables for the paper; frame-level data follow --exclude-flagged
N_BOOT = 10000


# ═════════════════════════════════════════════════════════════════════════
# small helpers
# ═════════════════════════════════════════════════════════════════════════
def rolling_median(a, w=SMOOTH_FRAMES):
    a = np.asarray(a, float)
    h, n = w // 2, len(a)
    return np.array([np.nanmedian(a[max(0, i - h):min(n, i + h + 1)]) for i in range(n)])


def unit(vx, vy):
    n = math.hypot(vx, vy)
    return (vx / n, vy / n) if n > 1e-9 else (0.0, 0.0)


def recovery_metrics(t, x, y, prev_target, t_step):
    """Overshoot (cm) past the origin and settling time (s) from 0.5 s-smoothed positions."""
    t, x, y = (np.asarray(v, float) for v in (t, x, y))
    ok = np.isfinite(t) & np.isfinite(x) & np.isfinite(y)
    t, x, y = t[ok], x[ok], y[ok]
    if len(t) < SMOOTH_FRAMES:
        return np.nan, np.nan
    o = np.argsort(t)
    t, xs, ys = t[o], rolling_median(x[o]), rolling_median(y[o])
    u = unit(*prev_target)
    overshoot = max(0.0, float(np.max(-(xs * u[0] + ys * u[1])))) * 100
    r = np.hypot(xs, ys)
    out = np.where(r > SETTLE_RADIUS_M)[0]
    if len(out) == 0:
        settle = 0.0
    elif out[-1] < len(r) - 1:
        settle = float(t[out[-1] + 1] - t_step)
    else:
        settle = np.nan
    return overshoot, settle


def event_t(summary, name, last=False):
    ts = [e["t"] for e in summary.get("events", []) if e.get("event") == name]
    if not ts:
        return None
    return ts[-1] if last else ts[0]


def fnum(v):
    try:
        f = float(v)
        return f if math.isfinite(f) else np.nan
    except (TypeError, ValueError):
        return np.nan


# ═════════════════════════════════════════════════════════════════════════
# DataFlash (.bin) parsing — one worker per file, results cached
# ═════════════════════════════════════════════════════════════════════════
BIN_TYPES = ["PARM", "MSG", "VISP", "ATT", "XKF4", "XKF3", "XKF1", "OF", "RFND", "GUIP", "EV", "ERR"]


def _parse_bin_worker(path):
    """Read one .bin into compact arrays. Runs in a separate process."""
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(path)
    P, out = {}, {k: [] for k in BIN_TYPES if k != "PARM"}
    has_pi = None
    while True:
        try:
            x = m.recv_match(type=BIN_TYPES, blocking=False)
        except Exception:                                   # noqa: BLE001  corrupt tail
            break
        if x is None:
            break
        t = x.get_type()
        T = getattr(x, "TimeUS", 0) / 1e6
        if t == "PARM":
            P[x.Name] = float(x.Value)
        elif t == "MSG":
            out["MSG"].append((T, str(x.Message)))
        elif t == "VISP":
            out["VISP"].append((T, x.PX, x.PY, x.PErr, getattr(x, "Ign", 0), x.Y, getattr(x, "RTimeUS", 0) / 1e6))
        elif t == "ATT":
            out["ATT"].append((T, x.Roll, x.Pitch))
        elif t == "XKF4":
            if has_pi is None:
                has_pi = hasattr(x, "PI")
            out["XKF4"].append((T, x.C, x.SP, x.SV, x.SH, x.SM, x.SS, x.TS, getattr(x, "PI", 0)))
        elif t == "XKF3":
            out["XKF3"].append((T, x.C, x.IPN, x.IPE))
        elif t == "XKF1":
            out["XKF1"].append((T, x.C, x.PN, x.PE, x.PD))
        elif t == "OF":
            out["OF"].append((T, x.Qual))
        elif t == "RFND":
            out["RFND"].append((T, x.Dist))
        elif t == "GUIP":
            out["GUIP"].append((T, x.pX, x.pY))
        elif t == "EV":
            out["EV"].append((T, x.Id))
        elif t == "ERR":
            out["ERR"].append((T, x.Subsys, x.ECode))
    out["PARM"] = P
    out["_file"] = os.path.basename(path)
    return out


CACHE_VERSION = 2


def _cache_key(path):
    st = os.stat(path)
    key = f"v{CACHE_VERSION}|{os.path.abspath(path)}|{st.st_size}|{int(st.st_mtime)}"
    return hashlib.sha1(key.encode()).hexdigest()[:16]


def parse_bins(paths, cache_dir, jobs):
    os.makedirs(cache_dir, exist_ok=True)
    res, todo = {}, []
    for p in paths:
        c = os.path.join(cache_dir, _cache_key(p) + ".json")
        if os.path.exists(c):
            with open(c, encoding="utf-8") as f:
                res[p] = json.load(f)
        else:
            todo.append(p)
    if todo:
        print(f"  parsing {len(todo)} .bin file(s) with {jobs} worker(s) (cached for next time)...")
        if jobs > 1:
            with ProcessPoolExecutor(max_workers=jobs) as ex:
                parsed = list(ex.map(_parse_bin_worker, todo))
        else:
            parsed = [_parse_bin_worker(p) for p in todo]
        for p, d in zip(todo, parsed):
            res[p] = d
            with open(os.path.join(cache_dir, _cache_key(p) + ".json"), "w", encoding="utf-8") as f:
                json.dump(d, f)
    return res


def arr(d, key, ncol):
    a = np.array(d.get(key) or [], dtype=float)
    return a.reshape(-1, ncol) if a.size else np.zeros((0, ncol))


def primary_rows(a, c_col=1, pi_col=None):
    """Rows of the primary EKF lane (XKF4.PI if logged, else lane 0)."""
    if not len(a):
        return a
    if pi_col is not None:
        return a[a[:, c_col] == a[:, pi_col]]
    return a[a[:, c_col] == 0]


def bin_metrics(d):
    """Everything the paper needs from one DataFlash log. Windows come from the log itself
    (guided step targets, disarm event), so no clock alignment with the Pi is needed."""
    P = d.get("PARM", {})
    visp = arr(d, "VISP", 7)
    att = arr(d, "ATT", 3)
    k4 = arr(d, "XKF4", 9)
    k3 = arr(d, "XKF3", 4)
    k1 = arr(d, "XKF1", 5)
    of = arr(d, "OF", 2)
    rf = arr(d, "RFND", 2)
    gu = arr(d, "GUIP", 3)
    ev = arr(d, "EV", 2)
    err = arr(d, "ERR", 3)
    msgs = [m for _, m in (d.get("MSG") or [])]
    m = {"bin_file": d.get("_file")}
    if not len(visp) or not len(att):
        m["bin_ok"] = False
        return m
    m["bin_ok"] = True
    # primary lane
    k4p = k4[k4[:, 1] == k4[:, 8]] if len(k4) else k4
    if not len(k4p):
        k4p = k4[k4[:, 1] == 0]
    lane0 = int(np.round(np.median(k4p[:, 8]))) if len(k4p) else 0
    k3p = k3[k3[:, 1] == lane0] if len(k3) else k3
    k1p = k1[k1[:, 1] == lane0] if len(k1) else k1
    tilt = np.degrees(np.arccos(np.clip(np.cos(np.radians(att[:, 1])) * np.cos(np.radians(att[:, 2])), -1, 1)))
    # maneuver window = first .. last guided step (+ one step hold)
    steps = [gu[i, 0] for i in range(1, len(gu)) if math.hypot(gu[i, 1] - gu[i - 1, 1], gu[i, 2] - gu[i - 1, 2]) > 20]
    if steps:
        t_m0, t_m1 = steps[0], steps[-1] + 4.0
    else:
        t_m0 = t_m1 = np.nan
    dis = [t for t, i in ev if int(i) == 11]                      # EV 11 = DISARMED
    t_dis = dis[-1] if dis else (k1p[-1, 0] if len(k1p) else visp[-1, 0])
    fl = rf[:, 1] > 0.6 if len(rf) else np.array([], bool)
    t_f0, t_f1 = (rf[fl, 0].min() + 3, rf[fl, 0].max() - 3) if fl.any() else (visp[0, 0], visp[-1, 0])
    m["n_steps"] = len(steps)
    # sigma the EKF actually used
    vf = visp[(visp[:, 0] > t_f0) & (visp[:, 0] < t_f1)]
    vm = visp[(visp[:, 0] > t_m0) & (visp[:, 0] < t_m1)] if steps else vf
    m["visp_perr_med_cm"] = float(np.median(vf[:, 3])) * 100
    m["visp_perr_p05_cm"] = float(np.percentile(vf[:, 3], 5)) * 100
    m["visp_perr_p95_cm"] = float(np.percentile(vf[:, 3], 95)) * 100
    m["visp_man_med_cm"] = float(np.median(vm[:, 3])) * 100 if len(vm) else np.nan
    m["visp_man_p95_cm"] = float(np.percentile(vm[:, 3], 95)) * 100 if len(vm) else np.nan
    m["visp_ignored_frac"] = float(np.mean(vf[:, 4])) if len(vf) else np.nan
    m["visp_pinned_frac"] = float(np.mean(np.abs(vf[:, 3] - SIGMA_FLOOR_M) < 2e-5)) if len(vf) else np.nan
    if len(vm) > 20 and np.ptp(vm[:, 3]) > 1e-6:
        m["visp_rho_sigma_tilt"] = float(stats.spearmanr(vm[:, 3], np.interp(vm[:, 0], att[:, 0], tilt)).correlation)
    else:
        m["visp_rho_sigma_tilt"] = np.nan
    m["_visp_man"] = np.c_[np.interp(vm[:, 0], att[:, 0], tilt), vm[:, 3]].tolist() if len(vm) else []
    # arm signature from the log alone
    flat = np.ptp(np.round(vf[:, 3], 5)) < 1e-4
    if flat and abs(np.median(vf[:, 3]) - SIGMA_DEFAULT_M) < 1e-3:
        m["bin_arm"] = "constant"
    elif flat:
        m["bin_arm"] = "tuned"
    else:
        m["bin_arm"] = "adaptive"
    m["bin_flat_sigma_m"] = float(np.median(vf[:, 3])) if flat else np.nan
    # tilt in the maneuver (cross-check of the script's own value)
    w = (att[:, 0] > t_m0) & (att[:, 0] < t_m1) if steps else np.zeros(len(att), bool)
    m["bin_tilt_max_deg"] = float(tilt[w].max()) if w.any() else np.nan
    # filter consistency (primary lane)
    gate = max(P.get("EK3_POS_I_GATE", 500.0), 100.0) / 100.0
    m["ek3_pos_i_gate"] = gate
    kw = k4p[(k4p[:, 0] > t_f0) & (k4p[:, 0] < t_f1)] if len(k4p) else k4p
    if len(kw):
        nis = 2.0 * gate ** 2 * kw[:, 2] ** 2                     # 2-D NIS from the logged test ratio
        m["nis_mean"] = float(nis.mean())
        m["nis_frac_gt_5p99"] = float(np.mean(nis > 5.991))
        m["pos_test_ratio_max"] = float(kw[:, 2].max())
        m["_nis"] = nis.tolist()
        m["relpos_lost_samples"] = int(np.sum([not (int(s) >> 3) & 1 for s in kw[:, 6]]))
        m["pos_timeout_latched"] = int(int(k4p[0, 7]) & 1)
    k3w = k3p[(k3p[:, 0] > t_f0) & (k3p[:, 0] < t_f1)] if len(k3p) else k3p
    m["pos_innov_rms_cm"] = float(np.sqrt(np.mean(k3w[:, 2] ** 2 + k3w[:, 3] ** 2))) * 100 if len(k3w) else np.nan
    # EKF position where the drone landed (world/marker frame)
    if len(k1p):
        j = max(np.searchsorted(k1p[:, 0], t_dis) - 3, 0)
        m["ekf_td_x_cm"], m["ekf_td_y_cm"] = float(k1p[j, 2]) * 100, float(k1p[j, 3]) * 100
    # sensor health
    ow = of[(of[:, 0] > t_f0) & (of[:, 0] < t_f1)] if len(of) else of
    gaps = np.diff(ow[:, 0]) if len(ow) > 1 else np.array([0.0])
    m["flow_gap_max_s"] = float(gaps.max())
    m["flow_gaps_gt_0p3s"] = int((gaps > 0.3).sum())
    m["flow_quality_med"] = float(np.median(ow[:, 1])) if len(ow) else np.nan
    rw = rf[(rf[:, 0] > t_f0) & (rf[:, 0] < t_f1)] if len(rf) else rf
    m["rng_zero_readings"] = int((rw[:, 1] < 0.1).sum()) if len(rw) else 0
    m["ekf_failsafe"] = bool(any(("EKF Failsafe" in s) or ("EKF variance" in s) for s in msgs)
                             or (len(err) and np.any((err[:, 1] == 17) & (err[:, 2] == 1))))
    m["lane_switches"] = int(sum("lane switch" in s for s in msgs))
    m["ek3_imu_mask"] = P.get("EK3_IMU_MASK")
    m["_params"] = {k: P[k] for k in sorted(P) if k.startswith(("EK3_", "VISO_", "FLOW_", "RNGFND1_", "WPNAV_",
                                                               "PSC_", "GUID_", "ANGLE_MAX"))}
    return m


def bin_number(path):
    mt = re.match(r"\s*0*(\d+)", os.path.basename(path))
    return int(mt.group(1)) if mt else int(os.path.getmtime(path))


# ═════════════════════════════════════════════════════════════════════════
# sorties, truth, frame mapping
# ═════════════════════════════════════════════════════════════════════════
def find_sorties(root, experiment=None, include_bench=False):
    out = []
    for js in glob.glob(os.path.join(os.path.expanduser(root), "**", "sortie_summary.json"), recursive=True):
        with open(js, encoding="utf-8") as f:
            s = json.load(f)
        if s.get("bench") and not include_bench:
            continue
        exp = (s.get("args") or {}).get("experiment") or "E1"
        if experiment and exp != experiment:
            continue
        s["_dir"] = os.path.dirname(js)
        s["_t0"] = s["events"][0]["t"] if s.get("events") else os.path.getmtime(js)
        out.append(s)
    out.sort(key=lambda s: s["_t0"])
    return out


def read_workbook(path):
    """sortie_id -> dict(status, dx_cm, dy_cm, mode) from the campaign workbook."""
    from openpyxl import load_workbook
    wb = load_workbook(path, data_only=True)
    res = {}

    def col_index(ws, row, names):
        hdr = {str(ws.cell(row=row, column=c).value or "").strip().lower(): c for c in range(1, 40)}
        return {k: next((c for h, c in hdr.items() if h.startswith(n)), None) for k, n in names.items()}
    if "E1 Schedule" in wb.sheetnames or any(n.endswith("Schedule") for n in wb.sheetnames):
        ws = wb[next(n for n in wb.sheetnames if n.endswith("Schedule"))]
        ci = col_index(ws, 4, dict(sid="sortie id", mode="mode", status="status",
                                   dx="touchdown dx", dy="touchdown dy"))
        for r in range(5, ws.max_row + 1):
            sid = ws.cell(row=r, column=ci["sid"]).value
            if sid:
                res[str(sid)] = dict(mode=ws.cell(row=r, column=ci["mode"]).value,
                                     status=str(ws.cell(row=r, column=ci["status"]).value or ""),
                                     dx_cm=fnum(ws.cell(row=r, column=ci["dx"]).value),
                                     dy_cm=fnum(ws.cell(row=r, column=ci["dy"]).value), source="workbook")
    if "Re-fly Log" in wb.sheetnames:
        ws = wb["Re-fly Log"]
        ci = col_index(ws, 4, dict(sid="re-fly id", mode="mode", status="status",
                                   dx="touchdown dx", dy="touchdown dy"))
        for r in range(5, ws.max_row + 1):
            sid = ws.cell(row=r, column=ci["sid"]).value
            if sid:
                res[str(sid)] = dict(mode=ws.cell(row=r, column=ci["mode"]).value,
                                     status=str(ws.cell(row=r, column=ci["status"]).value or ""),
                                     dx_cm=fnum(ws.cell(row=r, column=ci["dx"]).value),
                                     dy_cm=fnum(ws.cell(row=r, column=ci["dy"]).value), source="workbook re-fly")
    return res


MAPS = {f"{'F' if flip < 0 else 'R'}{rot}": (rot, flip) for flip in (1, -1) for rot in (0, 90, 180, 270)}


def map_matrix(name):
    rot, flip = MAPS[name]
    th = math.radians(rot)
    return np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]]) @ np.diag([1.0, flip])


def describe_map(name):
    M = map_matrix(name)
    lab = ["world X(N)", "world Y(E)"]
    parts = []
    for i, axis in enumerate(("pad X", "pad Y")):
        j = int(np.argmax(np.abs(M[i])))
        parts.append(f"{axis} = {'+' if M[i, j] > 0 else '-'}{lab[j]}")
    return ", ".join(parts)


def auto_map(E_world, T_pad):
    """Pick the axis mapping that best explains truth = M @ EKF + constant offset."""
    ok = np.isfinite(E_world).all(1) & np.isfinite(T_pad).all(1)
    E, T = E_world[ok], T_pad[ok]
    if len(E) < 6:
        return "R0", []
    scores = []
    for name in MAPS:
        P = E @ map_matrix(name).T
        res = T - P - (T - P).mean(0)
        scores.append((float(np.sqrt((res ** 2).sum(1).mean())), name, (T - P).mean(0)))
    scores.sort()
    return scores[0][1], scores


# ═════════════════════════════════════════════════════════════════════════
# per-sortie metrics from the Pi files
# ═════════════════════════════════════════════════════════════════════════
def camera_offset_m(s):
    """Camera position relative to the EKF reference point (body frame, m): the VISO_POS_X/Y the
    script read from the flight controller (args.cam_offset_xy), else fc_params, else 0.11/0."""
    a = s.get("args") or {}
    off = a.get("cam_offset_xy")
    if off and len(off) == 2 and all(math.isfinite(fnum(v)) for v in off):
        return float(off[0]), float(off[1])
    fp = s.get("fc_params") or {}
    if math.isfinite(fnum(fp.get("VISO_POS_X"))) and math.isfinite(fnum(fp.get("VISO_POS_Y"))):
        return float(fp["VISO_POS_X"]), float(fp["VISO_POS_Y"])
    return 0.11, 0.0


def add_cg_columns(s, df):
    """x_raw / y_raw are the CAMERA position. The EKF flies its reference point (the VISO_POS
    origin), so recovery metrics must use cg = camera - R(yaw) * offset."""
    ox, oy = camera_offset_m(s)
    yaw = pd.to_numeric(df.get("yaw_marker"), errors="coerce").to_numpy(float) if "yaw_marker" in df else np.zeros(len(df))
    if np.nanmax(np.abs(yaw)) > 2 * math.pi + 0.1:                 # degrees by mistake -> radians
        yaw = np.radians(yaw)
    c, sn = np.cos(yaw), np.sin(yaw)
    df["cg_x"] = df.x_raw - (c * ox - sn * oy)
    df["cg_y"] = df.y_raw - (sn * ox + c * oy)
    return df


def csv_metrics(s, df):
    m = {}
    if df is None or not len(df):
        return m
    df = add_cg_columns(s, df)
    acc = df[df.accepted == 1]
    for ph in ("HOVER_BASE", "MANEUVER", "CONVERGE"):
        v = acc.loc[acc.phase == ph, "sigma_sent"].dropna()
        m[f"sigma_{ph.lower()}_med_cm"] = v.median() * 100 if len(v) else np.nan
    mv = df[df.phase == "MANEUVER"]
    mva = mv[mv.accepted == 1]
    m["sigma_maneuver_p05_cm"] = mva.sigma_sent.quantile(.05) * 100 if len(mva) else np.nan
    m["sigma_maneuver_p95_cm"] = mva.sigma_sent.quantile(.95) * 100 if len(mva) else np.nan
    m["rho_sigma_tilt"] = (stats.spearmanr(mva.sigma_sent, mva.tilt_deg).correlation
                           if len(mva) > 10 and mva.sigma_sent.nunique() > 1 else np.nan)
    a_ = acc[["sigma_model", "sigma_sent"]].dropna()
    if len(a_):
        expect = a_.sigma_model.clip(SIGMA_FLOOR_M, SIGMA_CEIL_M)
        m["sigma_model_match"] = float((np.abs(a_.sigma_sent - expect) < 2e-5).mean())
        m["sigma_pinned_frac"] = float(((np.abs(a_.sigma_sent - SIGMA_FLOOR_M) < 2e-5) |
                                        (np.abs(a_.sigma_sent - SIGMA_CEIL_M) < 2e-5)).mean())
    m["sigma_all_med_cm"] = acc.sigma_sent.median() * 100 if len(acc) else np.nan
    m["tilt_max_deg"] = mv.tilt_deg.max() if len(mv) else np.nan
    m["tilt_p95_deg"] = mv.tilt_deg.quantile(.95) if len(mv) else np.nan
    m["maneuver_accept_frac"] = mv.accepted.mean() if len(mv) else np.nan
    m["maneuver_tilt_gated"] = int(mv.tilt_gated.sum()) if len(mv) else 0
    m["latency_med_ms"] = acc.latency_ms.median() if len(acc) else np.nan
    m["blur_hover_med"] = df.loc[df.phase == "HOVER_BASE", "blur_full"].median()
    steps = [e for e in s["events"] if e["event"] == "STEP"]
    t_conv_end = event_t(s, "DESCENT")
    if len(steps) >= 2 and t_conv_end:
        last, prev = steps[-1], steps[-2]
        prev_tgt = (prev.get("target_x", 0.0), prev.get("target_y", 0.0))
        w = df[(df.t >= last["t"]) & (df.t <= t_conv_end)]
        for src, cx, cy in (("vis", "cg_x", "cg_y"), ("ekf", "ekf_x", "ekf_y")):
            ww = w[w.detected == 1] if src == "vis" else w
            if len(ww) > SMOOTH_FRAMES:
                m[f"overshoot_{src}_cm"], m[f"settle_{src}_s"] = recovery_metrics(ww.t, ww[cx], ww[cy],
                                                                                  prev_tgt, last["t"])
        conv = df[(df.phase == "CONVERGE") & (df.detected == 1)]
        if len(conv) > SMOOTH_FRAMES:
            cx_, cy_ = rolling_median(conv.cg_x), rolling_median(conv.cg_y)
            m["hold_rms_vis_cm"] = float(np.sqrt(np.nanmean(cx_ ** 2 + cy_ ** 2))) * 100
            m["ekf_vs_vis_rms_cm"] = float(np.sqrt(np.mean((conv.ekf_x - conv.cg_x) ** 2 +
                                                           (conv.ekf_y - conv.cg_y) ** 2))) * 100
    return m


def load_frames(d):
    p = os.path.join(d, "vision_frames.csv")
    if not os.path.exists(p):
        return None
    df = pd.read_csv(p)
    for c in ("t_capture", "accepted", "detected", "sigma_sent", "sigma_model", "tilt_deg", "x_raw",
              "y_raw", "ekf_x", "ekf_y", "latency_ms", "tilt_gated", "blur_full"):
        if c in df:
            df[c] = pd.to_numeric(df[c], errors="coerce")
    df["t"] = df["t_capture"]
    return df


# ═════════════════════════════════════════════════════════════════════════
# statistics
# ═════════════════════════════════════════════════════════════════════════
def cliffs_delta(x, y):
    x, y = np.asarray(x, float), np.asarray(y, float)
    return float(((x[:, None] > y[None, :]).sum() - (x[:, None] < y[None, :]).sum()) / (len(x) * len(y)))


def boot_ci(a, fn, n=N_BOOT, seed=0):
    a = np.asarray(a, float)
    a = a[np.isfinite(a)]
    if len(a) < 2:
        return (np.nan, np.nan)
    rng = np.random.default_rng(seed)
    return tuple(np.percentile([fn(rng.choice(a, len(a))) for _ in range(n)], [2.5, 97.5]))


def compare(x, y, seed=0):
    """x = Arm B, y = comparator. Negative differences favour B (lower is better)."""
    x = np.asarray(x, float); x = x[np.isfinite(x)]
    y = np.asarray(y, float); y = y[np.isfinite(y)]
    if len(x) < 3 or len(y) < 3:
        return dict(n_b=len(x), n_ref=len(y), p=np.nan, delta=np.nan, med_diff=np.nan, ci_lo=np.nan, ci_hi=np.nan)
    p = stats.mannwhitneyu(x, y, alternative="two-sided").pvalue
    rng = np.random.default_rng(seed)
    diffs = [np.median(rng.choice(x, len(x))) - np.median(rng.choice(y, len(y))) for _ in range(N_BOOT)]
    lo, hi = np.percentile(diffs, [2.5, 97.5])
    return dict(n_b=len(x), n_ref=len(y), p=float(p), delta=cliffs_delta(x, y),
                med_diff=float(np.median(x) - np.median(y)), ci_lo=float(lo), ci_hi=float(hi))


def centroid_test(X, Y, n_perm=N_BOOT, seed=0):
    """Permutation test: distance between the 2-D touchdown centroids of two arms (offset-free)."""
    X, Y = np.asarray(X, float), np.asarray(Y, float)
    X, Y = X[np.isfinite(X).all(1)], Y[np.isfinite(Y).all(1)]
    if len(X) < 3 or len(Y) < 3:
        return np.nan, np.nan
    d0 = float(np.linalg.norm(X.mean(0) - Y.mean(0)))
    Z = np.r_[X, Y]
    rng = np.random.default_rng(seed)
    cnt = 0
    for _ in range(n_perm):
        idx = rng.permutation(len(Z))
        cnt += np.linalg.norm(Z[idx[:len(X)]].mean(0) - Z[idx[len(X):]].mean(0)) >= d0
    return d0, (cnt + 1) / (n_perm + 1)


def holm(pvals):
    pvals = np.asarray(pvals, float)
    idx = np.argsort(pvals)
    adj = np.empty(len(pvals))
    running = 0.0
    for rank, i in enumerate(idx):
        running = max(running, min(1.0, (len(pvals) - rank) * pvals[i]))
        adj[i] = running
    return adj


# ═════════════════════════════════════════════════════════════════════════
# figures
# ═════════════════════════════════════════════════════════════════════════
def setup_mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "font.family": "serif", "font.size": 8, "axes.titlesize": 8, "axes.labelsize": 8,
        "legend.fontsize": 7, "xtick.labelsize": 7, "ytick.labelsize": 7,
        "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2, "ytick.color": INK2,
        "text.color": INK, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.5,
        "axes.spines.top": False, "axes.spines.right": False, "lines.linewidth": 1.2,
        "savefig.dpi": 300, "savefig.bbox": "tight", "savefig.pad_inches": 0.02,
        "figure.facecolor": "white", "axes.facecolor": "white", "pdf.fonttype": 42, "ps.fonttype": 42})
    return plt


# IEEE column widths (inches)
W1, W2 = 3.5, 7.16


def save(fig, out, name):
    for ext in ("png", "pdf"):
        fig.savefig(os.path.join(out, f"{name}.{ext}"))


def panel_label(ax, k):
    ax.text(-0.02, 1.02, f"({'abcdefgh'[k]})", transform=ax.transAxes, ha="right", va="bottom",
            fontsize=8, fontweight="bold")


def sigma_axis(ax, lo=0.8, hi=30):
    from matplotlib.ticker import FixedLocator, FixedFormatter, NullLocator
    ax.set_yscale("log")
    ticks = [t for t in (1, 1.5, 2, 3, 5, 10, 20, 50) if lo <= t <= hi]
    ax.yaxis.set_major_locator(FixedLocator(ticks))
    ax.yaxis.set_major_formatter(FixedFormatter([f"{t:g}" for t in ticks]))
    ax.yaxis.set_minor_locator(NullLocator())
    ax.set_ylim(lo, hi)


def fig_manipulation(plt, pts, out):
    """Per-frame sigma that EKF3 used (VISP.PErr) vs tilt, maneuver phase, valid sorties."""
    from matplotlib.lines import Line2D
    fig, ax = plt.subplots(figsize=(W1, 2.3))
    handles = []
    for arm in ARMS:
        p = np.asarray(pts.get(arm) or [], float).reshape(-1, 2)
        if not len(p):
            continue
        if len(p) > 6000:
            p = p[np.random.default_rng(0).choice(len(p), 6000, replace=False)]
        ax.scatter(p[:, 0], p[:, 1] * 100, s=4, marker=MARK[arm], color=COLOR[arm], alpha=0.35,
                   linewidths=0, rasterized=True, zorder=2)
        handles.append(Line2D([], [], ls="none", marker=MARK[arm], ms=4, color=COLOR[arm], label=ARM_NAME[arm]))
    ax.axhline(SIGMA_FLOOR_M * 100, color=INK2, lw=0.8, ls=":", zorder=1)
    ax.text(0.99, SIGMA_FLOOR_M * 100 * 0.93, "EKF3 floor (1 cm)", transform=ax.get_yaxis_transform(),
            ha="right", va="top", fontsize=6.5, color=INK2)
    sigma_axis(ax)
    ax.set_xlabel("Vehicle tilt (deg)")
    ax.set_ylabel(r"$\sigma$ used by EKF3 (cm)")
    ax.legend(handles=handles, frameon=False, loc="center right")
    save(fig, out, "fig1_manipulation")
    plt.close(fig)


def _touchdown_panel(plt, ax, dx, dy, arm, lim):
    """Scatter with tied grid readings drawn larger (area ~ count)."""
    pts = pd.DataFrame(dict(x=np.round(dx, 2), y=np.round(dy, 2))).groupby(["x", "y"]).size().reset_index(name="n")
    ax.scatter(pts.x, pts.y, s=20 * pts.n, marker=MARK[arm], color=COLOR[arm], edgecolors="white",
               linewidths=0.6, zorder=3, alpha=0.9)
    c = np.array([np.mean(dx), np.mean(dy)])
    disp = np.median(np.hypot(dx - c[0], dy - c[1]))
    ax.add_patch(plt.Circle(c, disp, fill=False, color=COLOR[arm], lw=1.0, ls="--", zorder=2))
    ax.plot(*c, "x", color=COLOR[arm], ms=7, mew=1.5, zorder=4)
    ax.plot(0, 0, "+", color=INK, ms=10, mew=1.2, zorder=4)
    from matplotlib.ticker import MaxNLocator
    ax.set_aspect("equal")
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim)
    ax.xaxis.set_major_locator(MaxNLocator(5, integer=True)); ax.yaxis.set_major_locator(MaxNLocator(5, integer=True))
    return disp


def _touchdown_legend(fig, plt, extra=None):
    from matplotlib.lines import Line2D
    h = [Line2D([], [], ls="none", marker="+", ms=8, mew=1.2, color=INK, label="marker centre"),
         Line2D([], [], ls="none", marker="x", ms=6, mew=1.5, color=INK2, label="arm centroid"),
         Line2D([], [], ls="--", color=INK2, lw=1.0, label="spread (median distance to centroid)"),
         Line2D([], [], ls="none", marker="o", ms=4, color=INK2, label="sortie (larger = repeated reading)")]
    if extra:
        h.append(extra)
    fig.legend(handles=h, loc="lower center", ncol=len(h), frameon=False, bbox_to_anchor=(0.5, -0.06),
               fontsize=6.5, handletextpad=0.4, columnspacing=1.2)


def fig_touchdown(plt, per, out, offset):
    from matplotlib.lines import Line2D
    v = per[per.valid & per.td_dx_cm.notna()]
    if not len(v):
        return
    # (main) controlled point = camera-point truth minus the reference offset; origin = marker centre
    X, Y = v.td_corr_dx_cm.to_numpy(float), v.td_corr_dy_cm.to_numpy(float)
    lim = np.ceil(max(np.nanmax(np.abs(X)), np.nanmax(np.abs(Y))) + 1.5)
    fig, axes = plt.subplots(1, 3, figsize=(W2, 2.55), sharex=True, sharey=True)
    for k, (ax, arm) in enumerate(zip(axes, ARMS)):
        p = v[v["mode"] == arm]
        if not len(p):
            continue
        disp = _touchdown_panel(plt, ax, p.td_corr_dx_cm.to_numpy(float), p.td_corr_dy_cm.to_numpy(float), arm, lim)
        ax.set_title(f"({'abc'[k]}) {ARM_NAME[arm]} (n={len(p)})\nmedian {p.td_radial_corr_cm.median():.1f} cm, "
                     f"spread {disp:.1f} cm", loc="left")
        ax.set_xlabel("dx, pad +X (cm)")
    axes[0].set_ylabel("dy, pad +Y (cm)")
    _touchdown_legend(fig, plt)
    fig.subplots_adjust(wspace=0.12)
    save(fig, out, "fig3_touchdown_scatter")
    plt.close(fig)

    # (supplementary) raw camera-point readings, with the configured offset shown
    xr = np.r_[v.td_dx_cm, 0.0, offset[0]]; yr = np.r_[v.td_dy_cm, 0.0, offset[1]]
    cx, cy = (np.nanmax(xr) + np.nanmin(xr)) / 2, (np.nanmax(yr) + np.nanmin(yr)) / 2
    span = np.ceil(max(np.nanmax(xr) - np.nanmin(xr), np.nanmax(yr) - np.nanmin(yr)) / 2 + 2.0)
    fig, axes = plt.subplots(1, 3, figsize=(W2, 2.55), sharex=True, sharey=True)
    for k, (ax, arm) in enumerate(zip(axes, ARMS)):
        p = v[v["mode"] == arm]
        if not len(p):
            continue
        _touchdown_panel(plt, ax, p.td_dx_cm.to_numpy(float), p.td_dy_cm.to_numpy(float), arm, 1)
        ax.plot(*offset, "D", color=INK2, ms=4.5, mfc="none", zorder=4)
        ax.set_xlim(cx - span, cx + span); ax.set_ylim(cy - span, cy + span)
        ax.set_title(f"({'abc'[k]}) {ARM_NAME[arm]} (n={len(p)})\nmedian {p.td_radial_cm.median():.1f} cm (camera point)",
                     loc="left")
        ax.set_xlabel("dx, pad +X (cm)")
    axes[0].set_ylabel("dy, pad +Y (cm)")
    _touchdown_legend(fig, plt, Line2D([], [], ls="none", marker="D", ms=4.5, mfc="none", color=INK2,
                                       label=f"expected camera point ({offset[0]:+.0f}, {offset[1]:+.0f}) cm"))
    fig.subplots_adjust(wspace=0.12)
    save(fig, out, "figS1_touchdown_camera_point")
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(W2, 2.3), sharey=True)
    for k, (ax, col, ttl) in enumerate(((axes[0], "td_radial_cm", "Camera point, raw (pre-registered primary)"),
                                        (axes[1], "td_radial_corr_cm", "Controlled point (reference offset removed)"))):
        for arm in ARMS:
            x = np.sort(v.loc[v["mode"] == arm, col].dropna().to_numpy())
            if len(x):
                yv = np.arange(1, len(x) + 1) / len(x)
                ax.step(np.r_[0, x], np.r_[0, yv], where="post", color=COLOR[arm], label=ARM_NAME[arm])
                ax.plot(x, yv, MARK[arm], color=COLOR[arm], ms=3)
        ax.axhline(0.5, color=INK2, lw=0.6, ls=":")
        ax.set_xlabel("Radial touchdown error (cm)")
        ax.set_xlim(left=0)
        ax.set_title(ttl, loc="left")
        panel_label(ax, k)
    axes[0].set_ylabel("Fraction of sorties")
    axes[1].legend(frameon=False, loc="lower right")
    save(fig, out, "fig4_touchdown_ecdf")
    plt.close(fig)


def fig_recovery(plt, traces, out):
    from matplotlib.patches import Patch
    arms = [a for a in ARMS if traces.get(a)]
    if not arms:
        return
    fig, axes = plt.subplots(1, len(arms), figsize=(W2, 2.1), sharey=True, sharex=True)
    axes = np.atleast_1d(axes)
    for k, (ax, arm) in enumerate(zip(axes, arms)):
        for tt, pos in traces[arm]:
            tt, pos = np.asarray(tt, float), np.asarray(pos, float) * 100
            gap = np.where(np.diff(tt) > 0.25)[0] + 1            # do not draw across detection gaps
            tt, pos = np.insert(tt, gap, np.nan), np.insert(pos, gap, np.nan)
            ax.plot(tt, pos, color=COLOR[arm], lw=0.7, alpha=0.6)
        ax.axhspan(-SETTLE_RADIUS_M * 100, SETTLE_RADIUS_M * 100, color=GRID, zorder=0)
        ax.set_title(f"({'abc'[k]}) {ARM_NAME[arm]} (n={len(traces[arm])})", loc="left")
    axes[0].set_ylabel("Along-step position (cm)")
    axes[len(axes) // 2].set_xlabel("Time after the final step to the origin (s)")
    axes[-1].legend(handles=[Patch(color=GRID, label=f"±{SETTLE_RADIUS_M * 100:.0f} cm settling band")],
                    frameon=False, loc="upper right")
    save(fig, out, "fig5_recovery")
    plt.close(fig)


def fig_nis(plt, nis, out, per_sortie_mean=None):
    """ECDF of the external-nav position NIS against the chi-square(2) reference (log x)."""
    arms = [a for a in ARMS if a in nis and len(nis[a])]
    if not arms:
        return
    fig, ax = plt.subplots(figsize=(W1, 2.3))
    xs = np.logspace(-4, np.log10(30), 400)
    ax.plot(xs, stats.chi2.cdf(xs, 2), color=INK, lw=1.2, ls="--", label=r"$\chi^2_2$ (consistent filter)")
    for arm in arms:
        x = np.sort(np.clip(np.asarray(nis[arm], float), 1e-4, None))
        m = (per_sortie_mean or {}).get(arm, np.mean(x))
        ax.step(x, np.arange(1, len(x) + 1) / len(x), where="post", color=COLOR[arm],
                label=f"{ARM_NAME[arm]} (mean {m:.2f})")
    ax.axvline(5.991, color=INK2, lw=0.6, ls=":")
    ax.text(5.991 * 1.08, 0.04, r"$\chi^2_{2,0.95}$", fontsize=6.5, color=INK2)
    ax.set_xscale("log")
    ax.set_xlim(1e-4, 30); ax.set_ylim(0, 1.02)
    ax.set_xlabel("External-navigation position NIS")
    ax.set_ylabel("Fraction of updates")
    ax.legend(frameon=False, loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=2, fontsize=6.5,
              columnspacing=1.0, handlelength=1.6)
    save(fig, out, "fig6_nis")
    plt.close(fig)


def fig_decomposition(plt, per, out):
    v = per[per.valid]
    cols = [("lastfix_radial_pad_cm", "EKF error\nat last fix"), ("blind_drift_cm", "Blind\ndrift"),
            ("td_spread_cm", "Spread"), ("td_radial_corr_cm", "Touchdown\n(controlled)")]
    cols = [c for c in cols if c[0] in v and v[c[0]].notna().any()]
    if not cols:
        return
    fig, ax = plt.subplots(figsize=(W1, 2.3))
    w = 0.26
    top = 0.0
    for k, arm in enumerate(ARMS):
        p = v[v["mode"] == arm]
        med = [p[c].median() for c, _ in cols]
        q1 = [p[c].quantile(.25) for c, _ in cols]
        q3 = [p[c].quantile(.75) for c, _ in cols]
        x = np.arange(len(cols)) + (k - 1) * w
        ax.bar(x, med, w * 0.9, color=COLOR[arm], label=f"{ARM_SHORT[arm]} (n={len(p)})")
        ax.errorbar(x, med, yerr=[np.subtract(med, q1), np.subtract(q3, med)], fmt="none", ecolor=INK2, lw=0.8)
        top = max(top, np.nanmax(q3))
    ax.set_xticks(np.arange(len(cols)))
    ax.set_xticklabels([l for _, l in cols], fontsize=7)
    ax.grid(axis="x", visible=False)
    ax.set_ylabel("cm (median, IQR)")
    ax.set_ylim(0, top * 1.28)
    ax.legend(frameon=False, ncol=3, loc="upper left")
    save(fig, out, "fig7_error_decomposition")
    plt.close(fig)


# ═════════════════════════════════════════════════════════════════════════
# analyze
# ═════════════════════════════════════════════════════════════════════════
def verify_match(s, bm):
    mode = s["mode"]
    want_tuned = fnum((s.get("args") or {}).get("sigma_tuned"))
    if bm.get("bin_arm") != mode:
        return False, f"log looks like {bm.get('bin_arm')}, sortie is {mode}"
    if mode == "tuned" and math.isfinite(want_tuned) and abs(bm["bin_flat_sigma_m"] - max(want_tuned, SIGMA_FLOOR_M)) > 5e-4:
        return False, f"flat sigma {bm['bin_flat_sigma_m']:.4f} m != --sigma-tuned {want_tuned:.4f}"
    if bm.get("n_steps", 0) < 3:
        return False, "no maneuver steps in this log"
    return True, "ok"


def content_match(s, df, d):
    """Do the vision positions the Pi sent (vision_frames.csv) equal the ones the flight controller
    logged (VISP)? Primary key: VISP.RTimeUS = the Pi's capture timestamp sent with every message,
    so rows join exactly. Fallback: FC time + the logged clock offset. Returns (median |difference|
    in cm, fraction of VISP rows paired) or (nan, 0) when it cannot be checked."""
    if df is None or "x_sent" not in df:
        return np.nan, 0.0
    sent = df[df.get("sent", df.get("accepted")) == 1][["t_capture", "t_send", "x_sent", "y_sent"]].apply(
        pd.to_numeric, errors="coerce").dropna()
    visp = arr(d, "VISP", 7)
    if len(sent) < 50 or len(visp) < 50:
        return np.nan, 0.0
    if np.nanmedian(visp[:, 6]) > 1e8:                     # remote timestamp present (epoch seconds)
        sent = sent.sort_values("t_capture")
        tc = sent.t_capture.to_numpy()
        j = np.clip(np.searchsorted(tc, visp[:, 6]), 1, len(tc) - 1)
        j = np.where(np.abs(tc[j - 1] - visp[:, 6]) < np.abs(tc[j] - visp[:, 6]), j - 1, j)
        ok = np.abs(tc[j] - visp[:, 6]) < 5e-6
        if ok.sum() >= 30:
            dd = np.hypot(sent.x_sent.to_numpy()[j[ok]] - visp[ok, 1], sent.y_sent.to_numpy()[j[ok]] - visp[ok, 2])
            return float(np.median(dd)) * 100, float(ok.mean())
        return np.inf, 0.0                                 # timestamps exist but none belong to this sortie
    off = fnum(s.get("fc_clock_offset_s"))
    if not math.isfinite(off):
        return np.nan, 0.0
    sent = sent.sort_values("t_send")
    ts = sent.t_send.to_numpy()
    best = (np.inf, 0.0)
    for lag in (0.0, -0.05, 0.05, -0.1, 0.1):                      # clock offset is good to ~10 ms
        tv = visp[:, 0] + off + lag
        j = np.clip(np.searchsorted(ts, tv), 1, len(ts) - 1)
        j = np.where(np.abs(ts[j - 1] - tv) < np.abs(ts[j] - tv), j - 1, j)
        ok = np.abs(ts[j] - tv) < 0.02
        if ok.sum() < 30:
            continue
        dd = np.hypot(sent.x_sent.to_numpy()[j[ok]] - visp[ok, 1], sent.y_sent.to_numpy()[j[ok]] - visp[ok, 2])
        med = float(np.median(dd)) * 100
        if med < best[0]:
            best = (med, float(ok.mean()))
    return best if math.isfinite(best[0]) else (np.nan, 0.0)


CONTENT_OK_CM = 0.5          # same flight: identical positions (mm); a different flight differs by cm


def match_bins(sorties, bins_dir, cache_dir, jobs, force=False):
    """sortie_id -> bin path. A .bin inside a sortie folder wins. External logs are paired by
    CONTENT (VISP positions == the positions the Pi sent, at the same instants), checked against
    the arm signature; if content cannot be checked, by flight order + arm signature."""
    inside = {}
    for s in sorties:
        b = sorted(glob.glob(os.path.join(s["_dir"], "*.bin")) + glob.glob(os.path.join(s["_dir"], "*.BIN")))
        if b:
            inside[s["sortie_id"]] = b[0]
    ext = []
    if bins_dir:
        ext = sorted(set(glob.glob(os.path.join(os.path.expanduser(bins_dir), "*.bin")) +
                         glob.glob(os.path.join(os.path.expanduser(bins_dir), "*.BIN"))), key=bin_number)
    parsed = parse_bins(sorted(set(list(inside.values()) + ext)), cache_dir, jobs)
    metrics = {p: bin_metrics(d) for p, d in parsed.items()}
    ext = [p for p in ext if metrics[p].get("bin_ok") and metrics[p].get("n_steps", 0) >= 3]  # skip ground/test logs
    frames = {s["sortie_id"]: load_frames(s["_dir"]) for s in sorties}
    pairs, rows, used, j = {}, [], set(), 0
    for s in sorties:
        sid = s["sortie_id"]
        if sid in inside:
            p = inside[sid]
            ok, why = verify_match(s, metrics[p])
            cm, frac = content_match(s, frames[sid], parsed[p])
            if not math.isnan(cm) and cm > CONTENT_OK_CM:
                ok, why = False, f"vision positions differ by {cm:.2f} cm - not this flight"
            pairs[sid] = p if (ok or force) else None
            rows.append(dict(sortie_id=sid, mode=s["mode"], bin=os.path.basename(p), source="in folder",
                             bin_arm=metrics[p].get("bin_arm"), content_diff_cm=cm, verified=ok, note=why))
            used.add(p)
            continue
        cands = [p for p in ext if p not in used and verify_match(s, metrics[p])[0]]
        scored = [(content_match(s, frames[sid], parsed[p]), p) for p in cands]
        checkable = [(cm, frac, p) for (cm, frac), p in scored if not math.isnan(cm)]
        if checkable:
            cm, frac, p = min(checkable)
            ok = cm <= CONTENT_OK_CM
            note = (f"content match {cm:.3f} cm over {frac * 100:.0f}% of VISP rows" if ok
                    else f"best log still differs by {cm:.2f} cm - no log of this flight")
        else:                                             # fall back to order + signature
            later = [p for p in cands if ext.index(p) >= j]
            p = later[0] if later else None
            cm, ok = np.nan, p is not None
            note = "order + arm signature (content not checkable)" if p else "no matching log found"
        if p is None or not ok:
            pairs[sid] = None
            rows.append(dict(sortie_id=sid, mode=s["mode"], bin="" if p is None else os.path.basename(p),
                             source="--bins", bin_arm="", content_diff_cm=cm, verified=False, note=note))
            continue
        pairs[sid] = p
        used.add(p)
        j = ext.index(p) + 1
        rows.append(dict(sortie_id=sid, mode=s["mode"], bin=os.path.basename(p), source="--bins",
                         bin_arm=metrics[p].get("bin_arm"), content_diff_cm=cm, verified=True, note=note))
    unused = [os.path.basename(x) for x in ext if x not in used]
    return pairs, metrics, pd.DataFrame(rows), unused


def cmd_match_bins(a):
    sorties = find_sorties(a.logs, a.experiment)
    if not sorties:
        sys.exit("No sorties found.")
    pairs, _, table, unused = match_bins(sorties, a.bins, os.path.join(a.logs, "_bin_cache"), a.jobs)
    print(table.to_string(index=False))
    if unused:
        print(f"\nLogs not used: {', '.join(unused)}")
    bad = table[~table.verified]
    if len(bad):
        sys.exit(f"\n{len(bad)} sortie(s) without a verified log - fix before analysing.")
    if a.copy:
        for s in sorties:
            p = pairs.get(s["sortie_id"])
            if p and os.path.dirname(os.path.abspath(p)) != os.path.abspath(s["_dir"]):
                shutil.copy2(p, os.path.join(s["_dir"], os.path.basename(p)))
        print("\nCopied every verified .bin into its sortie folder.")


def cmd_analyze(a):
    os.makedirs(a.out, exist_ok=True)
    sorties = find_sorties(a.logs, a.experiment)
    if not sorties:
        sys.exit(f"No sorties for experiment {a.experiment} under {a.logs}.")
    versions = sorted({s.get("version", "?") for s in sorties})
    models = sorted({json.dumps(s.get("model"), sort_keys=True) for s in sorties})
    cameras = sorted({(s.get("camera") or {}).get("kind", (s.get("args") or {}).get("camera", "?")) for s in sorties})
    tuned_vals = sorted({round(fnum((s.get("args") or {}).get("sigma_tuned")), 5) for s in sorties if s["mode"] == "tuned"})
    print(f"  {len(sorties)} sorties | versions {versions} | cameras {cameras} | A' sigma {tuned_vals}")
    if len(models) > 1:
        print("  WARNING: more than one VQA model in these sorties - the campaign must use one frozen model.")

    wbk = read_workbook(a.workbook) if a.workbook else {}
    td_csv = pd.read_csv(a.touchdown).set_index("sortie_id") if a.touchdown else None
    pairs, bmet, match_table, unused = (match_bins(sorties, a.bins, os.path.join(a.out, "_bin_cache"), a.jobs, a.force_bins)
                                        if (a.bins or any(glob.glob(os.path.join(s["_dir"], "*.bin")) for s in sorties))
                                        else ({}, {}, pd.DataFrame(), []))
    if len(match_table):
        match_table.to_csv(os.path.join(a.out, "bin_match.csv"), index=False)
        bad = match_table[~match_table.verified]
        print(f"  DataFlash logs: {int(match_table.verified.sum())}/{len(match_table)} sorties matched and verified")
        if len(bad) and not a.force_bins:
            print(bad.to_string(index=False))
            sys.exit("  Unverified log matches - see bin_match.csv (or run with --force-bins).")

    rows, params_seen = [], []
    nis_by, pts_by, tr_by = {}, {}, {}            # per sortie; filtered to valid sorties below
    for s in sorties:
        sid, arm = s["sortie_id"], s["mode"]
        df = load_frames(s["_dir"])
        if df is not None:
            df = df.copy()
        m = dict(sortie_id=sid, mode=arm, version=s.get("version"), completed=bool(s.get("completed")),
                 abort=s.get("abort_reason", ""), date_time=s.get("_t0"))
        # truth: workbook > csv > prompt
        truth, src = s.get("touchdown_truth") or {}, "prompt"
        if td_csv is not None and sid in td_csv.index:
            truth, src = dict(dx_cm=float(td_csv.loc[sid].dx_cm), dy_cm=float(td_csv.loc[sid].dy_cm)), "csv"
        status = ""
        if sid in wbk:
            status = wbk[sid]["status"]
            if math.isfinite(wbk[sid]["dx_cm"]) and math.isfinite(wbk[sid]["dy_cm"]):
                truth, src = dict(dx_cm=wbk[sid]["dx_cm"], dy_cm=wbk[sid]["dy_cm"]), wbk[sid]["source"]
        m["td_dx_cm"], m["td_dy_cm"] = fnum(truth.get("dx_cm")), fnum(truth.get("dy_cm"))
        m["td_radial_cm"] = math.hypot(m["td_dx_cm"], m["td_dy_cm"]) if math.isfinite(m["td_dx_cm"]) else np.nan
        m["truth_source"], m["workbook_status"] = src, status
        m["valid"] = bool(s.get("completed")) and (status == "Valid" if wbk else True) and math.isfinite(m["td_radial_cm"])
        le = [e for e in s.get("events", []) if e.get("event") == "LANDING_END"]
        le = le[-1] if le else {}
        m["landing_result"] = le.get("result", "")
        m["lastfix_wx_cm"] = fnum(le.get("last_fix_cg_x")) * 100
        m["lastfix_wy_cm"] = fnum(le.get("last_fix_cg_y")) * 100
        m["lastfix_h_m"], m["blind_s"] = fnum(le.get("last_fix_h")), fnum(le.get("blind_s"))
        ekf_final = s.get("ekf_final_m") or [np.nan, np.nan]
        m["ekf_td_x_cm"], m["ekf_td_y_cm"] = fnum(ekf_final[0]) * 100, fnum(ekf_final[1]) * 100
        m.update(csv_metrics(s, df))
        p = pairs.get(sid)
        if p:
            bm = dict(bmet[p])
            pts_by[sid] = list(bm.pop("_visp_man", []))
            nis_by[sid] = list(bm.pop("_nis", []))
            params_seen.append(bm.pop("_params", {}))
            for k in ("ekf_td_x_cm", "ekf_td_y_cm"):          # the .bin is the better source
                if k in bm and math.isfinite(bm[k]):
                    m[k] = bm[k]
            m.update({k: v for k, v in bm.items() if k not in ("ekf_td_x_cm", "ekf_td_y_cm")})
            m["bin_ekf_td_x_cm"], m["bin_ekf_td_y_cm"] = bm.get("ekf_td_x_cm"), bm.get("ekf_td_y_cm")
        elif df is not None:
            f = df[(df.phase == "MANEUVER") & (df.accepted == 1)][["tilt_deg", "sigma_sent"]].dropna()
            pts_by[sid] = f.to_numpy().tolist()
        # recovery trace
        steps = [e for e in s["events"] if e["event"] == "STEP"]
        t_end = event_t(s, "DESCENT")
        if df is not None and len(steps) >= 2 and t_end:
            u = unit(steps[-2].get("target_x", 0.0), steps[-2].get("target_y", 0.0))
            w = add_cg_columns(s, df[(df.t >= steps[-1]["t"]) & (df.t <= t_end) & (df.detected == 1)].sort_values("t").copy())
            if len(w) > SMOOTH_FRAMES:
                along = rolling_median(w.cg_x) * u[0] + rolling_median(w.cg_y) * u[1]
                tr_by[sid] = (w.t.to_numpy() - steps[-1]["t"], along)
        rows.append(m)
    per = pd.DataFrame(rows)

    # parameters identical across the campaign?
    if params_seen:
        keys = sorted(set().union(*params_seen))
        varying = {k: sorted({round(ps.get(k, np.nan), 5) for ps in params_seen}) for k in keys}
        varying = {k: v for k, v in varying.items() if len(v) > 1}
        with open(os.path.join(a.out, "fc_params.json"), "w", encoding="utf-8") as f:
            json.dump(dict(params_first_log=params_seen[0], varying=varying), f, indent=1)
    else:
        varying = None

    # ── pad-frame mapping and reference offset ──────────────────────────────
    v = per[per.valid]
    E_world = v[["ekf_td_x_cm", "ekf_td_y_cm"]].to_numpy(float)
    T_pad = v[["td_dx_cm", "td_dy_cm"]].to_numpy(float)
    if a.pad_map == "auto":
        mp, scores = auto_map(E_world, T_pad)
        map_note = (f"auto: {mp} ({describe_map(mp)}); residual {scores[0][0]:.2f} cm vs next best "
                    f"{scores[1][0]:.2f} cm ({scores[1][1]})" if scores else "auto: not enough data, R0 assumed")
    else:
        mp, map_note = a.pad_map, f"given: {a.pad_map} ({describe_map(a.pad_map)})"
    M = map_matrix(mp)
    print(f"  pad-frame mapping: {map_note}")
    for src, dst in ((("ekf_td_x_cm", "ekf_td_y_cm"), ("ekf_td_padx_cm", "ekf_td_pady_cm")),
                     (("lastfix_wx_cm", "lastfix_wy_cm"), ("lastfix_padx_cm", "lastfix_pady_cm"))):
        XY = per[list(src)].to_numpy(float) @ M.T
        per[dst[0]], per[dst[1]] = XY[:, 0], XY[:, 1]
    per["lastfix_radial_pad_cm"] = np.hypot(per.lastfix_padx_cm, per.lastfix_pady_cm)
    if a.ref_offset_cm:
        offset, off_src = np.array(a.ref_offset_cm, float), "measured (--ref-offset-cm)"
    else:
        d = (per.loc[per.valid, ["td_dx_cm", "td_dy_cm"]].to_numpy(float) -
             per.loc[per.valid, ["ekf_td_padx_cm", "ekf_td_pady_cm"]].to_numpy(float))
        d = d[np.isfinite(d).all(1)]
        offset = d.mean(0) if len(d) else np.zeros(2)
        off_src = "estimated from the data (all arms pooled) - DESCRIPTIVE ONLY; measure it for the paper"
    per["td_corr_dx_cm"] = per.td_dx_cm - offset[0]
    per["td_corr_dy_cm"] = per.td_dy_cm - offset[1]
    per["td_radial_corr_cm"] = np.hypot(per.td_corr_dx_cm, per.td_corr_dy_cm)
    per["blind_drift_cm"] = np.hypot(per.td_corr_dx_cm - per.lastfix_padx_cm, per.td_corr_dy_cm - per.lastfix_pady_cm)
    per["ekf_td_error_cm"] = np.hypot(per.td_corr_dx_cm - per.ekf_td_padx_cm, per.td_corr_dy_cm - per.ekf_td_pady_cm)
    for arm in ARMS:
        i = per.valid & (per["mode"] == arm)
        c = per.loc[i, ["td_dx_cm", "td_dy_cm"]].mean()
        per.loc[i, "td_spread_cm"] = np.hypot(per.loc[i, "td_dx_cm"] - c.td_dx_cm, per.loc[i, "td_dy_cm"] - c.td_dy_cm)

    # data quality flags
    def flags(r):
        f = []
        if r.get("ekf_failsafe") is True: f.append("EKF failsafe")
        if fnum(r.get("lane_switches")) > 0: f.append("lane switch")
        if fnum(r.get("flow_gaps_gt_0p3s")) > 0: f.append("flow dropout")
        if fnum(r.get("rng_zero_readings")) > 0: f.append("rangefinder zeros")
        if fnum(r.get("relpos_lost_samples")) > 0: f.append("EKF position lost")
        if fnum(r.get("visp_ignored_frac")) > 0.05: f.append("vision ignored >5%")
        if fnum(r.get("maneuver_accept_frac")) < 0.9: f.append("maneuver frames <90%")
        tm = fnum(r.get("tilt_max_deg")) if math.isfinite(fnum(r.get("tilt_max_deg"))) else fnum(r.get("bin_tilt_max_deg"))
        if math.isfinite(tm) and not 15 <= tm <= 22: f.append(f"tilt {tm:.1f} deg")
        if not r.get("bin_ok", False) and pairs: f.append("no .bin")
        return "; ".join(f)
    per["quality_flags"] = per.apply(flags, axis=1)
    if a.exclude_flagged:
        per.loc[per.quality_flags != "", "valid"] = False
    qcols = ["sortie_id", "mode", "valid", "workbook_status", "quality_flags", "ekf_failsafe", "lane_switches",
             "flow_gaps_gt_0p3s", "rng_zero_readings", "relpos_lost_samples", "pos_timeout_latched",
             "visp_ignored_frac", "maneuver_accept_frac", "tilt_max_deg", "bin_tilt_max_deg", "latency_med_ms", "bin_file"]
    per[[c for c in qcols if c in per]].to_csv(os.path.join(a.out, "data_quality.csv"), index=False)
    per.to_csv(os.path.join(a.out, "per_sortie.csv"), index=False)

    # ── frame-level data (figures, H3) from the VALID sorties only ─────────
    v = per[per.valid]
    nis, man_pts, traces = {}, {}, {}
    for sid, arm in zip(v.sortie_id, v["mode"]):
        if sid in nis_by:
            nis.setdefault(arm, []).extend(nis_by[sid])
        if sid in pts_by:
            man_pts.setdefault(arm, []).extend(pts_by[sid])
        if sid in tr_by:
            traces.setdefault(arm, []).append(tr_by[sid])

    # ── arm summaries ───────────────────────────────────────────────────────
    summ = []
    for arm in ARMS:
        p = v[v["mode"] == arm]
        if not len(p):
            continue
        tr = p.td_radial_cm
        r = dict(arm=ARM_NAME[arm], n_valid=len(p), n_flown=int((per["mode"] == arm).sum()),
                 CEP50_cm=tr.median(), CEP50_ci=boot_ci(tr, np.median), CEP95_cm=tr.quantile(.95),
                 centroid_dx_cm=p.td_dx_cm.mean(), centroid_dy_cm=p.td_dy_cm.mean(),
                 spread_rms_cm=float(np.sqrt((p.td_spread_cm ** 2).mean())), spread_cep50_cm=p.td_spread_cm.median(),
                 CEP50_corr_cm=p.td_radial_corr_cm.median(), CEP50_corr_ci=boot_ci(p.td_radial_corr_cm, np.median))
        for c in ("visp_man_med_cm", "visp_man_p95_cm", "visp_rho_sigma_tilt", "visp_pinned_frac", "nis_mean",
                  "nis_frac_gt_5p99", "pos_innov_rms_cm", "tilt_max_deg", "bin_tilt_max_deg", "overshoot_vis_cm",
                  "settle_vis_s", "hold_rms_vis_cm", "lastfix_radial_pad_cm", "blind_drift_cm", "ekf_td_error_cm",
                  "latency_med_ms", "blur_hover_med", "sigma_maneuver_med_cm", "sigma_maneuver_p95_cm"):
            if c in p:
                r[c + "_median"] = p[c].median()
        summ.append(r)
    summ = pd.DataFrame(summ)
    summ.to_csv(os.path.join(a.out, "summary_by_arm.csv"), index=False)

    # ── tests ───────────────────────────────────────────────────────────────
    B = v[v["mode"] == "adaptive"]
    tests = []
    outcomes = [("td_radial_cm", "primary", "Touchdown radial error from the marker (pre-registered)"),
                ("td_radial_corr_cm", "secondary", "Touchdown radial error after the reference offset"),
                ("td_spread_cm", "secondary", "Distance to the arm's own touchdown centroid (precision)"),
                ("td_dx_cm", "secondary", "Touchdown dx (pad X)"), ("td_dy_cm", "secondary", "Touchdown dy (pad Y)"),
                ("lastfix_radial_pad_cm", "secondary", "EKF-controlled point vs marker at the last camera fix"),
                ("blind_drift_cm", "secondary", "Blind-segment drift (last fix -> touchdown)"),
                ("overshoot_vis_cm", "secondary", "Overshoot after the last step"),
                ("settle_vis_s", "secondary", "Settling time to 5 cm"),
                ("hold_rms_vis_cm", "secondary", "Hold RMS in CONVERGE"),
                ("nis_mean", "secondary", "Mean position NIS (ideal 2)"),
                ("pos_innov_rms_cm", "secondary", "Position innovation RMS")]
    for metric, kind, label in outcomes:
        if metric not in v:
            continue
        for ref in ("constant", "tuned"):
            R = v[v["mode"] == ref]
            if not len(B) or not len(R):
                continue
            c = compare(B[metric], R[metric])
            c.update(metric=metric, label=label, kind=kind, comparison=f"B vs {ARM_SHORT[ref]}")
            tests.append(c)
    for ref in ("constant", "tuned"):
        R = v[v["mode"] == ref]
        d, p = centroid_test(B[["td_dx_cm", "td_dy_cm"]].to_numpy(), R[["td_dx_cm", "td_dy_cm"]].to_numpy())
        tests.append(dict(metric="td_centroid_shift_cm", label="Distance between 2-D touchdown centroids (offset-free)",
                          kind="secondary", comparison=f"B vs {ARM_SHORT[ref]}", n_b=len(B), n_ref=len(R),
                          p=p, delta=np.nan, med_diff=d, ci_lo=np.nan, ci_hi=np.nan))
    tests = pd.DataFrame(tests)
    tests["p_holm"] = np.nan
    prim = tests.kind == "primary"
    if prim.sum() and tests.loc[prim, "p"].notna().all():
        tests.loc[prim, "p_holm"] = holm(tests.loc[prim, "p"].to_numpy())
    kw = [v.loc[v["mode"] == a_, "td_radial_cm"].dropna() for a_ in ARMS]
    kw_p = stats.kruskal(*kw).pvalue if all(len(k) >= 3 for k in kw) else np.nan
    tests.to_csv(os.path.join(a.out, "tests.csv"), index=False)

    write_latex(summ, v, a.out)
    write_latex_tests(tests, a.out)
    report = write_report(a, per, v, summ, tests, nis, kw_p, dict(versions=versions, cameras=cameras,
                          tuned=tuned_vals, models=models, map_note=map_note, offset=offset, off_src=off_src,
                          match=match_table, unused=unused, varying=varying), a.out)
    print("\n" + report)

    plt = setup_mpl()
    fig_manipulation(plt, man_pts, a.out)
    fig_touchdown(plt, per, a.out, offset)
    fig_recovery(plt, traces, a.out)
    fig_nis(plt, {k: np.asarray(x) for k, x in nis.items()}, a.out,
            {arm: v.loc[v["mode"] == arm, "nis_mean"].median() for arm in ARMS if "nis_mean" in v})
    fig_decomposition(plt, per, a.out)
    print(f"\nWrote tables, report and figures to {os.path.abspath(a.out)}")


def write_latex(summ, v, out):
    v = v.copy()
    if "maneuver_accept_frac" in v:
        v["maneuver_det_pct"] = v.maneuver_accept_frac * 100

    def cell(arm, key, fmt="{:.1f}"):
        p = v[v["mode"] == arm]
        if key not in p or not p[key].notna().any():
            return "--"
        x = p[key].dropna()
        q = [fmt.format(x.median()), fmt.format(x.quantile(.25)), fmt.format(x.quantile(.75))]
        return q[0] if q[1] == q[2] == q[0] else f"{q[0]} [{q[1]}, {q[2]}]"
    n = [int((v["mode"] == a).sum()) for a in ARMS]
    lines = [r"% requires \usepackage{booktabs}",
             r"\begin{table*}[t]", r"\centering", r"\footnotesize", r"\setlength{\tabcolsep}{6pt}",
             r"\caption{Flight results per arm, median [IQR] over sorties. Truth from the pad grid at the RGB "
             r"optical centre (camera point, pre-registered primary outcome). The controlled point is the "
             r"autopilot body origin, offset from the camera by \texttt{VISO\_POS}.}",
             r"\label{tab:results}", r"\begin{tabular}{@{}lccc@{}}", r"\toprule",
             r"Metric & A: const.\ 20\,cm & A$'$: tuned const. & B: VQA-EKF \\", r"\midrule",
             "Valid sorties & " + " & ".join(map(str, n)) + r" \\",
             r"\multicolumn{4}{@{}l}{\emph{Touchdown}} \\"]
    groups = [
        [("\\quad Error, camera point (cm)", "td_radial_cm", "{:.1f}"),
         ("\\quad Error, controlled point (cm)", "td_radial_corr_cm", "{:.1f}"),
         ("\\quad Distance to arm centroid (cm)", "td_spread_cm", "{:.1f}"),
         ("\\quad EKF error at last fix (cm)", "lastfix_radial_pad_cm", "{:.1f}"),
         ("\\quad Blind-segment drift (cm)", "blind_drift_cm", "{:.1f}")],
        [("\\quad Overshoot after step (cm)", "overshoot_vis_cm", "{:.1f}"),
         ("\\quad Settling time to 5\\,cm (s)", "settle_vis_s", "{:.1f}"),
         ("\\quad Hold RMS (cm)", "hold_rms_vis_cm", "{:.1f}")],
        [("\\quad $\\sigma$ used, median (cm)", "visp_man_med_cm", "{:.2f}"),
         ("\\quad $\\sigma$ used, p95 (cm)", "visp_man_p95_cm", "{:.2f}"),
         ("\\quad Mean position NIS (ideal 2)", "nis_mean", "{:.2f}"),
         ("\\quad Marker detection (\\%)", "maneuver_det_pct", "{:.0f}"),
         ("\\quad Max tilt (deg)", "tilt_max_deg", "{:.1f}")]]
    heads = [None, r"\multicolumn{4}{@{}l}{\emph{Step response}} \\",
             r"\multicolumn{4}{@{}l}{\emph{Estimator and conditions (maneuver)}} \\"]
    for head, items in zip(heads, groups):
        if head:
            lines.append(head)
        for label, key, fmt in items:
            lines.append(f"{label} & " + " & ".join(cell(a, key, fmt) for a in ARMS) + r" \\")
    lines += [r"\bottomrule", r"\end{tabular}", r"\end{table*}"]
    with open(os.path.join(out, "results_table.tex"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def write_latex_tests(tests, out):
    rows = [("td_radial_cm", "Error, camera pt.$^\\dagger$"),
            ("td_radial_corr_cm", "Error, controlled pt."),
            ("td_spread_cm", "Spread"),
            ("td_centroid_shift_cm", "Centroid shift$^\\ddagger$"),
            ("lastfix_radial_pad_cm", "EKF error, last fix"),
            ("blind_drift_cm", "Blind drift"),
            ("overshoot_vis_cm", "Overshoot")]

    def cells(metric, comp):
        r = tests[(tests.metric == metric) & (tests.comparison == comp)]
        if not len(r):
            return "--", "--"
        r = r.iloc[0]
        p = r.p_holm if math.isfinite(fnum(r.get("p_holm"))) else r.p
        pt = "$<$0.001" if p < 0.001 else f"{p:.2f}"
        if metric == "td_centroid_shift_cm":
            return f"{r.med_diff:.1f}", pt
        def m(x):
            x = 0.0 if abs(x) < 0.05 else x
            return "$0.0$" if x == 0.0 else f"${x:+.1f}$"
        return f"{m(r.med_diff)} [{m(r.ci_lo)}, {m(r.ci_hi)}]", pt
    lines = [r"% requires \usepackage{booktabs}",
             r"\begin{table}[t]", r"\centering", r"\footnotesize", r"\setlength{\tabcolsep}{3pt}",
             r"\caption{Arm B versus the constant-$\sigma$ arms: difference of medians, B $-$ comparator (cm), "
             r"with 95\% bootstrap CI; negative favours B. $p$: two-sided Mann--Whitney.}",
             r"\label{tab:tests}", r"\setlength{\tabcolsep}{1.5pt}", r"\begin{tabular}{@{}lcccc@{}}", r"\toprule",
             r" & \multicolumn{2}{c}{B vs A} & \multicolumn{2}{c}{B vs A$'$} \\",
             r"\cmidrule(lr){2-3}\cmidrule(lr){4-5}",
             r"Outcome & $\Delta$ [95\% CI] & $p$ & $\Delta$ [95\% CI] & $p$ \\", r"\midrule"]
    for metric, label in rows:
        a1, p1 = cells(metric, "B vs A")
        a2, p2 = cells(metric, "B vs A'")
        lines.append(f"{label} & {a1} & {p1} & {a2} & {p2} \\\\")
    lines += [r"\bottomrule",
              r"\multicolumn{5}{@{}p{0.97\columnwidth}@{}}{\scriptsize Touchdown outcomes (cm). Spread: distance "
              r"to the arm's own centroid. $^\dagger$Pre-registered primary outcome, Holm-adjusted. "
              r"$^\ddagger$Distance between 2-D touchdown centroids; permutation test.} \\",
              r"\end{tabular}", r"\end{table}"]
    with open(os.path.join(out, "results_tests.tex"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def fmt_test(r):
    if r is None or not math.isfinite(fnum(r.get("p"))):
        return "n/a"
    if r.get("metric") == "td_centroid_shift_cm":
        return f"centroid distance {r['med_diff']:.2f} cm, permutation p={r['p']:.3f}, n={r['n_b']}/{r['n_ref']}"
    ph = fnum(r.get("p_holm"))
    return (f"median diff {r['med_diff']:+.2f} (95% CI {r['ci_lo']:+.2f} to {r['ci_hi']:+.2f}), "
            f"Cliff's δ {r['delta']:+.2f}, Mann-Whitney p={r['p']:.3f}"
            + (f", Holm p={ph:.3f}" if math.isfinite(ph) else "") + f", n={r['n_b']}/{r['n_ref']}")


def verdict(r):
    if r is None or not math.isfinite(fnum(r.get("p"))):
        return "INCONCLUSIVE (too few sorties with data)"
    p = r["p_holm"] if math.isfinite(fnum(r.get("p_holm"))) else r["p"]
    if p < ALPHA:
        return "SUPPORTED (B better)" if r["med_diff"] < 0 else "CONTRADICTED (B significantly worse)"
    return "NOT SUPPORTED at α=0.05"


def write_report(a, per, v, summ, tests, nis, kw_p, info, out):
    L = [f"# VQA-EKF flight experiment {a.experiment} — automated report",
         f"_analyze_vqa_ekf_final.py v{ANALYZER_VERSION}_", ""]
    L += ["## Data", f"- Sorties found: {len(per)}; valid for the tests: {int(per.valid.sum())} "
          f"(A {int((v['mode'] == 'constant').sum())}, A′ {int((v['mode'] == 'tuned').sum())}, "
          f"B {int((v['mode'] == 'adaptive').sum())})",
          f"- Script versions: {', '.join(info['versions'])}; camera: {', '.join(map(str, info['cameras']))}; "
          f"A′ σ: {', '.join(str(x) for x in info['tuned'])} m; VQA models used: {len(info['models'])}"
          + (" (OK, one frozen model)" if len(info["models"]) == 1 else " — **more than one: check**")]
    if len(info["match"]):
        L.append(f"- DataFlash logs matched and verified: {int(info['match'].verified.sum())}/{len(info['match'])}"
                 + (f"; unused logs: {', '.join(info['unused'])}" if info["unused"] else ""))
    if info["varying"] is not None:
        L.append("- Flight-controller parameters identical in every log: "
                 + ("yes" if not info["varying"] else f"**no** — {info['varying']}"))
    L.append(f"- Pad grid vs EKF frame: {info['map_note']}")
    L.append(f"- Common reference offset (pad frame): ({info['offset'][0]:+.1f}, {info['offset'][1]:+.1f}) cm — {info['off_src']}")
    excl = per[~per.valid]
    if len(excl):
        L.append(f"- Excluded: {', '.join(f'{r.sortie_id} ({r.workbook_status or r.abort or r.quality_flags})' for r in excl.itertuples())}")
    flagged = per[per.quality_flags != ""]
    L.append(f"- Data-quality flags: {'none' if not len(flagged) else '; '.join(f'{r.sortie_id}: {r.quality_flags}' for r in flagged.itertuples())}")
    L.append("")

    L.append("## H0 — Manipulation check (did EKF3 use what each arm intended?)")
    for arm in ARMS:
        p = v[v["mode"] == arm]
        if not len(p):
            continue
        has_bin = "visp_man_med_cm" in p and p.visp_man_med_cm.notna().any()
        med = p.visp_man_med_cm.median() if has_bin else p.get("sigma_maneuver_med_cm", pd.Series([np.nan])).median()
        hi = p.visp_man_p95_cm.median() if has_bin else p.get("sigma_maneuver_p95_cm", pd.Series([np.nan])).median()
        src = "VISP.PErr (what EKF3 used)" if has_bin else "sigma_sent (script CSV)"
        if arm == "constant":
            ok = abs(med - 20.0) < 0.5
            extra = ""
        elif arm == "tuned":
            t = info["tuned"][0] * 100 if info["tuned"] else med
            ok = abs(med - t) < 0.1
            extra = ""
        else:
            rho = p.visp_rho_sigma_tilt.median() if "visp_rho_sigma_tilt" in p else p.get("rho_sigma_tilt", pd.Series([np.nan])).median()
            pin = p.visp_pinned_frac.median() if "visp_pinned_frac" in p else p.get("sigma_pinned_frac", pd.Series([np.nan])).median()
            match = p.sigma_model_match.min() if "sigma_model_match" in p and p.sigma_model_match.notna().any() else np.nan
            ok = (not math.isfinite(match) or match >= 0.999) and (not math.isfinite(pin) or pin < 0.5) and (rho > 0)
            extra = (f", ρ(σ, tilt) = {rho:.2f}, at the 1 cm floor {pin * 100:.0f}% of frames"
                     + (f", sent = model on {match * 100:.1f}% of frames" if math.isfinite(match) else ""))
        L.append(f"- {ARM_NAME[arm]}: median {med:.2f} cm, p95 {hi:.2f} cm in the maneuver{extra} [{src}] → **{'PASS' if ok else 'FAIL'}**")
    pb = v[v["mode"] == "adaptive"]
    if len(pb) and "visp_man_p95_cm" in pb:
        ratio = pb.visp_man_p95_cm.median() / max(pb.visp_man_med_cm.median(), 1e-6)
        L.append(f"  - Size of the adaptation: B's σ p95/median ×{ratio:.2f} in the maneuver; "
                 f"B median {pb.visp_man_med_cm.median():.2f} cm vs A′ {info['tuned'][0] * 100 if info['tuned'] else float('nan'):.2f} cm")
    L.append("")

    def get(metric, comp):
        r = tests[(tests.metric == metric) & (tests.comparison == comp)]
        return None if not len(r) else r.iloc[0].to_dict()
    L.append("## H1 / H2 — Touchdown (pad-grid truth)")
    L.append("Primary outcome as pre-registered: radial error from the marker centre.")
    for comp, h in (("B vs A", "H1"), ("B vs A'", "H2")):
        r = get("td_radial_cm", comp)
        L.append(f"- **{h}** {comp}: {fmt_test(r)} → **{verdict(r)}**")
    L.append(f"- Omnibus Kruskal–Wallis across the three arms: p={kw_p:.3f}")
    L.append("")
    L.append("Per arm (median [95% bootstrap CI]):")
    for r in summ.itertuples():
        L.append(f"- {r.arm}: CEP50 {r.CEP50_cm:.1f} [{r.CEP50_ci[0]:.1f}, {r.CEP50_ci[1]:.1f}] cm, CEP95 {r.CEP95_cm:.1f} cm; "
                 f"centroid ({r.centroid_dx_cm:+.1f}, {r.centroid_dy_cm:+.1f}) cm; spread about centroid RMS {r.spread_rms_cm:.1f} cm; "
                 f"after reference offset CEP50 {r.CEP50_corr_cm:.1f} [{r.CEP50_corr_ci[0]:.1f}, {r.CEP50_corr_ci[1]:.1f}] cm")
    L.append("")
    L.append("Offset-free comparisons (a common reference offset cancels in these):")
    for metric in ("td_centroid_shift_cm", "td_spread_cm", "td_radial_corr_cm"):
        for comp in ("B vs A", "B vs A'"):
            r = get(metric, comp)
            if r:
                L.append(f"- {r['label']}, {comp}: {fmt_test(r)}")
    L.append("")
    L.append("## Where the touchdown error comes from")
    for arm in ARMS:
        p = v[v["mode"] == arm]
        if len(p) and "blind_drift_cm" in p:
            L.append(f"- {ARM_NAME[arm]}: at the last camera fix ({p.lastfix_h_m.median():.2f} m, {p.blind_s.median():.1f} s before touchdown) "
                     f"the EKF-controlled point was {p.lastfix_radial_pad_cm.median():.1f} cm from the marker; "
                     f"blind-segment drift {p.blind_drift_cm.median():.1f} cm (median)")
    L.append("")
    L.append("## H3 — Filter consistency (position NIS, ideal mean 2.0; from the .bin)")
    for arm in ARMS:
        if arm in nis and len(nis[arm]):
            x = np.asarray(nis[arm])
            note = "consistent" if 1.0 <= x.mean() <= 3.0 else ("R larger than the real error (conservative)" if x.mean() < 1.0 else "R smaller than the real error (over-confident)")
            ps = v.loc[v["mode"] == arm, "nis_mean"].median() if "nis_mean" in v else np.nan
            L.append(f"- {ARM_NAME[arm]}: mean NIS {x.mean():.2f} (all frames pooled; median of per-sortie means "
                     f"{ps:.2f}), fraction > χ²₂(0.95)=5.99: {np.mean(x > 5.991):.3f} (ideal 0.05) — {note}")
    L.append("")
    L.append("## Secondary outcomes (exploratory, not adjusted for multiplicity)")
    for r in tests[(tests.kind == "secondary") & (~tests.metric.isin(["td_centroid_shift_cm", "td_spread_cm", "td_radial_corr_cm"]))].to_dict("records"):
        L.append(f"- {r['label']}, {r['comparison']}: {fmt_test(r)}")
    L.append("")
    L.append("## Validity")
    tm = v.tilt_max_deg if "tilt_max_deg" in v and v.tilt_max_deg.notna().any() else v.get("bin_tilt_max_deg", pd.Series([np.nan]))
    L.append(f"- Max maneuver tilt, median over sorties: {tm.median():.1f} deg (range {tm.min():.1f}–{tm.max():.1f}; target 15–22)")
    L.append(f"- Sorties without touchdown truth: {int(per.td_radial_cm.isna().sum())}")
    L.append(f"- Not completed: {int((~per.completed).sum())} (report them; do not silently drop)")
    txt = "\n".join(L)
    with open(os.path.join(out, "report.md"), "w", encoding="utf-8") as f:
        f.write(txt)
    return txt


# ═════════════════════════════════════════════════════════════════════════
# tune / schedule (unchanged behaviour)
# ═════════════════════════════════════════════════════════════════════════
def cmd_tune(a):
    vals = []
    for s in find_sorties(a.logs):
        df = load_frames(s["_dir"])
        if df is None:
            continue
        x = df.loc[(df.phase == "HOVER_BASE") & (df.detected == 1), "sigma_model"].dropna()
        vals.extend(x.tolist())
        print(f"{os.path.basename(s['_dir'])}: {len(x)} frames, median model sigma {x.median() * 100:.2f} cm")
    if not vals:
        sys.exit("No HOVER_BASE frames found.")
    print(f"\nRecommended Arm A' value: --sigma-tuned {float(np.median(vals)):.4f}   (median of {len(vals)} frames)")


def cmd_schedule(a):
    rng = random.Random(a.seed)
    rows, n = [], 0
    for b in range(a.n_per_arm):
        block = ARMS[:]
        rng.shuffle(block)
        for m in block:
            n += 1
            rows.append(dict(order=n, block=b + 1, mode=m, sortie_id=f"{a.experiment}_S{n:02d}"))
    df = pd.DataFrame(rows)
    df.to_csv(a.out, index=False)
    print(df.to_string(index=False))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    z = sub.add_parser("analyze")
    z.add_argument("--logs", required=True, help="paper log root copied from the Pi")
    z.add_argument("--experiment", default="E1")
    z.add_argument("--out", default="results")
    z.add_argument("--bins", help="folder with the DataFlash .bin logs (Mission Planner download)")
    z.add_argument("--workbook", help="campaign workbook (.xlsx): status + touchdown truth")
    z.add_argument("--touchdown", help="CSV sortie_id,dx_cm,dy_cm (alternative truth source)")
    z.add_argument("--pad-map", default="auto", choices=["auto"] + list(MAPS))
    z.add_argument("--ref-offset-cm", nargs=2, type=float, metavar=("DX", "DY"))
    z.add_argument("--exclude-flagged", action="store_true", help="drop sorties with data-quality flags")
    z.add_argument("--force-bins", action="store_true", help="accept unverified .bin matches (not recommended)")
    z.add_argument("--jobs", type=int, default=max(1, min(8, (os.cpu_count() or 2) - 1)))
    mb = sub.add_parser("match-bins")
    mb.add_argument("--logs", required=True)
    mb.add_argument("--experiment", default="E1")
    mb.add_argument("--bins", required=True)
    mb.add_argument("--copy", action="store_true", help="copy each verified .bin into its sortie folder")
    mb.add_argument("--jobs", type=int, default=max(1, min(8, (os.cpu_count() or 2) - 1)))
    t = sub.add_parser("tune")
    t.add_argument("--logs", required=True)
    s = sub.add_parser("schedule")
    s.add_argument("--n-per-arm", type=int, default=15)
    s.add_argument("--seed", type=int, default=2026)
    s.add_argument("--experiment", default="E1")
    s.add_argument("--out", default="schedule.csv")
    a = ap.parse_args()
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except (AttributeError, ValueError):
            pass
    {"analyze": cmd_analyze, "match-bins": cmd_match_bins, "tune": cmd_tune, "schedule": cmd_schedule}[a.cmd](a)


if __name__ == "__main__":
    main()
