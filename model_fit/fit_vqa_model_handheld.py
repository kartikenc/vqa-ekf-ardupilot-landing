#!/usr/bin/env python3
"""
fit_vqa_model_handheld.py — fit the VQA noise model WITHOUT a rig or ground truth
==================================================================================
Gate G1 for the RGB camera. Offline tool: it never touches the flight code.

Input: one or more sessions recorded with the flight script's --bench mode while the
drone (props OFF, Cube + camera powered) is HELD BY HAND over the marker and tilted
slowly through 0-35 deg. The hand moves slowly; the camera's measurement error jumps
from frame to frame. A short Savitzky-Golay smoother (default 0.4 s) follows the hand,
and what is left (the residual) is the camera's frame-to-frame noise. The model

    sigma^2 = sigma_base^2 * (sec^2 tilt)^p * (100 / L)^q       (L = blur, BLUR_FOR_MODEL)

is fitted to those residuals by maximum likelihood (2-D isotropic Gaussian, exactly the
form the flight script uses), with confidence intervals from a block bootstrap
(2 s blocks: 30 Hz frames are correlated, blocks are the honest sample unit).

    python3 fit_vqa_model_handheld.py ~/vqa_rig_rgb/bench_adaptive_RIG_HAND_01
    python3 fit_vqa_model_handheld.py ~/vqa_rig_rgb/bench_adaptive_RIG_HAND_01 \\
            --static ~/vqa_rig_rgb/bench_adaptive_RIG_STATIC_01

Optional --static: a session with the drone resting still (level). Its noise is
compared with the hand-held level frames, to show hand tremor is not inflating sigma.

What it measures, and what not: frame-to-frame random noise (what the EKF's R describes).
A slowly varying bias (e.g. from a tilt-dependent calibration error) is removed by the
smoother together with the hand motion and is NOT in this model - state that in the paper.

Needs: numpy, scipy (same as fit_vqa_model.py). Runs on the laptop or the Pi.
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

try:
    from scipy.optimize import minimize
except ImportError:
    sys.exit("scipy is required: pip install scipy")

FPS = 30.0
SIGMA_FLOOR_M = 0.01           # EKF3 / VISO_POS_M_NSE floor used by the flight script
BLUR_COL = {"full_clahe": "blur_full", "roi_raw": "blur_roi"}
EXCLUDE_PHASES = {"TAKEOFF", "DESCENT", "LANDING", "TOUCHDOWN", "LAND", "WAIT_PILOT"}


# ─────────────────────────────── loading ───────────────────────────────
def csv_path(p):
    p = os.path.expanduser(p)
    return p if p.endswith(".csv") else os.path.join(p, "vision_frames.csv")


def load(path, blur_col):
    """Detected frames: t, x, y, height, tilt, blur. Flight sorties: only steady phases."""
    out = []
    with open(csv_path(path), newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r.get("detected") != "1" or r.get("phase", "") in EXCLUDE_PHASES:
                continue
            try:
                out.append((float(r["t_capture"]), float(r["x_raw"]), float(r["y_raw"]),
                            -float(r["z_raw"]), float(r["tilt_deg"]), float(r[blur_col])))
            except (KeyError, ValueError):
                continue
    a = np.array(out, float).reshape(-1, 6)
    return a[np.argsort(a[:, 0])] if len(a) else a


# ─────────────────────────────── detrending ───────────────────────────────
def sg_kernel(n, order=2):
    """Centre-point Savitzky-Golay weights (window n, polynomial order)."""
    k = np.arange(n) - n // 2
    A = np.vander(k, order + 1, increasing=True)
    return np.linalg.pinv(A)[0]


def residuals(a, window_s, max_gap_s=0.1, max_jump_m=0.05):
    """Per-frame residual (x, y) about a local quadratic fit, plus the white-noise
    correction so that E[e2 / c] = 2 sigma^2 for isotropic noise of std sigma."""
    n = int(round(window_s * FPS)) | 1                     # odd
    n = max(n, 5)
    w = sg_kernel(n)
    c = (1.0 - w[n // 2]) ** 2 + np.sum(np.delete(w, n // 2) ** 2)
    t = a[:, 0]
    # segments of continuous detection
    cut = np.where((np.diff(t) > max_gap_s) |
                   (np.hypot(np.diff(a[:, 1]), np.diff(a[:, 2])) > max_jump_m))[0] + 1
    keep, e2, blk = [], [], []
    for seg in np.split(np.arange(len(t)), cut):
        if len(seg) < n:
            continue
        xs, ys = a[seg, 1], a[seg, 2]
        sx = np.convolve(xs, w[::-1], mode="valid")
        sy = np.convolve(ys, w[::-1], mode="valid")
        idx = seg[n // 2: len(seg) - n // 2]
        rx, ry = xs[n // 2: len(seg) - n // 2] - sx, ys[n // 2: len(seg) - n // 2] - sy
        keep.append(idx)
        e2.append((rx ** 2 + ry ** 2) / c)
    if not keep:
        return np.array([], int), np.array([]), n
    return np.concatenate(keep), np.concatenate(e2), n


# ─────────────────────────────── model ───────────────────────────────
def design(tilt, blur, fix_q):
    c = np.cos(np.radians(np.clip(tilt, 0.0, 80.0)))
    cols = [np.ones_like(c), np.log(1.0 / c ** 2)]
    if fix_q is None:
        cols.append(np.log(100.0 / np.maximum(blur, 1.0)))
    return np.column_stack(cols)


def offset(blur, fix_q):
    return 0.0 if fix_q is None else fix_q * np.log(100.0 / np.maximum(blur, 1.0))


def fit(X, e2, off):
    def nll(b):
        ls2 = X @ b + off
        return np.sum(ls2 + e2 * np.exp(-ls2) / 2.0)

    def grad(b):
        ls2 = X @ b + off
        return X.T @ (1.0 - e2 * np.exp(-ls2) / 2.0)

    b0 = np.zeros(X.shape[1])
    b0[0] = math.log(max(np.mean(e2) / 2.0, 1e-10))
    return minimize(nll, b0, jac=grad, method="BFGS").x


def params(b, fix_q):
    return dict(sigma_base_m=math.sqrt(math.exp(b[0])), p=b[1],
                q=(b[2] if fix_q is None else fix_q))


def predict_sigma(prm, tilt, blur):
    c = np.cos(np.radians(np.clip(tilt, 0.0, 80.0)))
    var = prm["sigma_base_m"] ** 2 * (1.0 / c ** 2) ** prm["p"] * \
        (100.0 / np.maximum(blur, 1.0)) ** prm["q"]
    return np.sqrt(var)


# ─────────────────────────────── main ───────────────────────────────
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+", help="bench session folder(s) or vision_frames.csv")
    ap.add_argument("--static", help="optional resting (level, still) session for the tremor check")
    ap.add_argument("--blur", choices=list(BLUR_COL), default="full_clahe",
                    help="must equal BLUR_FOR_MODEL in the flight script (default full_clahe)")
    ap.add_argument("--window-s", type=float, default=0.4, help="smoother window (s)")
    ap.add_argument("--max-tilt", type=float, default=40.0)
    ap.add_argument("--fix-q", type=float, default=None, help="fix the blur exponent (e.g. 0)")
    ap.add_argument("--boot", type=int, default=1000)
    a = ap.parse_args()
    col = BLUR_COL[a.blur]

    T, E2, B, H, BLK = [], [], [], [], []
    blk0 = 0
    print("=" * 72)
    for s in a.sessions:
        d = load(s, col)
        if len(d) < 100:
            print(f"  {s}: only {len(d)} usable frames - skipped")
            continue
        idx, e2, n = residuals(d, a.window_s)
        ok = d[idx, 4] <= a.max_tilt
        idx, e2 = idx[ok], e2[ok]
        blocks = blk0 + ((d[idx, 0] - d[0, 0]) // 2.0).astype(int)
        blk0 = blocks.max() + 1 if len(blocks) else blk0
        T.append(d[idx, 4]); B.append(d[idx, 5]); H.append(d[idx, 3]); E2.append(e2); BLK.append(blocks)
        print(f"  {os.path.basename(os.path.dirname(csv_path(s)))}: {len(d)} detected frames, "
              f"{len(e2)} used (smoother {n} frames), tilt 0-{d[idx, 4].max():.1f} deg, "
              f"height {np.median(d[idx, 3]):.2f} m")
    if not E2:
        sys.exit("No usable session.")
    T, B, H, E2, BLK = map(np.concatenate, (T, B, H, E2, BLK))

    X, off = design(T, B, a.fix_q), offset(B, a.fix_q)
    beta = fit(X, E2, off)
    prm = params(beta, a.fix_q)

    # block bootstrap
    rng = np.random.default_rng(0)
    ub = np.unique(BLK)
    groups = [np.where(BLK == u)[0] for u in ub]
    boots = []
    for _ in range(a.boot):
        pick = np.concatenate([groups[i] for i in rng.integers(0, len(groups), len(groups))])
        boots.append(params(fit(X[pick], E2[pick], off[pick] if np.ndim(off) else off), a.fix_q))
    ci = {k: np.percentile([bb[k] for bb in boots], [2.5, 97.5]) for k in ("sigma_base_m", "p", "q")}

    print("-" * 72)
    print(f"  Frames used {len(E2)}  |  blocks {len(ub)}  |  median height {np.median(H):.2f} m"
          f"  |  blur ({col}) {np.percentile(B, 5):.0f}-{np.percentile(B, 95):.0f}")
    print(f"  sigma_base = {prm['sigma_base_m'] * 1000:.2f} mm   95% CI "
          f"[{ci['sigma_base_m'][0] * 1000:.2f}, {ci['sigma_base_m'][1] * 1000:.2f}]")
    print(f"  p (tilt)   = {prm['p']:.2f}       95% CI [{ci['p'][0]:.2f}, {ci['p'][1]:.2f}]")
    if a.fix_q is None:
        print(f"  q (blur)   = {prm['q']:.2f}       95% CI [{ci['q'][0]:.2f}, {ci['q'][1]:.2f}]")
    else:
        print(f"  q (blur)   = {prm['q']:.2f}       (fixed)")

    # calibration by tilt band
    print("-" * 72)
    print("  tilt band   frames   observed sigma   predicted sigma   obs/pred")
    sig_obs_all = []
    for lo, hi in [(0, 5), (5, 10), (10, 15), (15, 20), (20, 25), (25, 30), (30, 40)]:
        m = (T >= lo) & (T < hi)
        if m.sum() < 30:
            continue
        so = math.sqrt(np.mean(E2[m]) / 2.0)
        sp = float(np.sqrt(np.mean(predict_sigma(prm, T[m], B[m]) ** 2)))
        sig_obs_all.append(so / sp)
        print(f"  {lo:2d}-{hi:2d} deg   {m.sum():6d}   {so * 1000:9.2f} mm     {sp * 1000:9.2f} mm      {so / sp:5.2f}")

    # floor check: the EKF never uses sigma below 1 cm
    s_pred = predict_sigma(prm, T, B)
    pinned = float(np.mean(s_pred < SIGMA_FLOOR_M))
    s_lvl = float(np.median(s_pred[T < 5])) if np.any(T < 5) else float("nan")

    # tremor check with a resting session
    tremor = None
    if a.static:
        d = load(a.static, col)
        if len(d) >= 100:
            idx, e2, _ = residuals(d, a.window_s)
            s_static = math.sqrt(np.mean(e2) / 2.0)
            lvl = T < 3
            s_hand = math.sqrt(np.mean(E2[lvl]) / 2.0) if lvl.sum() > 30 else float("nan")
            tremor = s_hand / s_static
            print("-" * 72)
            print(f"  Tremor check: resting {s_static * 1000:.2f} mm vs hand-held level {s_hand * 1000:.2f} mm"
                  f" -> ratio {tremor:.2f}")

    # verdicts
    print("=" * 72)
    print("  VERDICT")
    gate_a = ci["p"][0] > 0
    print(f"  - Gate A (tilt raises the noise): {'PASS' if gate_a else 'FAIL'} "
          f"(p 95% CI {'excludes' if gate_a else 'includes'} 0)")
    if not gate_a:
        print("    -> tilt does not predict the camera noise. STOP: do not start the campaign;")
        print("       the premise of Arm B needs rethinking first.")
    if sig_obs_all:
        cal = (min(sig_obs_all), max(sig_obs_all))
        good = 0.8 <= cal[0] and cal[1] <= 1.25
        print(f"  - Calibration across tilt bands: obs/pred {cal[0]:.2f}-{cal[1]:.2f} "
              f"-> {'GOOD (0.8-1.25)' if good else 'CHECK (outside 0.8-1.25)'}")
    if a.fix_q is None and ci["q"][0] <= 0 <= ci["q"][1]:
        print("  - Blur exponent q is not distinguishable from 0: consider --fix-q 0 (report it).")
    print(f"  - Predicted sigma when level: {s_lvl * 100:.2f} cm; frames below the EKF floor "
          f"(1 cm): {pinned * 100:.0f} %")
    if pinned >= 0.5:
        print("    -> WARNING: most frames would be clamped to 1 cm by EKF3, so Arm B could not")
        print("       vary and its sigma check fails (pinned >= 50 %). Discuss before pasting.")
    if tremor is not None and tremor > 1.3:
        print("  - Hand tremor inflates the level noise by more than 30 %: repeat more steadily,")
        print("    or rest the elbows / use a support for the low-tilt part.")
    if not (1.6 <= np.median(H) <= 2.3):
        print(f"  - Median height {np.median(H):.2f} m is far from the 2 m flight height: sigma_base "
              "applies at the recorded height.")
    print("-" * 72)
    print("  Paste into the flight script (replace the VQA = dict(...) line only):")
    print(f"  VQA = dict(sigma_base_m={prm['sigma_base_m']:.4f}, tilt_exp_p={prm['p']:.2f}, "
          f"blur_exp_q={prm['q']:.2f},")
    print("             var_min_m2=1e-4, var_max_m2=0.25)")
    print(f"  BLUR_FOR_MODEL = \"{a.blur}\"   (must stay as it is in the script)")
    print("=" * 72)


if __name__ == "__main__":
    main()
