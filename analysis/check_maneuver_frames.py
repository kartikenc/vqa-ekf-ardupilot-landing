#!/usr/bin/env python3
"""
check_maneuver_frames.py - why do some sorties have < 90 % accepted camera frames in MANEUVER?

For every sortie it splits the MANEUVER frames into: detected / not detected, tilt-gated,
outlier-rejected, accepted. It also prints the conditions (brightness, exposure, marker size,
blur, tilt, clock time) so a cause such as lighting, glare or the marker leaving the image shows up.

    py -3.13 check_maneuver_frames.py --logs vqa_paper_E1 --experiment E1
Writes maneuver_frames_check.csv next to where you run it.
"""
import argparse
import datetime as dt
import glob
import json
import os

import numpy as np
import pandas as pd

NUM = ["detected", "accepted", "tilt_gated", "outlier_rejected", "tilt_deg", "brightness",
       "exposure_us", "marker_px", "blur_full", "t_capture", "marker_cx", "marker_cy"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--logs", required=True)
    ap.add_argument("--experiment", default="E1")
    ap.add_argument("--threshold", type=float, default=0.90)
    a = ap.parse_args()

    rows = []
    for js in glob.glob(os.path.join(os.path.expanduser(a.logs), "**", "sortie_summary.json"), recursive=True):
        s = json.load(open(js, encoding="utf-8"))
        if s.get("bench") or ((s.get("args") or {}).get("experiment") or "E1") != a.experiment:
            continue
        csv = os.path.join(os.path.dirname(js), "vision_frames.csv")
        if not os.path.exists(csv):
            continue
        df = pd.read_csv(csv, low_memory=False)
        for c in NUM:
            if c in df:
                df[c] = pd.to_numeric(df[c], errors="coerce")
        mv = df[df.phase == "MANEUVER"]
        if not len(mv):
            continue
        det = mv[mv.detected == 1]
        t0 = mv.t_capture.dropna()
        rows.append(dict(
            sortie_id=s.get("sortie_id"), mode=s.get("mode"),
            start_local=dt.datetime.fromtimestamp(t0.iloc[0]).strftime("%Y-%m-%d %H:%M") if len(t0) else "",
            frames=len(mv),
            fps=len(mv) / max(t0.iloc[-1] - t0.iloc[0], 1e-6) if len(t0) > 1 else np.nan,
            detected=mv.detected.mean(),
            tilt_gated=mv.tilt_gated.mean() if "tilt_gated" in mv else np.nan,
            outlier=mv.outlier_rejected.mean() if "outlier_rejected" in mv else np.nan,
            accepted=mv.accepted.mean(),
            longest_gap_s=(det.t_capture.diff().max() if len(det) > 1 else np.nan),
            tilt_max=mv.tilt_deg.max(),
            tilt_at_miss=mv.loc[mv.detected != 1, "tilt_deg"].median(),
            bright=mv.brightness.median() if "brightness" in mv else np.nan,
            expo_us=mv.exposure_us.median() if "exposure_us" in mv else np.nan,
            marker_px=det.marker_px.median() if "marker_px" in det else np.nan,
            blur=det.blur_full.median() if "blur_full" in det else np.nan,
            # last detected marker centre before a miss: near the image edge -> marker left the view
            edge_before_miss=_edge_before_miss(mv),
        ))
    if not rows:
        raise SystemExit("No sorties found.")
    r = pd.DataFrame(rows).sort_values("sortie_id").reset_index(drop=True)
    r["flag"] = np.where(r.accepted < a.threshold, "<90%", "")
    pd.set_option("display.width", 250, "display.max_columns", 30, "display.max_rows", 200)
    fmt = r.copy()
    for c in ["detected", "tilt_gated", "outlier", "accepted"]:
        fmt[c] = (fmt[c] * 100).round(1)
    for c in ["fps", "longest_gap_s", "tilt_max", "tilt_at_miss", "bright", "marker_px", "blur", "edge_before_miss"]:
        fmt[c] = fmt[c].round(2)
    fmt["expo_us"] = fmt["expo_us"].round(0)
    print("MANEUVER frames per sortie (detected/tilt_gated/outlier/accepted in % of frames)\n")
    print(fmt.to_string(index=False))

    print("\nFlagged vs not flagged (medians):")
    g = r.assign(group=np.where(r.flag == "", "ok", "flagged")).groupby("group")
    print(g[["detected", "tilt_gated", "outlier", "accepted", "fps", "tilt_max", "tilt_at_miss",
             "bright", "expo_us", "marker_px", "blur", "edge_before_miss"]].median().round(3).to_string())
    print("\nFlagged sorties per arm:", r[r.flag != ""].groupby("mode").size().to_dict())
    out = "maneuver_frames_check.csv"
    r.to_csv(out, index=False)
    print(f"\nWrote {os.path.abspath(out)}")


def _edge_before_miss(mv):
    """Median normalised distance (0 = image centre, 1 = edge) of the marker in the frame
    just before each detection loss. Needs marker_cx/cy in pixels and an image width guess."""
    if "marker_cx" not in mv or mv.marker_cx.isna().all():
        return np.nan
    w = np.nanmax(mv.marker_cx) * 1.05
    h = np.nanmax(mv.marker_cy) * 1.05
    d = mv.detected.fillna(0).to_numpy()
    cx, cy = mv.marker_cx.to_numpy(), mv.marker_cy.to_numpy()
    vals = []
    for i in range(1, len(d)):
        if d[i - 1] == 1 and d[i] != 1 and np.isfinite(cx[i - 1]) and w > 0 and h > 0:
            vals.append(max(abs(cx[i - 1] - w / 2) / (w / 2), abs(cy[i - 1] - h / 2) / (h / 2)))
    return float(np.median(vals)) if vals else np.nan


if __name__ == "__main__":
    main()
