#!/usr/bin/env python3
"""
vqa_ekf_ab_test_v7.3.py — VQA-EKF controlled flight experiment (Paper 1, IEEE RA-L)
================================================================================
Script version 7.3: v7.2 (= v7.1 + RGB-fitted VQA model) + Experiment-3 (no optical flow)
safety gate on the EKF position status. Flight profile, landing, sigma per arm: unchanged.
ArduCopter 4.6.3 (EKF3) + Raspberry Pi 5 + Intel RealSense D455 (RGB, downward)
ArUco ID 0, 16 cm, DICT_4X4_1000, on the landing pad.

Camera: --camera rgb (default) uses the D455 colour camera. Its IR-cut filter hides the
MTF-01's infrared ToF light that flooded the IR image in v5. --camera ir = the v1-v5 IR1
path. Use ONE camera for a whole campaign (it is stored in every summary and ledger row).
    --cam-check      No flight. 100 frames: marker detection rate + a snapshot PNG.

ONE script, ONE trajectory, ONE switch: the position-measurement sigma that is
handed to EKF3 with every accepted vision frame.

    --mode constant   Arm A   baseline: sigma = 0.20 m  (ArduPilot's default trust)
    --mode tuned      Arm A'  control : sigma = constant tuned value (--sigma-tuned)
    --mode adaptive   Arm B   VQA-EKF : sigma = f(tilt, blur) recomputed every frame

    --bench           No flight. Prints pose/yaw/sigma live so you can verify axes
                      and signs by moving the drone by hand.  ALWAYS do this first.

What changed from v1 (and why) is listed in CHANGES at the bottom of this file.

Flight sequence (identical for every arm), about 2 minutes from GUIDED to touchdown:
    WAIT_PILOT -> TAKEOFF to 2.0 m (<= 8 s) -> SETTLE (<= 7 s) -> HOVER_BASE (30 s)
    -> MANEUVER (11 steps x 4 s = 44 s: 0 -> +A -> -A -> ... -> 0 along the drone's own axis)
    -> CONVERGE (15 s at origin) -> DESCENT (1 s: pad height measured)
    -> LANDING (v7: EKF-only. Straight down at a constant 0.20 m/s to the EKF origin (0, 0);
       no marker corrections, no pauses - every arm lands the same way, so the touchdown
       error measures the EKF of that arm. The last few cm are flown blind, as before.)
    -> forced disarm on touchdown
    -> vision keeps feeding EKF3 until DISARM -> summary + ledger saved
    -> operator enters the pad-grid touchdown measurement (independent ground truth)

Safety behaviour:
    * v6.1a: GUIDED may be selected at any height, also on the ground: if ArduCopter reports
      ON_GROUND the script takes off itself (2 s countdown, cancel by leaving GUIDED).
    * v6.1a: once the rangefinder shows the drone standing on the pad (<= 0.25 m, still for
      0.3 s) while the script is landing it, the script force-disarms at once (no skid).
      Never after a pilot takeover. Disable with --no-auto-disarm.
    * v6.1b: needs GUID_OPTIONS bit 2 (+4, IgnorePilotYaw) so an off-centre yaw stick cannot
      turn the drone in GUIDED. The landing is finished in GUIDED (heading held); ArduPilot
      LAND is only the fallback, and LAND still obeys the yaw stick - keep it centred.
    * v7: the landing never reacts to the marker. A marker lost during the landing is only
      logged (MARKER_LOST_IN_LANDING); the descent continues on the EKF.
    * Pilot leaving GUIDED = script stops commanding; vision keeps running until disarm.
    * Ctrl-C once = LAND (vision keeps running). Ctrl-C again = force-stop vision.

Outputs (one folder per sortie):  ~/vqa_logs/<mode>_<sortie_id>/
    vision_frames.csv   every camera frame (inputs, gating, sigma, poses, latency)
    sortie_summary.json events, parameters read from the FC, per-phase stats, truth
    console.log         copy of the console
  + copy the DataFlash .bin of the flight into the same folder afterwards
    (analysis reads VISP.PErr = sigma EKF3 actually used, and XKF3/XKF4 innovations).

Authors: Team ASCEND / Drone-A-Charyas
"""

import os
os.environ.setdefault("MAVLINK20", "1")          # covariance fields need MAVLink 2

import argparse
import bisect
import collections
import csv
import datetime
import hashlib
import json
import math
import sys
import threading
import time

import numpy as np
import cv2

SCRIPT_VERSION = "v7.3"

# One time base for everything: monotonic (immune to the Pi's clock being stepped by
# NTP) but offset to epoch seconds so logs still read as wall-clock time.
_T0 = time.time() - time.monotonic()


def now():
    return time.monotonic() + _T0


try:                                    # imported lazily-checked so the geometry /
    from pymavlink import mavutil       # covariance functions can be unit-tested
except ImportError:                     # without flight hardware
    mavutil = None
try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


# ═══════════════════════════════════════════════════════════════════════════
# CONFIGURATION (fixed for the whole experiment — do not change between arms)
# ═══════════════════════════════════════════════════════════════════════════
MAVLINK_PORT = "/dev/ttyAMA2"      # Pi 5 UART2: pin 7 (TX) / pin 29 (RX) -> Cube TELEM1
MAVLINK_BAUD = 921600

ARUCO_DICT    = cv2.aruco.DICT_4X4_1000
MARKER_ID     = 0
MARKER_SIZE_M = 0.16

# Camera (v6). "rgb" = D455 colour camera, "ir" = left IR imager IR1 (v1-v5).
CAMERA_DEFAULT = "rgb"
CAM_W, CAM_H, CAM_FPS = 640, 480, 30
SUPPORTED_DISTORTION = ("none", "brown_conrady", "inverse_brown_conrady",
                        "modified_brown_conrady")

# ── MISSION PLAN (v5): about 2 minutes from GUIDED to touchdown ──────────────
#   Phase        Duration                         Ends when
#   TAKEOFF      <= 8 s  (TAKEOFF_TIMEOUT_S)      within 15 cm of 2.0 m  (else abort)
#   SETTLE       <= 7 s  (SETTLE_TIMEOUT_S)       drift < 15 cm          (else abort)
#   HOVER_BASE   30 s    (HOVER_BASE_S)           time served
#   MANEUVER     44 s    (11 steps x 4 s)         time served
#   CONVERGE     15 s    (CONVERGE_S)             time served
#   DESCENT      1 s     (PAD_REF_S)              pad EKF height measured
#   LANDING      >= 10 s (0.20 m/s while centred) touchdown on the marker + forced disarm
#   Total        ~ 2 min.  Keep identical for every sortie of the campaign.
TARGET_ALT_M        = 2.0     # flight altitude (m) — also the --alt default
TAKEOFF_TIMEOUT_S   = 8.0
SETTLE_TIMEOUT_S    = 7.0
HOVER_BASE_S        = 30.0
STEP_HOLD_S         = 4.0     # fixed hold per maneuver step (same timing in every arm)
N_CYCLES            = 5       # +A/-A pairs -> 2*5 + 1 = 11 steps
CONVERGE_S          = 15.0
DESCENT_ALT_M      = 0.40    # fallback pad height only (EKF height of the pad unknown)
# Pad reference (v6.1b). Log 29: EKF read -0.22 m on the pad (logs 15-24: -0.40 m), so a
# fixed EKF target is wrong; the pad's EKF height is measured every sortie.
PAD_REF_S           = 1.0     # rangefinder + EKF averaged this long at hover -> pad EKF z
RNG_PAD_DEFAULT_M   = 0.22    # rangefinder on the pad if not measured (log 29: 0.22 m)
# EKF-only landing (v7). Same profile in every arm; nothing here depends on the marker.
CAM_OFFSET_DEFAULT  = (0.11, 0.0)  # camera ahead/right of the CG (m) if VISO_POS_X/Y not read
                                   # (used only to LOG where the marker last saw the CG)
LAND_VZ_MPS         = 0.20    # constant descent speed of the setpoint (same as v6.1c)
LAND_LEAD_MAX_M     = 0.30    # descent setpoint never more than this below the drone
LAND_BELOW_PAD_M    = 0.20    # setpoint ends this far BELOW the pad, so the drone sits down
LAND_BOTTOM_WAIT_S  = 3.0     # setpoint at the bottom, no touchdown this long -> ArduPilot LAND
LAND_TIMEOUT_S      = 60.0    # whole landing; then ArduPilot LAND
LAND_HANDOVER_M     = 0.20    # no rangefinder / --no-auto-disarm: ArduPilot LAND below this
LAND_LOST_LOG_M     = 0.50    # marker lost above this height during the landing -> logged
LAND_STATUS_S       = 1.0     # console status line period
ALT_CONFIRM_M       = 0.15
SETTLE_DRIFT_M      = 0.15

MARKER_LOST_LAND_S  = 5.0
FOV_MARGIN_DEG      = 3.0
# Take-off (v6.1a): GUIDED may be selected at ANY height, also still on the ground/platform.
# If ArduCopter reports ON_GROUND it ignores position setpoints, so the script then sends
# MAV_CMD_NAV_TAKEOFF; if it already counts as flying (e.g. throttle was raised) it climbs
# with setpoints directly.
TKOFF_COUNTDOWN_S   = 2.0     # pilot can cancel by leaving GUIDED
TKOFF_CMD_ALT_M     = 1.0     # NAV_TAKEOFF height; the setpoint climb then goes to --alt
LIFTOFF_RISE_M      = 0.15    # EKF height rise that counts as "left the ground"
LIFTOFF_TIMEOUT_S   = 8.0
# Touchdown auto-disarm (v6.1a). ArduCopter refuses a normal MAVLink disarm until it has
# detected the landing itself, and it never does while the position controller still asks
# for > 15 deg lean on the ground (log 16) -> skid. The script force-disarms instead, only
# while IT is landing, and only when the rangefinder shows the drone is standing on the pad.
RNG_ON_GROUND_M     = 0.25    # MTF-01 reads 0.19-0.21 m when landed (logs 15/16/18/24)
TOUCHDOWN_VZ_MPS    = 0.15    # and the EKF vertical speed is about zero ...
TOUCHDOWN_CONFIRM_S = 0.3     # ... continuously for this long
RNG_MIN_VALID_M     = 0.10    # v6.1b: MTF-01 drop-outs read 0.00-0.02 m for up to 0.4 s in
                              # flight (log 29); landed it never reads below 0.18 m
PAD_Z_TOL_M         = 0.20    # v6.1b: ... and EKF height must be within this of the pad
FORCE_DISARM_MAGIC  = 21196   # MAV_CMD_COMPONENT_ARM_DISARM param2: disarm even if "flying"
MAV_CMD_NAV_TAKEOFF_ID = 22
MAV_CMD_ARM_DISARM_ID = 400
HB_TIMEOUT_S        = 3.0     # no FC heartbeat this long = link lost (FC sends 1 Hz)
MAX_ATT_AGE_MS      = 200.0   # FC attitude older than this = no tilt correction -> no GUIDED
ATT_LOST_ABORT_S    = 0.5     # v7.1: in flight, abort only if attitude stays stale this long
# v7.3 - Experiment 3 (--no-flow). Without optical flow, vision is the ONLY horizontal aid, so
# the EKF must be in absolute-position aiding before take-off. EKF_STATUS_REPORT flags:
EKF_POS_HORIZ_ABS   = 16      # EKF3: !posTimeout && AID_ABSOLUTE && healthy
EKF_CONST_POS_MODE  = 128     # EKF3: no aiding at all (AID_NONE)
EKF_GATE_S          = 2.0     # --no-flow: absolute position OK continuously this long before take-off
EKF_LOST_ABORT_S    = 1.0     # --no-flow: in flight, abort (LAND) if it is lost this long
E3_WINDOW_LOCAL     = (10.0, 15.5)   # E3 protocol: fly 10:00-15:30 local (low-sun glare, E1)

# Gating (identical in every arm — it is NOT the variable under test)
OUTLIER_JUMP_M      = 0.30
OUTLIER_RESET_COUNT = 10
REACQ_GAP_S         = 0.5     # re-seed without a jump check only after a gap this long
MAX_SPEED_MPS       = 1.5     # jump allowance grows with the time since the last fix

# Sigma sent to EKF3 (metres, 1-sigma). EKF3 itself floors posErr at 0.01 m and
# VISO_POS_M_NSE must be <= SIGMA_FLOOR_M or values get clamped (the v1 bug).
SIGMA_FLOOR_M    = 0.01
SIGMA_CEIL_M     = 0.50
SIGMA_DEFAULT_M  = 0.20       # Arm A: what ArduPilot uses when no covariance is given
SIGMA_YAW_RAD    = 0.10       # same in every arm (yaw is not the variable)

# VQA error model (fitted offline; see paper Table 4).  var_xy = sb^2 (sec^2 t)^p (100/L)^q
VQA = dict(sigma_base_m=0.0226, tilt_exp_p=5.25, blur_exp_q=0.69,     # v7.2: RGB fit (gate G1)
           var_min_m2=1e-4, var_max_m2=0.25)
# Which blur definition the model was FITTED with. Must match the fitting script.
#   "full_clahe" : Laplacian variance of the full CLAHE-enhanced grey frame (v1 behaviour)
#   "roi_raw"    : Laplacian variance of the raw grey marker ROI
# v7.2: constants refitted on RGB data with fit_vqa_model_handheld.py (gate G1); frozen
# for E1 and E3 (sigma_base set so the sharpest level frames sit at the 1 cm EKF3 floor).
BLUR_FOR_MODEL = "full_clahe"

# GPS-less EKF origin (any fixed value; only needed so EKF3 initialises)
ORIGIN_LAT, ORIGIN_LON, ORIGIN_ALT = int(12.9716e7), int(77.5946e7), 920000

# Parameters the experiment depends on. The script refuses to fly if any check fails.
# (name, test, what it must be).  `flow` = optical flow in use (False with --no-flow).
def required_param_checks(flow):
    eq = lambda want: (lambda v: v is not None and abs(v - want) < 1e-6)
    return [
        ("AHRS_EKF_TYPE",   eq(3), "3 (EKF3)"),
        ("EK3_SRC1_POSXY",  eq(6), "6 (ExternalNav)"),
        ("EK3_SRC1_VELXY",  eq(5 if flow else 0), "5 (OpticalFlow)" if flow else "0 (none, --no-flow)"),
        ("EK3_SRC1_POSZ",   eq(2), "2 (RangeFinder)"),
        ("EK3_SRC1_VELZ",   eq(0), "0 (none)"),
        ("EK3_SRC1_YAW",    eq(6), "6 (ExternalNav)"),
        ("EK3_SRC_OPTIONS", eq(0), "0"),
        ("VISO_TYPE",       eq(1), "1 (MAVLink)"),
        ("VISO_SCALE",      eq(1.0), "1.0 (any other value scales every position)"),
        ("VISO_QUAL_MIN",   lambda v: v is not None and v <= 0,
                            "<= 0 (higher makes ArduPilot ignore every vision message)"),
        ("VISO_POS_M_NSE",  lambda v: v is not None and v <= SIGMA_FLOOR_M + 1e-6,
                            f"<= {SIGMA_FLOOR_M} (higher clamps every sigma — the v1 failure)"),
        ("VISO_YAW_M_NSE",  lambda v: v is not None and v <= SIGMA_YAW_RAD + 1e-6,
                            f"<= {SIGMA_YAW_RAD} (else the yaw sigma we send is clamped)"),
        ("EK3_FLOW_USE",    (eq(1) if flow else (lambda v: True)),
                            "1 (Navigation)" if flow else "any (flow not used)"),
        ("FLOW_TYPE",       ((lambda v: v is not None and v > 0) if flow else (lambda v: True)),
                            "> 0 (flow sensor enabled)" if flow else "any (flow not used)"),
        # v6.1b: log 29 - yaw stick resting 45-52 us right of trim (outside RC4_DZ) turned the
        # drone 11.7 deg/s for the whole sortie, because GUIDED obeys pilot yaw by default.
        ("GUID_OPTIONS",    lambda v: v is not None and (int(v) & 4) != 0,
                            "bit 2 set, e.g. 4 (IgnorePilotYaw: heading held in GUIDED)"),
    ]


LOGGED_PARAMS = ["VISO_DELAY_MS", "VISO_POS_X", "VISO_POS_Y", "VISO_POS_Z", "VISO_ORIENT",
                 "EK3_POS_I_GATE", "WPNAV_ACCEL", "WPNAV_SPEED", "ANGLE_MAX",
                 "RNGFND1_MIN_CM", "RNGFND1_GNDCLEAR"]

MODE_GUIDED, MODE_LAND = 4, 9
ARM_LABEL = {"constant": "Arm A (constant 0.20 m)", "tuned": "Arm A' (tuned constant)",
             "adaptive": "Arm B (VQA-EKF adaptive)"}
ARM_SHORT = {"constant": "A", "tuned": "A'", "adaptive": "B"}


def mission_plan(cycles=N_CYCLES):
    """Phase budget actually used by the flight thread (for the console and the log)."""
    steps = 2 * cycles + 1
    plan = [("TAKEOFF (max)", TAKEOFF_TIMEOUT_S), ("SETTLE (max)", SETTLE_TIMEOUT_S),
            ("HOVER_BASE", HOVER_BASE_S), (f"MANEUVER ({steps} x {STEP_HOLD_S:g} s)", steps * STEP_HOLD_S),
            ("CONVERGE", CONVERGE_S), ("DESCENT (pad ref)", PAD_REF_S),
            ("LANDING (EKF only, min)", TARGET_ALT_M / LAND_VZ_MPS)]
    return plan, sum(d for _, d in plan)


# ═══════════════════════════════════════════════════════════════════════════
# COVARIANCE: model, per-arm sigma, and encoding for ArduPilot
# ═══════════════════════════════════════════════════════════════════════════
def model_sigma(tilt_deg, blur):
    """VQA error model -> 1-sigma lateral position error (m)."""
    c = math.cos(math.radians(min(max(tilt_deg, 0.0), 80.0)))
    sec2 = 1.0 / (c * c)
    var = (VQA["sigma_base_m"] ** 2) * sec2 ** VQA["tilt_exp_p"] \
        * (100.0 / max(blur, 1.0)) ** VQA["blur_exp_q"]
    var = min(max(var, VQA["var_min_m2"]), VQA["var_max_m2"])
    return math.sqrt(var)


def arm_sigma(mode, sigma_model, sigma_tuned):
    if mode == "constant":
        s = SIGMA_DEFAULT_M
    elif mode == "tuned":
        s = sigma_tuned
    else:
        s = sigma_model
    return min(max(s, SIGMA_FLOOR_M), SIGMA_CEIL_M)


def ap_encode(sigma_pos_m, sigma_yaw_rad=SIGMA_YAW_RAD):
    """21-element VISION_POSITION_ESTIMATE covariance that makes ArduPilot use
    exactly sigma_pos_m.  ArduPilot (GCS_Common.cpp) computes
        posErr = cbrt(c[0]^2 + c[6]^2 + c[11]^2)
    so with c[0]=c[6]=c[11]=c we need c = sqrt(sigma^3 / 3)."""
    c = math.sqrt(sigma_pos_m ** 3 / 3.0)
    a = math.sqrt(sigma_yaw_rad ** 3 / 3.0)
    cov = [0.0] * 21
    cov[0] = cov[6] = cov[11] = c
    cov[15] = cov[18] = cov[20] = a
    return cov


def ap_decode(cov):
    """Mirror of ArduPilot's decoding (used for the start-up self-test)."""
    pos = (cov[0] ** 2 + cov[6] ** 2 + cov[11] ** 2) ** (1.0 / 3.0)
    ang = (cov[15] ** 2 + cov[18] ** 2 + cov[20] ** 2) ** (1.0 / 3.0)
    return pos, ang


# ═══════════════════════════════════════════════════════════════════════════
# GEOMETRY
# ═══════════════════════════════════════════════════════════════════════════
#  World frame W (what EKF3 sees): origin = marker centre, z down,
#     x_W = marker +x (printed right edge direction), y_W = marker -y.
#  Body frame: x nose, y right, z down.   Camera: OpenCV (x right, y down, z out).
R_WM = np.diag([1.0, -1.0, -1.0])                  # marker -> world


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def cam_to_body(cam_yaw_deg):
    """Camera->body rotation for a downward camera.
    cam_yaw_deg = direction the IMAGE TOP points, measured from the nose, clockwise
    seen from above (0 = image top toward nose, 90 = toward right side)."""
    b0 = np.array([[0.0, -1.0, 0.0],
                   [1.0, 0.0, 0.0],
                   [0.0, 0.0, 1.0]])
    return rot_z(math.radians(cam_yaw_deg)) @ b0


def marker_obj_points(size=MARKER_SIZE_M):
    h = size / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)


def estimate_pose(corners, cam_mat, dist, roll, pitch, B):
    """Corners of the target marker -> (x, y, z, yaw, info) in world frame W.
    Roll/pitch come from the IMU (time-aligned); yaw comes from the marker.
    Returns None if PnP fails."""
    img = corners.reshape(4, 2).astype(np.float64)
    try:
        ok, rvecs, tvecs, reproj = cv2.solvePnPGeneric(
            marker_obj_points(), img, cam_mat, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    except cv2.error:
        return None
    if not ok or not rvecs:
        return None

    # Resolve the IPPE two-fold ambiguity with the IMU: pick the solution whose
    # marker normal best matches "world up" as seen from the body.
    R_bw = (rot_y(pitch) @ rot_x(roll)).T
    up_b = R_bw @ np.array([0.0, 0.0, -1.0])
    best, best_ang = None, 1e9
    for i in range(len(rvecs)):
        R_cm, _ = cv2.Rodrigues(rvecs[i])
        n_b = B @ R_cm[:, 2]
        ang = math.acos(max(-1.0, min(1.0, float(n_b @ up_b))))
        if ang < best_ang:
            best, best_ang = i, ang
    R_cm, _ = cv2.Rodrigues(rvecs[best])
    t = tvecs[best].reshape(3)
    if not (0.10 < t[2] < 10.0):
        return None

    # Yaw from the marker (well conditioned: in-plane rotation of the marker image)
    R_wb_vis = R_WM @ (B @ R_cm).T
    yaw = math.atan2(R_wb_vis[1, 0], R_wb_vis[0, 0])

    # Position: IMU roll/pitch + marker yaw, origin at marker centre
    R_wb = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)
    p = -R_wb @ (B @ t)
    info = dict(normal_err_deg=math.degrees(best_ang),
                reproj=float(np.ravel(reproj)[best]) if reproj is not None else float("nan"),
                range_m=float(np.linalg.norm(t)))
    return float(p[0]), float(p[1]), float(p[2]), yaw, info


def true_tilt_deg(roll, pitch):
    return math.degrees(math.acos(max(-1.0, min(1.0, math.cos(roll) * math.cos(pitch)))))


def max_step_amplitude(alt_m, half_fov_deg, tilt_deg, margin_deg=FOV_MARGIN_DEG,
                       marker=MARKER_SIZE_M):
    """Largest step A (m) that keeps the whole marker in view at alt with tilt."""
    ang = half_fov_deg - margin_deg - tilt_deg
    if ang <= 0:
        return 0.0
    return max(0.0, alt_m * math.tan(math.radians(ang)) - marker / 2.0)


def laplacian_var(img):
    return float(cv2.Laplacian(img, cv2.CV_64F).var())


# ═══════════════════════════════════════════════════════════════════════════
# MAVLINK LINK: one RX thread, locked TX, time-aligned attitude
# ═══════════════════════════════════════════════════════════════════════════
def is_mavlink2_msg(msg):
    """True if this received message was framed as MAVLink 2 (start byte 0xFD)."""
    try:
        buf = msg.get_msgbuf()
        return bool(buf) and buf[0] == 0xFD
    except Exception:
        return False


def is_autopilot_heartbeat(hb):
    """The flight controller's own heartbeat: component 1 and a real autopilot type.
    Other devices on the link (e.g. the MTF-01 forwarded from TELEM2: component 0,
    autopilot 8 = invalid, MAVLink 1) are not the flight controller."""
    return (hb.get_srcComponent() == 1 and
            hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID)


class Link:
    def __init__(self, port, baud, require_v2=True, hb_timeout_s=30.0):
        if mavutil is None:
            sys.exit("ERROR: pymavlink not installed (pip install pymavlink)")
        print(f"[MAV] Connecting {port} @ {baud} (MAVLink 2)...")
        self.m = mavutil.mavlink_connection(port, baud=baud, source_system=254,
                                            source_component=197)
        # Wait for the FLIGHT CONTROLLER's heartbeat. v6.0 took the first heartbeat of any
        # device, so a forwarded MTF-01 heartbeat made the MAVLink 2 check fail at random.
        hb, others, t_end = None, set(), time.monotonic() + hb_timeout_s
        while time.monotonic() < t_end:
            h = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if h is None:
                continue
            if is_autopilot_heartbeat(h):
                hb = h
                break
            others.add((h.get_srcSystem(), h.get_srcComponent(),
                        "v2" if is_mavlink2_msg(h) else "v1", h.autopilot))
        if others:
            print(f"[MAV] ignored heartbeats from other devices (sys, comp, protocol, autopilot): "
                  f"{sorted(others)}\n      -> they are forwarded to the Pi; set SERIAL2_OPTIONS = 1024 "
                  "(MTF-01 port) and reboot the Cube to stop that.")
        if hb is None:
            sys.exit(f"ERROR: no heartbeat from the flight controller in {hb_timeout_s:.0f} s.\n"
                     "       Is the Cube powered and fully booted? Check the TELEM1 wiring "
                     "(Pi pin 7 -> TELEM1 pin 3, pin 29 -> pin 2, GND) and MAVLINK_PORT.")
        self.m.target_system = hb.get_srcSystem()
        # Covariance is a MAVLink-2-only field. On MAVLink 1 it never arrives, ArduPilot
        # reads it as 0 and clamps every arm to VISO_POS_M_NSE — silently. Refuse.
        ours_v2 = getattr(self.m, "WIRE_PROTOCOL_VERSION", "") == "2.0"
        fc_v2 = is_mavlink2_msg(hb)
        print(f"[MAV] Protocol: script {'MAVLink 2' if ours_v2 else 'MAVLink 1'}, "
              f"flight controller {'MAVLink 2' if fc_v2 else 'MAVLink 1'}")
        if require_v2 and not (ours_v2 and fc_v2):
            sys.exit("ERROR: MAVLink 2 required on both sides (covariance would be lost).\n"
                     "       Set SERIALx_PROTOCOL = 2 on the Cube port wired to the Pi, and do\n"
                     "       not set MAVLINK20/MAVLINK09 in the environment.")
        if self.m.target_system == 0:
            self.m.target_system = 1
        self.m.target_component = 1
        print(f"[MAV] Connected sys={self.m.target_system}")
        self.tx = threading.Lock()
        self.lock = threading.Lock()
        self.att = collections.deque(maxlen=400)       # (fc_t, roll, pitch, yaw)
        self.off_samples = collections.deque(maxlen=600)   # (host_t, host-fc)
        self.clock_off = None
        self.ekf = (0.0,) * 6
        self.ekf_host_t = 0.0
        self.armed = False
        self.custom_mode = -1
        self.voltage = float("nan")
        self.hb_t = now()
        self.rx_errors = 0
        self.statustext = collections.deque(maxlen=50)
        self.events_cb = None
        self.acks = {}                                 # command id -> (host time, result)
        self.rng = (float("nan"), 0.0)                 # downward rangefinder (m, host time)
        self.landed = (None, 0.0)                      # MAV_LANDED_STATE (1 ground, 2 air, ...)
        self.auto_disarm = False                       # set by the flight thread while landing
        self.z_pad = None                              # v6.1b: EKF z of the pad (TouchdownWatch)
        self.ekf_flags = (None, 0.0)                   # v7.3: EKF_STATUS_REPORT flags, host time

    # ---- parameters (call before the RX thread starts) ----
    def read_param(self, name, retries=3):
        for _ in range(retries):
            with self.tx:
                self.m.mav.param_request_read_send(self.m.target_system,
                                                   self.m.target_component,
                                                   name.encode(), -1)
            t_end = now() + 1.5
            while now() < t_end:
                msg = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
                if msg is not None and msg.param_id.strip("\x00") == name:
                    return float(msg.param_value)
        return None

    def set_message_interval(self, msg_id, hz):
        with self.tx:
            self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                         mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                         msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

    def send_origin(self):
        for _ in range(3):
            with self.tx:
                self.m.mav.set_gps_global_origin_send(self.m.target_system, ORIGIN_LAT,
                                                      ORIGIN_LON, ORIGIN_ALT, 0)
            time.sleep(0.1)
        with self.tx:
            self.m.mav.set_home_position_send(self.m.target_system, ORIGIN_LAT, ORIGIN_LON,
                                              ORIGIN_ALT, 0, 0, 0, [1, 0, 0, 0], 0, 0, 1)

    # ---- RX thread ----
    def rx_loop(self, stop):
        """Only reader of MAVLink. Never dies: any error is counted and the loop goes on."""
        while not stop.is_set():
            try:
                msg = self.m.recv_match(blocking=True, timeout=0.05)
                if msg is None:
                    continue
                self.handle(msg)
            except Exception as e:                       # noqa: BLE001
                self.rx_errors += 1
                if self.rx_errors <= 5:
                    print(f"[MAV] RX error #{self.rx_errors}: {type(e).__name__}: {e}")
                time.sleep(0.01)

    def handle(self, msg):
        t_host = now()
        typ = msg.get_type()
        if typ == "ATTITUDE":
            fc_t = msg.time_boot_ms / 1000.0
            with self.lock:
                if self.att and fc_t < self.att[-1][0]:      # FC reboot
                    self.att.clear()
                    self.off_samples.clear()
                self.att.append((fc_t, msg.roll, msg.pitch, msg.yaw))
                # host-minus-FC offset, lower envelope over ~10 s
                self.off_samples.append((t_host, t_host - fc_t))
                while self.off_samples and t_host - self.off_samples[0][0] > 10.0:
                    self.off_samples.popleft()
                self.clock_off = min(o for _, o in self.off_samples)
        elif typ == "LOCAL_POSITION_NED":
            with self.lock:
                self.ekf = (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)
                self.ekf_host_t = t_host
        elif typ == "HEARTBEAT":
            if msg.get_srcComponent() == 1 and \
                    msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                with self.lock:
                    self.armed = bool(msg.base_mode &
                                      mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.custom_mode = msg.custom_mode
                    self.hb_t = t_host
        elif typ == "SYS_STATUS":
            with self.lock:
                self.voltage = msg.voltage_battery / 1000.0
        elif typ == "STATUSTEXT":
            txt = msg.text.strip("\x00") if isinstance(msg.text, str) else str(msg.text)
            print(f"[FC] {txt}")
            with self.lock:
                self.statustext.append((t_host, txt))
            if self.events_cb:
                self.events_cb("FC_STATUSTEXT", text=txt)
        elif typ == "DISTANCE_SENSOR":
            # v7.1: only the flight controller's own report (component 1). The MTF-01's own
            # DISTANCE_SENSOR, forwarded from TELEM2, must not mix in (touchdown + landing height).
            if (msg.get_srcComponent() == 1 and msg.get_srcSystem() == self.m.target_system
                    and msg.orientation == 25 and msg.current_distance > 0):   # downward-facing
                with self.lock:
                    self.rng = (msg.current_distance / 100.0, t_host)
        elif typ == "RANGEFINDER":                                   # ArduPilot backup message
            if msg.get_srcComponent() != 1 or msg.get_srcSystem() != self.m.target_system:
                return
            with self.lock:
                if now() - self.rng[1] > 0.5:
                    self.rng = (float(msg.distance), t_host)
        elif typ == "EKF_STATUS_REPORT":                             # v7.3
            flags = int(msg.flags)
            with self.lock:
                prev = self.ekf_flags[0]
                self.ekf_flags = (flags, t_host)
            if prev != flags and self.events_cb:
                self.events_cb("EKF_FLAGS", flags=flags,
                               pos_horiz_abs=bool(flags & EKF_POS_HORIZ_ABS),
                               const_pos_mode=bool(flags & EKF_CONST_POS_MODE))
        elif typ == "EXTENDED_SYS_STATE":
            with self.lock:
                self.landed = (int(msg.landed_state), t_host)
        elif typ == "COMMAND_ACK":
            with self.lock:
                self.acks[int(msg.command)] = (t_host, int(msg.result))

    # ---- accessors ----
    def attitude_at(self, t_host):
        """Roll, pitch interpolated to host time t_host. Returns (roll, pitch, age_ms).
        Takes ONE snapshot under the lock and works only on that snapshot, so the RX
        thread can never change the data mid-computation (the v2 race)."""
        with self.lock:
            if not self.att or self.clock_off is None:
                return 0.0, 0.0, float("inf")
            fc_t = t_host - self.clock_off
            att = list(self.att)
        ts = [a[0] for a in att]
        i = bisect.bisect_left(ts, fc_t)
        if i <= 0:
            _, r, p, _ = att[0]
            return r, p, abs(ts[0] - fc_t) * 1000.0
        if i >= len(ts):
            _, r, p, _ = att[-1]
            return r, p, abs(fc_t - ts[-1]) * 1000.0
        t0, r0, p0, _ = att[i - 1]
        t1, r1, p1, _ = att[i]
        w = (fc_t - t0) / max(t1 - t0, 1e-6)
        return r0 + w * (r1 - r0), p0 + w * (p1 - p0), 0.0

    def get_ekf(self):
        with self.lock:
            return self.ekf, (now() - self.ekf_host_t) * 1000.0

    def get_status(self):
        with self.lock:
            return self.armed, self.custom_mode, self.voltage

    def get_range(self):
        """Downward rangefinder: (metres, age_ms)."""
        with self.lock:
            return self.rng[0], (now() - self.rng[1]) * 1000.0

    def ekf_status(self):
        """v7.3: (flags or None, age_ms, absolute_position_ok)."""
        with self.lock:
            flags, t = self.ekf_flags
        age = (now() - t) * 1000.0 if flags is not None else float("inf")
        ok = (flags is not None and age < 1000.0 and bool(flags & EKF_POS_HORIZ_ABS)
              and not flags & EKF_CONST_POS_MODE)
        return flags, age, ok

    def landed_state(self):
        """ArduCopter's own landed state: (MAV_LANDED_STATE or None, age_ms)."""
        with self.lock:
            return self.landed[0], (now() - self.landed[1]) * 1000.0

    def ack_after(self, command, t_sent):
        """Result of the first COMMAND_ACK for `command` received after t_sent, else None."""
        with self.lock:
            a = self.acks.get(int(command))
        return a[1] if a and a[0] >= t_sent else None

    def heartbeat_age(self):
        with self.lock:
            return now() - self.hb_t

    # ---- TX ----
    def send_vision(self, t_capture, x, y, z, roll, pitch, yaw, cov):
        with self.tx:
            self.m.mav.vision_position_estimate_send(int(t_capture * 1e6), x, y, z,
                                                     roll, pitch, yaw, cov, 0)

    def send_setpoint(self, x, y, z, yaw):
        mask = 0b0000_1001_1111_1000          # use position + yaw; ignore vel/acc/yaw-rate
        with self.tx:
            self.m.mav.set_position_target_local_ned_send(
                0, self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
                x, y, z, 0, 0, 0, 0, 0, 0, yaw, 0)

    def set_mode(self, custom_mode):
        with self.tx:
            self.m.mav.set_mode_send(self.m.target_system,
                                     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                     custom_mode)

    def send_takeoff(self, alt_m):
        """GUIDED take-off from the ground (position setpoints are ignored while landed)."""
        with self.tx:
            self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                         MAV_CMD_NAV_TAKEOFF_ID, 0,
                                         0, 0, 0, 0, 0, 0, float(alt_m))

    def send_force_disarm(self):
        """Disarm even though ArduCopter has not (yet) detected the landing."""
        with self.tx:
            self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                         MAV_CMD_ARM_DISARM_ID, 0,
                                         0, FORCE_DISARM_MAGIC, 0, 0, 0, 0, 0)


class TouchdownWatch:
    """True once the drone has stood on the pad for TOUCHDOWN_CONFIRM_S: downward rangefinder
    at landed level (<= RNG_ON_GROUND_M) and EKF vertical speed about zero, continuously.
    v6.1b: rangefinder drop-outs (< RNG_MIN_VALID_M) never count, and once the pad's EKF
    height is known (link.z_pad) the EKF must also be within PAD_Z_TOL_M of it, so a drop-out
    during a hover hold can never disarm the drone in the air."""

    def __init__(self, link):
        self.link, self.t_ok = link, None

    def update(self):
        rng, r_age = self.link.get_range()
        (_, _, ez, _, _, vz), e_age = self.link.get_ekf()
        z_pad = getattr(self.link, "z_pad", None)
        ok = (r_age < 300 and math.isfinite(rng) and RNG_MIN_VALID_M <= rng <= RNG_ON_GROUND_M
              and e_age < 500 and abs(vz) <= TOUCHDOWN_VZ_MPS
              and (z_pad is None or ez >= z_pad - PAD_Z_TOL_M))
        if not ok:
            self.t_ok = None
            return False
        self.t_ok = self.t_ok if self.t_ok is not None else now()
        return now() - self.t_ok >= TOUCHDOWN_CONFIRM_S

    def range_m(self):
        return self.link.get_range()[0]


def force_disarm(link, log, where):
    """Send the forced disarm (repeated until the FC reports disarmed, max ~1.5 s)."""
    rng = link.get_range()[0]
    log.event("TOUCHDOWN_DISARM", phase=where, rangefinder_m=rng)
    print(f"[FLT] TOUCHDOWN detected ({where}, rangefinder {rng:.2f} m) -> disarming now")
    for _ in range(6):
        link.send_force_disarm()
        time.sleep(0.25)
        if not link.get_status()[0]:
            return True
    print("[FLT] WARNING: FC did not confirm the disarm — PILOT: disarm manually")
    return False


def cg_from_camera(cam_x, cam_y, yaw, offset_xy):
    """Marker-frame position of the CG (landing-gear centre) from the camera position the
    vision thread measured: camera = CG + R(yaw) * offset (VISO_POS_X/Y, body frame)."""
    ox, oy = offset_xy
    c, s_ = math.cos(yaw), math.sin(yaw)
    return cam_x - (c * ox - s_ * oy), cam_y - (s_ * ox + c * oy)


# ═══════════════════════════════════════════════════════════════════════════
# SHARED STATE + LOGGING
# ═══════════════════════════════════════════════════════════════════════════
class Shared:
    def __init__(self):
        self.lock = threading.Lock()
        self.phase = "INIT"
        self.airborne = False
        self.last_accept_t = now()        # capture time of the last accepted frame
        self.pose = None                  # last accepted (x, y, z, yaw)
        self.yaw_hold = None
        self.vision_errors = 0


CSV_COLS = [
    "t_capture", "t_send", "ts_source", "latency_ms", "phase", "mode",
    "detected", "marker_px", "marker_cx", "marker_cy", "exposure_us",
    "roll", "pitch", "att_age_ms", "tilt_deg",
    "blur_full", "blur_roi", "brightness",
    "tilt_gated", "outlier_rejected", "accepted", "sent",
    "x_raw", "y_raw", "z_raw", "yaw_marker", "pnp_reproj", "normal_err_deg",
    "x_sent", "y_sent", "z_sent", "sigma_model", "sigma_sent",
    "ekf_x", "ekf_y", "ekf_z", "ekf_vx", "ekf_vy", "ekf_age_ms",
]


class Logger:
    def __init__(self, root, mode, sortie_id):
        self.dir = os.path.join(os.path.expanduser(root), f"{mode}_{sortie_id}")
        os.makedirs(self.dir, exist_ok=True)
        self.f = open(os.path.join(self.dir, "vision_frames.csv"), "w", newline="",
                      encoding="utf-8")
        self.w = csv.writer(self.f)
        self.w.writerow(CSV_COLS)
        self.lock = threading.Lock()
        self.events = []
        self.console = open(os.path.join(self.dir, "console.log"), "w", encoding="utf-8")
        self._stdout = sys.stdout
        sys.stdout = self

    # tee stdout into console.log
    def write(self, s):
        self._stdout.write(s)
        try:
            self.console.write(s)
        except ValueError:
            pass

    def flush(self):
        self._stdout.flush()
        try:
            self.console.flush()
        except ValueError:
            pass

    def row(self, d):
        with self.lock:
            self.w.writerow([d.get(c, "") for c in CSV_COLS])

    def flush_data(self):
        """Push buffered frame rows to disk (called ~1 Hz, so a power cut loses <1 s)."""
        with self.lock:
            try:
                self.f.flush()
                os.fsync(self.f.fileno())
            except (ValueError, OSError):
                pass

    def event(self, name, **kw):
        e = {"t": now(), "event": name}
        e.update(kw)
        with self.lock:
            self.events.append(e)

    def close(self):
        with self.lock:
            if not self.f.closed:
                self.f.close()
        if sys.stdout is self:
            sys.stdout = self._stdout
        if not self.console.closed:
            self.console.close()


def fmt(v, nd=5):
    if v is None:
        return ""
    if isinstance(v, float):
        return "" if math.isnan(v) else f"{v:.{nd}f}"
    return v


# ═══════════════════════════════════════════════════════════════════════════
# CAMERA
# ═══════════════════════════════════════════════════════════════════════════
ZERO_DIST = np.zeros(5)


def distort_normalized(x, y, model, coeffs):
    """Undistorted normalized coords -> distorted, exactly as librealsense
    rs2_project_point_to_pixel (v2.55) does it. Used for the self-test and the unit tests."""
    k1, k2, p1, p2, k3 = coeffs
    r2 = x * x + y * y
    f = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    if model == "brown_conrady":                        # = OpenCV's model
        return (x * f + 2 * p1 * x * y + p2 * (r2 + 2 * x * x),
                y * f + 2 * p2 * x * y + p1 * (r2 + 2 * y * y))
    if model in ("inverse_brown_conrady", "modified_brown_conrady"):
        x, y = x * f, y * f                             # tangential applied AFTER radial
        return (x + 2 * p1 * x * y + p2 * (r2 + 2 * x * x),
                y + 2 * p2 * x * y + p1 * (r2 + 2 * y * y))
    return x, y


def undistort_pixels(pts, K, model, coeffs, iters=20):
    """Detected (distorted) pixel corners -> the pixels an ideal, distortion-free camera with
    the same K would see. Inverse of distort_normalized (same iteration as librealsense
    rs2_deproject_pixel_to_point). solvePnP then runs with ZERO_DIST, so the RealSense
    'inverse Brown-Conrady' colour model is handled correctly (OpenCV has no such model)."""
    pts = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
    if model == "none" or not any(abs(c) > 0.0 for c in coeffs):
        return pts.copy()
    k1, k2, p1, p2, k3 = coeffs
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    xo, yo = (pts[:, 0] - cx) / fx, (pts[:, 1] - cy) / fy
    x, y = xo.copy(), yo.copy()
    for _ in range(iters):
        r2 = x * x + y * y
        icd = 1.0 / (1 + ((k3 * r2 + k2) * r2 + k1) * r2)
        xq, yq = (x, y) if model == "brown_conrady" else (x / icd, y / icd)
        dx = 2 * p1 * xq * yq + p2 * (r2 + 2 * xq * xq)
        dy = 2 * p2 * xq * yq + p1 * (r2 + 2 * yq * yq)
        x, y = (xo - dx) * icd, (yo - dy) * icd
    return np.stack([x * fx + cx, y * fy + cy], axis=1)


def start_camera(camera, exposure_us):
    """Start the chosen D455 stream. Returns a dict with the pipeline and the calibration.
    camera: "rgb" (colour, v6 default) or "ir" (left IR imager IR1, v1-v5)."""
    if rs is None:
        sys.exit("ERROR: pyrealsense2 not installed")
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.disable_all_streams()
    if camera == "rgb":
        cfg.enable_stream(rs.stream.color, CAM_W, CAM_H, rs.format.bgr8, CAM_FPS)
    else:
        cfg.enable_stream(rs.stream.infrared, 1, CAM_W, CAM_H, rs.format.y8, CAM_FPS)
    try:
        prof = pipe.start(cfg)
    except RuntimeError as e:
        sys.exit(f"ERROR: could not start the D455 {camera} stream {CAM_W}x{CAM_H}@{CAM_FPS}: {e}\n"
                 "       Close any other program using the camera; use a USB 3 port + cable.")
    dev = prof.get_device()
    sensor = dev.first_color_sensor() if camera == "rgb" else dev.first_depth_sensor()
    # Colour: never let auto-exposure lower the frame rate in dim light (keep 30 fps).
    # IR: switch the dot projector off (its pattern breaks marker detection).
    extra = (rs.option.auto_exposure_priority, 0) if camera == "rgb" else (rs.option.emitter_enabled, 0)
    for opt, val in ((rs.option.global_time_enabled, 1), extra):
        try:
            if sensor.supports(opt):
                sensor.set_option(opt, val)
        except Exception as e:
            print(f"[CAM] warn {opt}: {e}")
    # IR exposure is in microseconds; the colour exposure is the raw UVC control
    # (exposure_absolute), which counts in 100-microsecond steps.
    exp_unit_us = 100.0 if camera == "rgb" else 1.0
    try:
        if exposure_us > 0:
            sensor.set_option(rs.option.enable_auto_exposure, 0)
            rng = sensor.get_option_range(rs.option.exposure)
            val = min(max(round(exposure_us / exp_unit_us), rng.min), rng.max)
            sensor.set_option(rs.option.exposure, float(val))
            print(f"[CAM] Manual exposure {val:g} x {exp_unit_us:g} us = {val * exp_unit_us:g} us "
                  f"(sensor range {rng.min:g}-{rng.max:g})")
        else:
            sensor.set_option(rs.option.enable_auto_exposure, 1)
            print("[CAM] Auto-exposure ON")
    except Exception as e:
        print(f"[CAM] warn exposure: {e}")
    sp = prof.get_stream(rs.stream.color) if camera == "rgb" else prof.get_stream(rs.stream.infrared, 1)
    intr = sp.as_video_stream_profile().get_intrinsics()
    model = str(intr.model).split(".")[-1]
    coeffs = [float(c) for c in list(intr.coeffs)[:5]]
    if model not in SUPPORTED_DISTORTION:
        pipe.stop()
        sys.exit(f"ERROR: unsupported lens model '{model}' for the {camera} camera")
    K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], dtype=np.float64)
    hfov = math.degrees(2 * math.atan(intr.width / (2 * intr.fx)))
    vfov = math.degrees(2 * math.atan(intr.height / (2 * intr.fy)))
    print(f"[CAM] {camera.upper()} {intr.width}x{intr.height}@{CAM_FPS}  fx={intr.fx:.1f} fy={intr.fy:.1f}  "
          f"FOV {hfov:.1f} x {vfov:.1f} deg  lens model {model} {[round(c, 4) for c in coeffs]}")
    return dict(kind=camera, pipe=pipe, K=K, model=model, coeffs=coeffs, hfov=hfov, vfov=vfov,
                width=intr.width, height=intr.height, exp_unit_us=exp_unit_us)


def camera_public(cam):
    """The calibration part of `cam` that goes into sortie_summary.json."""
    return {k: (v.tolist() if isinstance(v, np.ndarray) else v)
            for k, v in cam.items() if k != "pipe"}


def grab_gray(fs, cam):
    """(frame, grey image) from a frameset for the camera in use, or (None, None)."""
    frame = fs.get_color_frame() if cam["kind"] == "rgb" else fs.get_infrared_frame(1)
    if not frame:
        return None, None
    img = np.asanyarray(frame.get_data())
    return frame, (cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img)


def frame_capture_time(ir, t_arrive):
    """Capture time in the script's monotonic time base (now()).
    RealSense 'global time' is wall-clock; we only use it to measure the frame's AGE
    (wall - wall at the same instant), so a clock step can't move the timeline."""
    try:
        if rs is not None and ir.get_frame_timestamp_domain() == rs.timestamp_domain.global_time:
            age = time.time() - ir.get_timestamp() / 1000.0
            if 0.0 <= age < 0.5:
                return t_arrive - age, "global"
    except Exception:
        pass
    return t_arrive, "arrival"


def frame_exposure(frame, unit_us=1.0):
    """Exposure of this frame in microseconds (colour metadata is in 100-us steps)."""
    try:
        if frame.supports_frame_metadata(rs.frame_metadata_value.actual_exposure):
            return round(frame.get_frame_metadata(rs.frame_metadata_value.actual_exposure) * unit_us, 1)
    except Exception:
        pass
    return ""


# ═══════════════════════════════════════════════════════════════════════════
# VISION THREAD — capture, gate, compute sigma, SEND (one message per accepted frame)
# ═══════════════════════════════════════════════════════════════════════════
class PhaseStats:
    def __init__(self):
        self.d = collections.defaultdict(lambda: collections.defaultdict(list))

    def add(self, phase, **kw):
        for k, v in kw.items():
            if v is not None and not (isinstance(v, float) and math.isnan(v)):
                self.d[phase][k].append(v)

    def summary(self):
        out = {}
        for ph, kv in self.d.items():
            s = {}
            for k, vals in kv.items():
                a = np.asarray(vals, dtype=float)
                if k in ("frames", "detected", "accepted", "tilt_gated", "outlier"):
                    s[k] = int(a.sum())
                else:
                    s[k] = dict(median=float(np.median(a)), p05=float(np.percentile(a, 5)),
                                p95=float(np.percentile(a, 95)), max=float(a.max()),
                                n=int(a.size))
            out[ph] = s
        return out


def vision_loop(args, link, shared, log, stop, cam, B, stats):
    """Runs until `stop` — which main() sets only after the drone has DISARMED (or on a
    forced stop). A failure on one frame is logged and skipped; the loop never dies."""
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(ARUCO_DICT),
                                  _aruco_params())
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    st = dict(filt=None, consec_out=0, last_acc_t=None)     # gating state across frames
    diag = collections.Counter()
    diag_t = now()
    flush_t = now()
    print("[VIS] Thread started.")

    while not stop.is_set():
        try:
            fs = cam["pipe"].wait_for_frames(timeout_ms=2000)
        except Exception:
            continue
        try:
            process_frame(fs, args, link, shared, log, stats, det, clahe, cam, B, st, diag)
        except Exception as e:                               # noqa: BLE001
            with shared.lock:
                shared.vision_errors += 1
                n = shared.vision_errors
            log.event("VISION_ERROR", n=n, error=f"{type(e).__name__}: {e}")
            if n <= 5 or n % 100 == 0:
                print(f"[VIS] frame skipped after error #{n}: {type(e).__name__}: {e}")
        t = now()
        if t - flush_t >= 1.0:
            log.flush_data()
            flush_t = t
        if t - diag_t >= 1.0:
            print_diag(args, shared, diag)
            diag.clear()
            diag_t = t


def process_frame(fs, args, link, shared, log, stats, det, clahe, cam, B, st, diag):
    frame, raw = grab_gray(fs, cam)
    if frame is None:
        return
    t_cap, ts_src = frame_capture_time(frame, now())
    enh = clahe.apply(raw)
    blur_full = laplacian_var(enh)
    brightness = float(raw.mean())

    if link is not None:
        roll, pitch, att_age = link.attitude_at(t_cap)
    else:
        roll, pitch, att_age = 0.0, 0.0, float("nan")
    tilt = true_tilt_deg(roll, pitch)
    with shared.lock:
        phase = shared.phase

    corners, ids, _ = det.detectMarkers(enh)
    idx = None
    if ids is not None:
        for i, mid in enumerate(ids.flatten()):
            if mid == MARKER_ID:
                idx = i
                break

    r = dict(t_capture=f"{t_cap:.6f}", ts_source=ts_src, phase=phase, mode=args.mode,
             detected=0, exposure_us=frame_exposure(frame, cam["exp_unit_us"]),
             roll=fmt(roll), pitch=fmt(pitch),
             att_age_ms=fmt(att_age, 1), tilt_deg=fmt(tilt, 3), blur_full=fmt(blur_full, 1),
             brightness=fmt(brightness, 1), tilt_gated=0, outlier_rejected=0,
             accepted=0, sent=0)
    pose = None
    blur_roi = float("nan")
    if idx is not None:
        c = corners[idx]
        c_ud = undistort_pixels(c, cam["K"], cam["model"], cam["coeffs"])   # lens model -> ideal
        pose = estimate_pose(c_ud, cam["K"], ZERO_DIST, roll, pitch, B)
        pts = c.reshape(4, 2)
        x0, y0 = np.floor(pts.min(0)).astype(int)
        x1, y1 = np.ceil(pts.max(0)).astype(int)
        roi = raw[max(y0, 0):min(y1, raw.shape[0]), max(x0, 0):min(x1, raw.shape[1])]
        blur_roi = laplacian_var(roi) if roi.size > 16 else float("nan")
        r.update(marker_px=int(x1 - x0), marker_cx=fmt(float(pts[:, 0].mean()), 1),
                 marker_cy=fmt(float(pts[:, 1].mean()), 1), blur_roi=fmt(blur_roi, 1))

    accepted = False
    sigma_model = sigma_sent = float("nan")
    if pose is not None:
        x, y, z, yaw, info = pose
        r.update(detected=1, x_raw=fmt(x), y_raw=fmt(y), z_raw=fmt(z), yaw_marker=fmt(yaw),
                 pnp_reproj=fmt(info["reproj"], 3), normal_err_deg=fmt(info["normal_err_deg"], 2))
        blur_for_model = blur_full if BLUR_FOR_MODEL == "full_clahe" else blur_roi
        sigma_model = model_sigma(tilt, blur_for_model if not math.isnan(blur_for_model) else 1.0)

        if tilt > args.tilt_gate:                            # hard gate (all arms)
            r["tilt_gated"] = 1
        else:
            gap = None if st["last_acc_t"] is None else t_cap - st["last_acc_t"]
            if st["filt"] is None or gap is None or gap > REACQ_GAP_S:
                # first fix, or the marker was really gone (> REACQ_GAP_S): re-seed
                st["filt"], st["consec_out"], accepted = np.array([x, y, z]), 0, True
            else:
                # jump check ALWAYS applies after a short dropout; the allowance grows
                # with the time since the last accepted frame (the drone may have moved)
                limit = OUTLIER_JUMP_M + MAX_SPEED_MPS * max(gap, 0.0)
                jump = math.hypot(x - st["filt"][0], y - st["filt"][1])
                if jump > limit:
                    st["consec_out"] += 1
                    r["outlier_rejected"] = 1
                    if st["consec_out"] >= OUTLIER_RESET_COUNT:  # death-lock recovery
                        st["filt"], st["consec_out"], accepted = np.array([x, y, z]), 0, True
                else:
                    st["consec_out"] = 0
                    a = args.ema_alpha
                    st["filt"] = a * np.array([x, y, z]) + (1 - a) * st["filt"]
                    accepted = True

        if accepted:
            filt = st["filt"]
            st["last_acc_t"] = t_cap
            sigma_sent = arm_sigma(args.mode, sigma_model, args.sigma_tuned)
            cov = ap_encode(sigma_sent)
            if link is not None and not args.bench:
                link.send_vision(t_cap, float(filt[0]), float(filt[1]), float(filt[2]),
                                 roll, pitch, yaw, cov)
                r["sent"] = 1
            t_send = now()
            r.update(accepted=1, t_send=f"{t_send:.6f}",
                     latency_ms=fmt((t_send - t_cap) * 1000.0, 2),
                     x_sent=fmt(float(filt[0])), y_sent=fmt(float(filt[1])),
                     z_sent=fmt(float(filt[2])))
            with shared.lock:
                shared.last_accept_t = t_cap
                shared.pose = (float(filt[0]), float(filt[1]), float(filt[2]), yaw)
    r.update(sigma_model=fmt(sigma_model, 5), sigma_sent=fmt(sigma_sent, 5))

    if link is not None:
        (ex, ey, ez, evx, evy, _), eage = link.get_ekf()
        r.update(ekf_x=fmt(ex), ekf_y=fmt(ey), ekf_z=fmt(ez), ekf_vx=fmt(evx, 4),
                 ekf_vy=fmt(evy, 4), ekf_age_ms=fmt(eage, 1))
    log.row(r)
    stats.add(phase, frames=1, detected=int(pose is not None), accepted=int(accepted),
              tilt_gated=r["tilt_gated"], outlier=r["outlier_rejected"], tilt_deg=tilt,
              blur_full=blur_full, sigma_model=sigma_model if pose else None,
              sigma_sent=sigma_sent if accepted else None,
              latency_ms=(float(r["latency_ms"]) if accepted else None),
              att_age_ms=att_age if math.isfinite(att_age) else None)
    diag.update(frames=1, det=int(pose is not None), acc=int(accepted),
                gate=r["tilt_gated"], out=r["outlier_rejected"])
    diag["_tilt"], diag["_blur"] = tilt, blur_full
    diag["_sigma"] = sigma_sent
    if pose is not None:
        diag["_pose"] = pose[:4]
        diag["_sigma_model"] = sigma_model
        diag["_px"] = r.get("marker_px")


def print_diag(args, shared, diag):
    with shared.lock:
        phase, airborne = shared.phase, shared.airborne
    tilt, blur = diag.get("_tilt", float("nan")), diag.get("_blur", float("nan"))
    if args.bench:
        if "_pose" in diag:
            x, y, z, yaw = diag["_pose"]
            sm = diag["_sigma_model"]
            s = {m: arm_sigma(m, sm, args.sigma_tuned) for m in ("constant", "tuned", "adaptive")}
            print(f"[BENCH] x={x:+.3f} y={y:+.3f} z={z:+.3f} m  yaw={math.degrees(yaw):+6.1f} deg"
                  f"  tilt={tilt:4.1f}  blur={blur:5.0f}  px={diag.get('_px')}"
                  f"  sigma A/A'/B = {s['constant']*100:.1f}/{s['tuned']*100:.1f}/"
                  f"{s['adaptive']*100:.1f} cm")
        else:
            print("[BENCH] marker not detected")
    elif airborne or phase not in ("INIT", "WAIT_PILOT"):
        sg = diag.get("_sigma", float("nan"))
        print(f"[DIAG] {phase:10s} fps={diag['frames']:2d} det={diag['det']:2d} "
              f"acc={diag['acc']:2d} gate={diag['gate']:2d} out={diag['out']:2d} "
              f"tilt={tilt:4.1f} blur={blur:4.0f} "
              f"sigma_sent={'-' if not math.isfinite(sg) else f'{sg*100:.1f}cm'}")


def _aruco_params():
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return p


# ═══════════════════════════════════════════════════════════════════════════
# FLIGHT THREAD — identical trajectory for every arm
# ═══════════════════════════════════════════════════════════════════════════
def body_to_world(bx, by, yaw):
    """Rotate a step given in the drone's own axes (nose, right) into the pad frame."""
    c, s_ = math.cos(yaw), math.sin(yaw)
    return c * bx - s_ * by, s_ * bx + c * by


def flight_loop(args, link, shared, log, abort, amp, axis, result):
    """Commands the sequence. Ends after LAND is commanded or the pilot takes over.
    It never stops the vision thread — main() does that only after DISARM."""
    try:
        _flight_sequence(args, link, shared, log, abort, amp, axis, result)
    except Exception as e:                                   # noqa: BLE001
        log.event("ABORT", reason=f"flight_thread_error: {type(e).__name__}: {e}")
        print(f"[FLT] ERROR in flight thread: {type(e).__name__}: {e} — commanding LAND")
        result.update(completed=False, abort_reason="flight_thread_error")
        try:
            if link.get_status()[1] == MODE_GUIDED:
                link.set_mode(MODE_LAND)
        except Exception:
            pass


def _flight_sequence(args, link, shared, log, abort, amp, axis, result):
    def set_phase(p, **kw):
        with shared.lock:
            shared.phase = p
        log.event(p, **kw)
        print(f"[FLT] -> {p}")

    def marker_lost_s():
        with shared.lock:
            return now() - shared.last_accept_t

    def height_m():
        (_, _, ez, *_), age_ms = link.get_ekf()
        return -ez if age_ms < 1000 else float("nan")

    hz = 20.0
    z_hover = -args.alt
    set_phase("WAIT_PILOT")
    print("\n" + "=" * 60)
    print(">>> PILOT: drone on the platform/pad, camera over the marker, nose along +X.")
    print(">>>        Arm in STABILIZE and switch to GUIDED at ANY height (ground included):")
    print(f">>>        on the ground the script takes off by itself after {TKOFF_COUNTDOWN_S:g} s.")
    print(">>> Pilot keeps RC override at all times (mode switch = abort).")
    print("=" * 60 + "\n")
    warn_t, rng_warned = 0.0, False
    t_wait = now()
    ekf_ok_since, ekf_print_t = None, 0.0                    # v7.3 (--no-flow only)
    if args.no_flow:
        print(">>> E3 (no optical flow): take-off only after the EKF reports ABSOLUTE position "
              f"for {EKF_GATE_S:g} s (camera must see the marker on the pad).")
    while not abort.is_set():
        if args.no_flow:
            flags, ekf_age, ekf_ok = link.ekf_status()
            ekf_ok_since = (ekf_ok_since or now()) if ekf_ok else None
            if now() - ekf_print_t > 3.0:
                state = ("OK" if ekf_ok else "NOT READY") + \
                    (f" (flags 0x{flags:04x})" if flags is not None else " (no EKF_STATUS_REPORT yet)")
                print(f"[EKF] absolute horizontal position: {state}")
                ekf_print_t = now()
        if not rng_warned and now() - t_wait > 3.0 and link.get_range()[1] > 1000:
            rng_warned = True
            print("[FLT] WARNING: no rangefinder data (DISTANCE_SENSOR) from the FC — touchdown "
                  "auto-disarm is OFF this sortie.")
            log.event("NO_RANGEFINDER_DATA")
        armed, mode, _ = link.get_status()
        if armed and mode == MODE_GUIDED:
            att_age = link.attitude_at(now())[2]
            ekf_ready = (not args.no_flow) or (ekf_ok_since is not None
                                               and now() - ekf_ok_since >= EKF_GATE_S)
            if att_age < MAX_ATT_AGE_MS and ekf_ready:        # v6.1a: no minimum height any more
                break
            if att_age < MAX_ATT_AGE_MS and not ekf_ready and now() - warn_t > 3.0:
                print("[FLT] GUIDED but the EKF has NO ABSOLUTE POSITION — NOT taking off. "
                      "Check the marker is in view; if it stays NOT READY, disarm and reboot "
                      "the flight controller.")
                log.event("GUIDED_NO_EKF_POSITION", flags=link.ekf_status()[0])
                warn_t = now()
            elif att_age >= MAX_ATT_AGE_MS and now() - warn_t > 3.0:   # tilt would read 0.0
                print("[FLT] GUIDED but NO ATTITUDE from the FC — NOT starting. "
                      "Check SR1_EXTRA1 / check_attitude_link.py, then re-select GUIDED.")
                log.event("GUIDED_NO_ATTITUDE", att_age_ms=att_age)
                warn_t = now()
        time.sleep(0.05)
    if abort.is_set():
        result.update(completed=False, abort_reason="operator_abort_before_start")
        return
    if not rng_warned and link.get_range()[1] > 1000:          # GUIDED selected quickly
        print("[FLT] WARNING: no rangefinder data (DISTANCE_SENSOR) from the FC — touchdown "
              "auto-disarm is OFF this sortie.")
        log.event("NO_RANGEFINDER_DATA")

    result["ekf_flags_at_start"] = link.ekf_status()[0]         # v7.3 (all experiments)
    log.event("EKF_STATUS_AT_START", flags=result["ekf_flags_at_start"])
    with shared.lock:
        shared.airborne = True
        pose = shared.pose
    yaw_hold = pose[3] if pose else 0.0
    _, _, v = link.get_status()
    set_phase("TAKEOFF", battery_v=v, yaw_hold=yaw_hold,
              heading_deg=math.degrees(yaw_hold), height_m=height_m())
    if abs(math.degrees(yaw_hold)) > 20:
        print(f"[FLT] note: heading {math.degrees(yaw_hold):+.0f} deg from pad +X. Steps are "
              "rotated to stay on the drone's own axis, so this is safe; pad dx/dy stay in pad frame.")

    reason = {"why": None}
    att_stale = {"t": None}
    ekf_bad = {"t": None}                                       # v7.3 (--no-flow only)

    def guard(check_marker=True):
        """Reason string if the sortie must stop, else None. The landing controller handles
        a lost marker itself (check_marker=False)."""
        if abort.is_set():
            return "operator_ctrl_c"
        if link.heartbeat_age() > HB_TIMEOUT_S:
            return "fc_link_lost"
        if link.attitude_at(now())[2] > MAX_ATT_AGE_MS:      # v7.1: one short gap is not a loss
            att_stale["t"] = att_stale["t"] or now()
            if now() - att_stale["t"] > ATT_LOST_ABORT_S:
                return "attitude_lost"
        else:
            att_stale["t"] = None
        if args.no_flow:                                         # v7.3: vision is the only aid
            if not link.ekf_status()[2]:
                ekf_bad["t"] = ekf_bad["t"] or now()
                if now() - ekf_bad["t"] > EKF_LOST_ABORT_S:
                    return "ekf_no_position"
            else:
                ekf_bad["t"] = None
        armed, mode, _ = link.get_status()
        if not armed:
            return "disarmed"
        if mode != MODE_GUIDED:
            return f"pilot_mode_change_{mode}"
        if check_marker and marker_lost_s() > args.lost_timeout:
            return "marker_lost"
        return None

    def hold(x, y, z, seconds, until=None):
        """'done' (condition met or time served), 'timeout' (condition never met), 'abort'."""
        t_end = now() + seconds
        while now() < t_end:
            g = guard()
            if g:
                reason["why"] = g
                return "abort"
            link.send_setpoint(x, y, z, yaw_hold)
            if until is not None and until():
                return "done"
            time.sleep(1.0 / hz)
        return "timeout" if until is not None else "done"

    def step(name, res):
        if res == "abort":
            return False
        if res == "timeout":
            reason["why"] = f"{name.lower()}_timeout"
            print(f"[FLT] {name} did not complete in time -> abort sortie")
            return False
        return True

    # ── LIFTOFF (v6.1a): GUIDED was selected at any height ───────────────────────
    # ArduCopter ignores position setpoints while it considers itself landed (ON_GROUND),
    # so then the script sends MAV_CMD_NAV_TAKEOFF first. If it already counts as flying
    # (IN_AIR - e.g. the throttle was raised in STABILIZE, which clears 'landed' at once,
    # and makes NAV_TAKEOFF fail with MAV_RESULT 4) it climbs with setpoints directly.
    ok = True
    z0 = link.get_ekf()[0][2]
    z0_pad, rng_pad = None, []                            # v6.1b: pad reference on the ground
    ls, ls_age = link.landed_state()
    landed_known = ls is not None and ls_age < 2000
    if not landed_known or ls == 1:                       # ON_GROUND (or unknown) -> take off
        print(f"[FLT] TAKE-OFF in {TKOFF_COUNTDOWN_S:g} s — switch out of GUIDED to cancel"
              f"{'' if landed_known else ' (landed state unknown)'}")
        t_c = now() + TKOFF_COUNTDOWN_S
        while ok and now() < t_c:
            g = guard()
            if g:
                reason["why"], ok = g, False
            r, r_age = link.get_range()
            if r_age < 300 and math.isfinite(r) and RNG_MIN_VALID_M <= r <= 2 * RNG_ON_GROUND_M:
                rng_pad.append(r)
            time.sleep(0.05)
        z0 = link.get_ekf()[0][2]
        z0_pad = z0
        if ok:
            t_sent, resent = now(), False
            link.send_takeoff(TKOFF_CMD_ALT_M)
            log.event("NAV_TAKEOFF", alt_m=TKOFF_CMD_ALT_M, landed_state=ls)
            t_end = now() + LIFTOFF_TIMEOUT_S
            while True:
                g = guard()
                if g:
                    reason["why"], ok = g, False
                    break
                if link.get_ekf()[0][2] <= z0 - LIFTOFF_RISE_M:
                    log.event("LIFTOFF", rise_m=z0 - link.get_ekf()[0][2])
                    print("[FLT] airborne")
                    break
                ack = link.ack_after(MAV_CMD_NAV_TAKEOFF_ID, t_sent)
                if ack is not None and ack not in (0, 5):           # not ACCEPTED / IN_PROGRESS
                    # FC already counts as flying -> position setpoints work: climb with them
                    log.event("NAV_TAKEOFF_REJECTED", result=ack)
                    print(f"[FLT] take-off command not needed (FC says flying, MAV_RESULT {ack}) "
                          "— climbing with position setpoints")
                    break
                if ack is None and not resent and now() - t_sent > 1.0:   # no answer: once more
                    t_sent, resent = now(), True
                    link.send_takeoff(TKOFF_CMD_ALT_M)
                if now() > t_end:
                    reason["why"], ok = "liftoff_timeout", False
                    print(f"[FLT] did not leave the ground within {LIFTOFF_TIMEOUT_S:g} s")
                    break
                time.sleep(0.05)
    if ok:
        ok = step("TAKEOFF", hold(0, 0, z_hover, TAKEOFF_TIMEOUT_S,
                                  until=lambda: link.get_ekf()[0][2] <= z_hover + ALT_CONFIRM_M))
    if ok:
        set_phase("SETTLE")
        ok = step("SETTLE", hold(0, 0, z_hover, SETTLE_TIMEOUT_S,
                                 until=lambda: math.hypot(*link.get_ekf()[0][:2]) < SETTLE_DRIFT_M))
    if ok:
        set_phase("HOVER_BASE")
        ok = step("HOVER_BASE", hold(0, 0, z_hover, HOVER_BASE_S))
    if ok:
        set_phase("MANEUVER", axis=axis, amplitude_m=amp, cycles=args.cycles,
                  step_hold_s=STEP_HOLD_S, heading_deg=math.degrees(yaw_hold))
        targets = []
        for _ in range(args.cycles):
            targets += [amp, -amp]
        targets.append(0.0)
        for k, sv in enumerate(targets):
            bx, by = (sv, 0.0) if axis == "nose" else (0.0, sv)
            tx, ty = body_to_world(bx, by, yaw_hold)           # along the drone's own axis
            log.event("STEP", k=k, target_x=tx, target_y=ty, body_x=bx, body_y=by)
            if not step("STEP", hold(tx, ty, z_hover, STEP_HOLD_S)):
                ok = False
                break
    if ok:
        set_phase("CONVERGE")
        ok = step("CONVERGE", hold(0, 0, z_hover, CONVERGE_S))
    # Landing (v7, EKF-only). The pad's EKF height is measured (v6.1b), then the setpoint goes
    # straight down at LAND_VZ_MPS to the EKF origin (0, 0) - the same in every arm and with no
    # marker feedback, so where the drone touches down is decided by that arm's EKF alone.
    # The camera only LOGS where it last saw the CG (LANDING_END: last_fix_*), so the analysis
    # can split the touchdown error into "EKF error when the camera went blind" + "blind drift".
    # Touchdown -> forced disarm. ArduPilot LAND is only the fallback.
    auto = not getattr(args, "no_auto_disarm", False)
    td = TouchdownWatch(link)

    def ekf_landing(z_pad, pad_measured, r_pad):
        """'touchdown', 'handover' (ArduPilot LAND finishes), 'timeout' or 'abort'."""
        off = tuple(getattr(args, "cam_offset_xy", None) or CAM_OFFSET_DEFAULT)
        own_td = auto and link.get_range()[1] < 1000
        z_end = z_pad + LAND_BELOW_PAD_M                   # NED: below the pad
        z_sp = link.get_ekf()[0][2]
        set_phase("LANDING", method="ekf_only", speed_mps=LAND_VZ_MPS, z_pad=z_pad,
                  z_end=z_end, own_touchdown=own_td)
        if not own_td:
            print("[LND] " + ("--no-auto-disarm" if not auto else "no rangefinder data") +
                  f": EKF descent to {LAND_HANDOVER_M:.2f} m, then ArduPilot LAND")
        last_cap, last_fix, n_fix = None, None, 0
        lost_since, lost_logged = None, False
        t0 = t_prev = now()
        t_status, t_bottom, out = 0.0, None, None
        while True:
            t = now()
            dt = min(max(t - t_prev, 0.0), 0.2)
            t_prev = t
            g = guard(check_marker=False)              # blind near the pad is expected
            if g:
                reason["why"], out = g, "abort"
                break
            (ex, ey, ez, *_), _ = link.get_ekf()
            r, r_age = link.get_range()
            h_ekf = z_pad - ez
            h = h_ekf
            if r_age < 300 and math.isfinite(r) and r >= RNG_MIN_VALID_M:
                h = r - r_pad
                if pad_measured and h_ekf - h > 0.30:            # implausibly low reading
                    h = h_ekf
            # --- logging only: where did the camera last see the CG? (never fed back) ---
            with shared.lock:
                pose, t_acc = shared.pose, shared.last_accept_t
            if pose is not None and t_acc != last_cap and t_acc >= t0:
                last_cap, n_fix = t_acc, n_fix + 1
                cx, cy = cg_from_camera(pose[0], pose[1], pose[3], off)
                last_fix = dict(t=t_acc, cg_x=cx, cg_y=cy, h=h, ekf_x=ex, ekf_y=ey)
                lost_since = None
            elif t - t_acc > 0.5:
                lost_since = lost_since or t
                if not lost_logged and math.isfinite(h) and h > LAND_LOST_LOG_M \
                        and t - lost_since > 1.0:
                    lost_logged = True
                    log.event("MARKER_LOST_IN_LANDING", h=h)
                    print(f"[LND] marker lost at {h:.2f} m — continuing on the EKF (logged)")
            # --- the descent itself: EKF only ---
            z_sp = min(z_sp + LAND_VZ_MPS * dt, ez + LAND_LEAD_MAX_M, z_end)
            if t_bottom is None and z_sp >= z_end - 1e-6:
                t_bottom = t
            link.send_setpoint(0.0, 0.0, z_sp, yaw_hold)
            if own_td and td.update():
                out = "touchdown"
                break
            if not own_td and math.isfinite(h) and h <= LAND_HANDOVER_M:
                out = "handover"
                break
            if (t_bottom is not None and t - t_bottom > LAND_BOTTOM_WAIT_S) or t - t0 > LAND_TIMEOUT_S:
                out = "timeout"
                print("[LND] no touchdown detected — handing over to ArduPilot LAND")
                break
            if t - t_status >= LAND_STATUS_S:
                t_status = t
                mk = "seen" if t - t_acc <= 0.5 else "LOST"
                print(f"[LND] height {h:4.2f} m | EKF off origin {math.hypot(ex, ey) * 100:4.1f} cm"
                      f" | marker {mk}")
            time.sleep(1.0 / hz)
        t_end = now()
        (ex, ey, ez, *_), _ = link.get_ekf()
        ev = dict(result=out, duration_s=t_end - t0, n_marker_fixes=n_fix,
                  ekf_x_end=ex, ekf_y_end=ey)
        if last_fix:
            ev.update(last_fix_cg_x=last_fix["cg_x"], last_fix_cg_y=last_fix["cg_y"],
                      last_fix_h=last_fix["h"], last_fix_ekf_x=last_fix["ekf_x"],
                      last_fix_ekf_y=last_fix["ekf_y"], blind_s=t_end - last_fix["t"])
        log.event("LANDING_END", **ev)
        if last_fix:
            print(f"[LND] last marker fix at {last_fix['h']:.2f} m: CG {last_fix['cg_x'] * 100:+.1f}, "
                  f"{last_fix['cg_y'] * 100:+.1f} cm from the marker; blind for {ev['blind_s']:.1f} s")
        return out

    if ok:
        set_phase("DESCENT")
        link.auto_disarm = auto
        samples = []

        def sample_pad():
            r, r_age = link.get_range()
            (_, _, ez, *_), e_age = link.get_ekf()
            if r_age < 300 and e_age < 500 and math.isfinite(r) and r > RNG_ON_GROUND_M:
                samples.append(ez + r - r_pad)          # NED: pad is r - r_pad below the drone
            return False

        r_pad = float(np.median(rng_pad)) if len(rng_pad) >= 5 else RNG_PAD_DEFAULT_M
        if hold(0, 0, z_hover, PAD_REF_S, until=sample_pad) == "abort":
            ok = False
    if ok:
        if len(samples) >= 5:
            z_pad, src = float(np.median(samples)), "rangefinder"
        elif z0_pad is not None:
            z_pad, src = z0_pad, "ekf_on_pad"
        else:
            z_pad, src = -DESCENT_ALT_M, "fixed_v6.1"
        if z0_pad is not None and src == "rangefinder" and abs(z_pad - z0_pad) > 0.15:
            print(f"[FLT] note: pad EKF z by rangefinder {z_pad:+.2f} vs measured at take-off "
                  f"{z0_pad:+.2f} — using the rangefinder value")
        link.z_pad = z_pad if src != "fixed_v6.1" else None
        log.event("PAD_REF", z_pad=z_pad, source=src, z_pad_takeoff=z0_pad, rng_pad_m=r_pad,
                  rng_pad_measured=len(rng_pad) >= 5, n_samples=len(samples))
        print(f"[FLT] pad at EKF z {z_pad:+.2f} m ({src}, rangefinder on pad {r_pad:.2f} m)")
        land_res = ekf_landing(z_pad, src != "fixed_v6.1", r_pad)
        if land_res == "abort":
            ok = False
        elif land_res == "touchdown":
            set_phase("TOUCHDOWN")
            force_disarm(link, log, "GUIDED landing")

    if not ok:
        log.event("ABORT", reason=reason["why"])
        print(f"[FLT] sortie aborted: {reason['why']}")
    (ex, ey, ez, *_), _ = link.get_ekf()
    with shared.lock:
        vp = shared.pose
    log.event("DESCENT_END", ekf_x=ex, ekf_y=ey, ekf_z=ez,
              vision_x=vp[0] if vp else None, vision_y=vp[1] if vp else None)

    armed, mode, _ = link.get_status()
    pilot_has_it = reason["why"] and reason["why"].startswith("pilot_mode_change")
    # also after an abort the script lands it; v7.1: after a pilot takeover NEVER (the pilot
    # may use the LAND switch and ArduPilot then disarms by itself)
    link.auto_disarm = auto and not pilot_has_it
    if armed and not pilot_has_it and mode in (MODE_GUIDED, MODE_LAND):   # never override a pilot
        set_phase("LAND")
        for _ in range(20):
            link.set_mode(MODE_LAND)
            time.sleep(0.5)
            md = link.get_status()[1]
            if md == MODE_LAND or md not in (MODE_GUIDED, MODE_LAND):
                break
    result.update(completed=ok, abort_reason=reason["why"] or "")
    print("[FLT] commanding finished — vision keeps feeding EKF3 until DISARM.")


# ═══════════════════════════════════════════════════════════════════════════
# PRE-FLIGHT
# ═══════════════════════════════════════════════════════════════════════════
def self_test():
    for s in (0.01, 0.017, 0.2, 0.5):
        pos, _ = ap_decode(ap_encode(s))
        assert abs(pos - s) < 1e-9, "covariance encoding self-test failed"
    # lens undistortion must exactly invert the RealSense distortion models
    K = np.array([[380.0, 0, 320], [0, 380.0, 240], [0, 0, 1]])
    co = [-0.055, 0.065, -0.0007, 0.0006, -0.021]
    g = np.array([[-0.8, -0.6], [0.0, 0.0], [0.5, -0.3], [0.79, 0.59]])
    for model in ("brown_conrady", "inverse_brown_conrady"):
        dx, dy = distort_normalized(g[:, 0], g[:, 1], model, co)
        px = np.stack([dx * 380 + 320, dy * 380 + 240], 1)
        ideal = np.stack([g[:, 0] * 380 + 320, g[:, 1] * 380 + 240], 1)
        assert np.abs(undistort_pixels(px, K, model, co) - ideal).max() < 1e-3, \
            f"lens undistortion self-test failed ({model})"


def camera_check(args, n=100):
    """--cam-check: no flight. Same camera set-up and detector as the flight; reports how
    often marker 0 is found and saves a snapshot with the detections drawn on it."""
    cam = start_camera(args.camera, args.exposure_us)
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(ARUCO_DICT), _aruco_params())
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    hits, got, enh, corners, ids, rej, raw = 0, 0, None, (), None, (), None
    try:
        for _ in range(n + 15):                       # first 15 frames: let exposure settle
            frame, raw = grab_gray(cam["pipe"].wait_for_frames(timeout_ms=3000), cam)
            if frame is None:
                continue
            got += 1
            if got <= 15:
                continue
            enh = clahe.apply(raw)
            corners, ids, rej = det.detectMarkers(enh)
            hits += int(ids is not None and MARKER_ID in ids.ravel())
    finally:
        cam["pipe"].stop()
    used = max(got - 15, 0)
    if enh is None:
        sys.exit("[CHK] no frames from the camera")
    out = cv2.cvtColor(enh, cv2.COLOR_GRAY2BGR)
    cv2.aruco.drawDetectedMarkers(out, corners, ids)
    if len(rej):
        cv2.aruco.drawDetectedMarkers(out, rej, borderColor=(0, 0, 255))
    path = os.path.join(os.path.expanduser(args.log_root), f"cam_check_{cam['kind']}.png")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cv2.imwrite(path, out)
    pct = 100.0 * hits / used if used else 0.0
    print(f"[CHK] {cam['kind'].upper()}: marker {MARKER_ID} found in {hits}/{used} frames ({pct:.0f} %)"
          f" | brightness {raw.mean():.0f}/255 | blur {laplacian_var(enh):.0f}")
    print(f"[CHK] snapshot saved: {path}")
    print("[CHK] " + ("PASS (>= 90 %)" if pct >= 90 else
                      "FAIL (< 90 %): centre the lens over the marker, fix glare/lighting, try --exposure-us"))


def check_params(link, args):
    print("[PRE] Reading flight-controller parameters...")
    got, problems = {}, []
    for name, ok_fn, want in required_param_checks(flow=not args.no_flow):
        v = link.read_param(name)
        got[name] = v
        if not ok_fn(v):
            problems.append(f"{name} = {v}  (must be {want})")
    for name in LOGGED_PARAMS:
        got[name] = link.read_param(name)
    for k, v in got.items():
        print(f"      {k:18s} = {v}")
    if problems:
        print("[PRE] PARAMETER PROBLEMS:")
        for p_ in problems:
            print("      - " + p_)
        if not args.force:
            sys.exit("[PRE] Refusing to fly. Fix parameters (or --force for bench work).")
    return got


def script_sha256():
    try:
        with open(os.path.abspath(__file__), "rb") as f:
            return hashlib.sha256(f.read()).hexdigest()
    except OSError:
        return None


def ask_touchdown():
    print("\n" + "=" * 60)
    print("GROUND TRUTH — measure on the pad grid, BEFORE moving the drone:")
    print("  reference point = centroid of the landing-gear contact points")
    print("  dx = offset along pad +X arrow, dy = along pad +Y arrow, in cm")
    print("  (the sortie is ALREADY saved. Enter or Ctrl-C skips; add it later with")
    print("   --add-touchdown <SORTIE_ID> <DX_CM> <DY_CM>)")
    try:
        s = input("  dx_cm dy_cm > ").strip()
        if not s:
            return None
        dx, dy = (float(v) for v in s.replace(",", " ").split()[:2])
        note = input("  note (optional) > ").strip()
        return dict(dx_cm=dx, dy_cm=dy, radial_cm=math.hypot(dx, dy), note=note)
    except (ValueError, EOFError, KeyboardInterrupt):
        print("\n  skipped — add it later with --add-touchdown")
        return None


# ═══════════════════════════════════════════════════════════════════════════
# CAMPAIGN LEDGER + COMPARISON TABLE
#   After every sortie one row is written to <log-root>/campaign_ledger.csv.
#   `--compare` turns the ledger into the A / A' / B comparison table.
#   This is the quick, in-field table. The paper-grade analysis (EKF3 log,
#   NIS, figures) is still analyze_vqa_ab.py.
# ═══════════════════════════════════════════════════════════════════════════
LEDGER_NAME = "campaign_ledger.csv"
LEDGER_COLS = [
    "sortie_id", "date_time", "experiment", "mode", "arm", "completed", "abort_reason",
    "valid", "td_dx_cm", "td_dy_cm", "td_radial_cm", "td_note",
    "sigma_hover_cm", "sigma_man_med_cm", "sigma_man_p05_cm", "sigma_man_p95_cm", "manip_ok",
    "sigma_man_ratio", "sigma_pinned_frac",
    "tilt_max_deg", "man_accept_pct", "overshoot_cm", "settle_s", "hold_rms_cm",
    "latency_ms", "ekf_final_n_cm", "ekf_final_e_cm", "battery_v",
    "sigma_tuned_m", "tilt_gate_deg", "ema_alpha", "amplitude_m", "axis", "log_dir",
    "camera", "version",
]
SETTLE_RADIUS_CM = 5.0
SMOOTH_FRAMES = 15          # 0.5 s rolling median before overshoot/settle (removes PnP noise)
MIN_N_FOR_TESTS = 5


def rolling_median(a, w=SMOOTH_FRAMES):
    a = np.asarray(a, float)
    h, n = w // 2, len(a)
    return np.array([np.nanmedian(a[max(0, i - h):min(n, i + h + 1)]) for i in range(n)])


def recovery_metrics(t, x, y, prev_target, t_step):
    """Overshoot (cm), settling time (s) and hold RMS (cm) after the final step back to
    the origin, from vision positions SMOOTHED over 0.5 s so single noisy frames can't
    dominate. prev_target = where the drone came from (world x, y); overshoot is how far
    it went past the origin on the other side. Settling = time after which the smoothed
    position stays within SETTLE_RADIUS_CM for the rest of the window (NaN = never)."""
    t, x, y = (np.asarray(v, float) for v in (t, x, y))
    ok = np.isfinite(t) & np.isfinite(x) & np.isfinite(y)
    t, x, y = t[ok], x[ok], y[ok]
    if len(t) < SMOOTH_FRAMES:
        return float("nan"), float("nan")
    order = np.argsort(t)
    t, xs, ys = t[order], rolling_median(x[order]), rolling_median(y[order])
    norm = math.hypot(*prev_target)
    u = (prev_target[0] / norm, prev_target[1] / norm) if norm > 1e-6 else (0.0, 0.0)
    overshoot = max(0.0, float(np.max(-(xs * u[0] + ys * u[1])))) * 100
    r = np.hypot(xs, ys) * 100
    out = np.where(r > SETTLE_RADIUS_CM)[0]
    if len(out) == 0:
        settle = 0.0
    elif out[-1] < len(r) - 1:
        settle = float(t[out[-1] + 1] - t_step)
    else:
        settle = float("nan")
    return overshoot, settle


def _f(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else float("nan")
    except (TypeError, ValueError):
        return float("nan")


def _pct(a, q):
    a = [x for x in a if math.isfinite(x)]
    return float(np.percentile(a, q)) if a else float("nan")


def _med(a):
    return _pct(a, 50)


def _r(x, nd=2):
    return "" if x is None or (isinstance(x, float) and not math.isfinite(x)) else round(x, nd)


def ledger_path(log_root):
    return os.path.join(os.path.expanduser(log_root), LEDGER_NAME)


def read_ledger(log_root):
    p = ledger_path(log_root)
    if not os.path.exists(p):
        return []
    with open(p, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def write_ledger(log_root, rows):
    p = ledger_path(log_root)
    os.makedirs(os.path.dirname(p), exist_ok=True)
    tmp = p + ".tmp"
    with open(tmp, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=LEDGER_COLS, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({c: r.get(c, "") for c in LEDGER_COLS})
    os.replace(tmp, p)                       # atomic: a crash never corrupts the ledger


def upsert_ledger(log_root, row):
    rows = [r for r in read_ledger(log_root) if r.get("sortie_id") != row["sortie_id"]]
    replaced = len(rows) != len(read_ledger(log_root))
    rows.append(row)
    write_ledger(log_root, rows)
    return replaced


ADAPT_WEAK_RATIO = 1.05      # below this, B's sigma barely varied (gentle sortie): warn only


def manip_check(mode, med, p05, p95, sigma_tuned_m, model_match=None, pinned_frac=None):
    """Did EKF3 get what this arm intended? (script side; confirm with VISP.PErr).
    A : flat 20 cm.   A': flat at --sigma-tuned.
    B : every accepted frame sent exactly the model's sigma (model_match) and sigma is not
        stuck at a limit (pinned_frac < 0.5). How MUCH it varied depends on how hard the
        drone tilted, not on whether the arm worked — so weak variation is a warning, not
        a reason to discard the sortie (v3/v4 discarded it)."""
    if not all(math.isfinite(v) for v in (med, p05, p95)):
        return False
    if mode == "constant":
        return abs(med - SIGMA_DEFAULT_M * 100) < 0.5 and (p95 - p05) < 0.5
    if mode == "tuned":
        return abs(med - sigma_tuned_m * 100) < 0.5 and (p95 - p05) < 0.5
    if model_match is None:                      # old summaries without per-frame data
        return p95 / max(p05, 1e-6) > ADAPT_WEAK_RATIO
    return bool(model_match) and (pinned_frac is not None and pinned_frac < 0.5)


def build_ledger_row(summary, frames_csv):
    """Everything the comparison table needs, from this sortie's own files."""
    ev = summary.get("events", [])
    a = summary.get("args", {}) or {}
    mode = summary["mode"]

    def ev_t(name):
        ts = [e["t"] for e in ev if e.get("event") == name]
        return ts[0] if ts else None

    rows = []
    with open(frames_csv, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            rows.append(r)
    acc = [r for r in rows if r.get("accepted") == "1"]
    hover = [_f(r["sigma_sent"]) * 100 for r in acc if r.get("phase") == "HOVER_BASE"]
    man_s = [_f(r["sigma_sent"]) * 100 for r in acc if r.get("phase") == "MANEUVER"]
    man_all = [r for r in rows if r.get("phase") == "MANEUVER"]
    tilt = [_f(r["tilt_deg"]) for r in man_all]
    lat = [_f(r["latency_ms"]) for r in acc]

    # recovery after the final step back to the origin (vision-measured, smoothed)
    overshoot = settle = hold = float("nan")
    steps = [e for e in ev if e.get("event") == "STEP"]
    t_desc = ev_t("DESCENT")
    if len(steps) >= 2 and t_desc:
        last, prev = steps[-1], steps[-2]
        w = [r for r in rows if r.get("detected") == "1"
             and last["t"] <= _f(r["t_capture"]) <= t_desc]
        if len(w) > SMOOTH_FRAMES:
            overshoot, settle = recovery_metrics(
                [_f(r["t_capture"]) for r in w], [_f(r["x_raw"]) for r in w],
                [_f(r["y_raw"]) for r in w],
                (prev.get("target_x", 0.0), prev.get("target_y", 0.0)), last["t"])
        conv = [r for r in rows if r.get("phase") == "CONVERGE" and r.get("detected") == "1"]
        if len(conv) > SMOOTH_FRAMES:
            cx = rolling_median([_f(r["x_raw"]) for r in conv])
            cy = rolling_median([_f(r["y_raw"]) for r in conv])
            hold = float(np.sqrt(np.nanmean(cx ** 2 + cy ** 2))) * 100

    med, p05, p95 = _med(man_s), _pct(man_s, 5), _pct(man_s, 95)
    # Arm B: did every accepted frame send exactly the model's sigma, and is it not pinned?
    pairs = [(_f(r["sigma_model"]), _f(r["sigma_sent"])) for r in acc]
    pairs = [(m_, s_) for m_, s_ in pairs if math.isfinite(m_) and math.isfinite(s_)]
    model_match = bool(pairs) and all(
        abs(s_ - min(max(m_, SIGMA_FLOOR_M), SIGMA_CEIL_M)) < 2e-5 for m_, s_ in pairs)
    pinned = (sum(abs(s_ - SIGMA_FLOOR_M) < 2e-5 or abs(s_ - SIGMA_CEIL_M) < 2e-5
                  for _, s_ in pairs) / len(pairs)) if pairs else float("nan")
    ratio = p95 / p05 if (math.isfinite(p05) and p05 > 0 and math.isfinite(p95)) else float("nan")
    ok = manip_check(mode, med, p05, p95, _f(a.get("sigma_tuned", 0.017)),
                     model_match=model_match if mode == "adaptive" else None,
                     pinned_frac=pinned if mode == "adaptive" else None)
    truth = summary.get("touchdown_truth") or {}
    ekf = summary.get("ekf_final_m") or (float("nan"), float("nan"))
    batt = [e.get("battery_v") for e in ev if e.get("event") == "TAKEOFF"]
    aborts = ";".join(e.get("reason", "") for e in ev if e.get("event") == "ABORT")
    completed = bool(summary.get("completed"))
    return dict(
        sortie_id=summary["sortie_id"],
        date_time=datetime.datetime.now().strftime("%Y-%m-%d %H:%M"),
        experiment=a.get("experiment", "E1"), mode=mode, arm=ARM_SHORT[mode],
        completed=int(completed), abort_reason=aborts,
        valid=int(completed and ok and not any(e.get("event") == "LANDING_MARKER_NOT_FOUND" for e in ev)),
        td_dx_cm=_r(_f(truth.get("dx_cm"))), td_dy_cm=_r(_f(truth.get("dy_cm"))),
        td_radial_cm=_r(_f(truth.get("radial_cm"))), td_note=truth.get("note", ""),
        sigma_hover_cm=_r(_med(hover), 3), sigma_man_med_cm=_r(med, 3),
        sigma_man_p05_cm=_r(p05, 3), sigma_man_p95_cm=_r(p95, 3), manip_ok=int(ok),
        sigma_man_ratio=_r(ratio, 3), sigma_pinned_frac=_r(pinned, 3),
        tilt_max_deg=_r(max([t for t in tilt if math.isfinite(t)], default=float("nan")), 1),
        man_accept_pct=_r(100.0 * sum(r.get("accepted") == "1" for r in man_all) / len(man_all)
                          if man_all else float("nan"), 1),
        overshoot_cm=_r(overshoot), settle_s=_r(settle), hold_rms_cm=_r(hold),
        latency_ms=_r(_med(lat), 1),
        ekf_final_n_cm=_r(_f(ekf[0]) * 100), ekf_final_e_cm=_r(_f(ekf[1]) * 100),
        battery_v=_r(_f(batt[0]) if batt else float("nan")),
        sigma_tuned_m=a.get("sigma_tuned", ""), tilt_gate_deg=a.get("tilt_gate", ""),
        ema_alpha=a.get("ema_alpha", ""), amplitude_m=summary.get("amplitude_m", ""),
        axis=summary.get("axis", ""), log_dir=os.path.dirname(os.path.abspath(frames_csv)),
        camera=a.get("camera") or "ir",                  # v1-v5 summaries had no camera -> IR1
        version=summary.get("version", ""))


def add_touchdown(log_root, sid, dx, dy):
    rows = read_ledger(log_root)
    hit = [r for r in rows if r.get("sortie_id") == sid]
    if not hit:
        sys.exit(f"Sortie {sid} not in {ledger_path(log_root)}")
    ledger_dir = os.path.dirname(ledger_path(log_root))
    for r in hit:
        r.update(td_dx_cm=_r(dx), td_dy_cm=_r(dy), td_radial_cm=_r(math.hypot(dx, dy)))
        js = find_sortie_json(r.get("log_dir", ""), ledger_dir)
        if js:                                       # keep the sortie's own record in sync
            with open(js, encoding="utf-8") as f:
                sj = json.load(f)
            sj["touchdown_truth"] = dict(dx_cm=dx, dy_cm=dy, radial_cm=math.hypot(dx, dy),
                                         note="added later with --add-touchdown")
            write_json_atomic(js, sj)
            print(f"  updated {js}")
        else:
            print(f"  WARNING: sortie_summary.json for {sid} not found (looked in "
                  f"{r.get('log_dir', '')!r} and next to the ledger). Ledger updated; the "
                  "final analysis reads the JSON — copy the sortie folder next to the ledger "
                  "and run this command again.")
    write_ledger(log_root, rows)
    print(f"Touchdown for {sid}: dx={dx:+.1f} dy={dy:+.1f} radial={math.hypot(dx, dy):.1f} cm saved.")


def find_sortie_json(log_dir, ledger_dir):
    """The ledger stores the absolute folder from the Pi. If the ledger was copied to
    another computer, look for the same folder name next to the ledger instead."""
    for d in (log_dir, os.path.join(ledger_dir, os.path.basename(os.path.normpath(log_dir or "")))):
        js = os.path.join(d, "sortie_summary.json")
        if d and os.path.exists(js):
            return js
    return None


def write_json_atomic(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2, default=str, ensure_ascii=False)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


# ---- statistics (numpy only; uses scipy for exact p-values if installed) ----
def _mannwhitney_p(x, y):
    try:
        from scipy.stats import mannwhitneyu
        return float(mannwhitneyu(x, y, alternative="two-sided").pvalue)
    except ImportError:
        allv = np.concatenate([x, y])
        order = allv.argsort()
        ranks = np.empty(len(allv))
        ranks[order] = np.arange(1, len(allv) + 1)
        for v in np.unique(allv):                    # average ranks for ties
            m = allv == v
            ranks[m] = ranks[m].mean()
        n1, n2 = len(x), len(y)
        n = n1 + n2
        u = ranks[:n1].sum() - n1 * (n1 + 1) / 2
        _, counts = np.unique(allv, return_counts=True)       # tie-corrected variance
        tie = float(np.sum(counts ** 3 - counts)) / (n * (n - 1)) if n > 1 else 0.0
        var = n1 * n2 / 12.0 * ((n + 1) - tie)
        if var <= 0:
            return 1.0                                        # all values identical
        z = max(0.0, abs(u - n1 * n2 / 2) - 0.5) / math.sqrt(var)
        return float(min(1.0, math.erfc(z / math.sqrt(2))))


def _cliffs(x, y):
    gt = sum((xi > yj) for xi in x for yj in y)
    lt = sum((xi < yj) for xi in x for yj in y)
    return (gt - lt) / (len(x) * len(y))


def compare_two(b, ref, seed=0):
    b = np.array([v for v in b if math.isfinite(v)])
    ref = np.array([v for v in ref if math.isfinite(v)])
    if len(b) < 2 or len(ref) < 2:
        return None
    rng = np.random.default_rng(seed)
    d = [np.median(rng.choice(b, len(b))) - np.median(rng.choice(ref, len(ref)))
         for _ in range(5000)]
    lo, hi = np.percentile(d, [2.5, 97.5])
    return dict(n=(len(b), len(ref)), diff=float(np.median(b) - np.median(ref)),
                lo=float(lo), hi=float(hi), delta=_cliffs(b, ref),
                p=_mannwhitney_p(b, ref) if min(len(b), len(ref)) >= 3 else float("nan"))


def print_comparison(log_root, experiment=None, include_invalid=False):
    rows = read_ledger(log_root)
    if not rows:
        sys.exit(f"No sorties yet in {ledger_path(log_root)}")
    exps = sorted({r.get("experiment", "E1") for r in rows})
    if experiment:
        exps = [e for e in exps if e == experiment]
        if not exps:
            sys.exit(f"No sorties for experiment {experiment} in {ledger_path(log_root)}")
    root = os.path.expanduser(log_root)
    for exp in exps:
        R = [r for r in rows if r.get("experiment", "E1") == exp]
        cams = sorted({r.get("camera") or "ir" for r in R})
        if len(cams) > 1:
            print(f"\n  WARNING experiment {exp} mixes cameras {cams}. Results from different cameras "
                  "must not be pooled — fly the RGB campaign under its own --experiment / --log-root.")
        arms = [m for m in ("constant", "tuned", "adaptive") if any(r["mode"] == m for r in R)]
        use = {m: [r for r in R if r["mode"] == m and (include_invalid or r["valid"] == "1")]
               for m in arms}

        def col(m, key):
            return [_f(r[key]) for r in use[m]]

        def cell_med(m, key, nd=1):
            v = [x for x in col(m, key) if math.isfinite(x)]
            if not v:
                return "—"
            return f"{_med(v):.{nd}f} [{_pct(v, 25):.{nd}f}–{_pct(v, 75):.{nd}f}]"

        def cell(m, fn):
            try:
                return fn(m)
            except (ValueError, ZeroDivisionError):
                return "—"

        def cep(m, q):
            v = [x for x in col(m, "td_radial_cm") if math.isfinite(x)]
            return f"{_pct(v, q):.1f}" if v else "—"

        def mean_sd(m):
            v = [x for x in col(m, "td_radial_cm") if math.isfinite(x)]
            return f"{np.mean(v):.1f} ± {np.std(v, ddof=1):.1f}" if len(v) > 1 else "—"

        def counts(m):
            allm = [r for r in R if r["mode"] == m]
            td = sum(math.isfinite(_f(r["td_radial_cm"])) for r in use[m])
            return (f"{len(allm)} / {sum(r['completed'] == '1' for r in allm)} / "
                    f"{sum(r['valid'] == '1' for r in allm)} / {td}")

        table = [
            ("Sorties flown/completed/valid/truth", lambda m: counts(m)),
            ("Touchdown CEP50 (cm)  ← primary", lambda m: cep(m, 50)),
            ("Touchdown CEP95 (cm)", lambda m: cep(m, 95)),
            ("Touchdown mean ± SD (cm)", mean_sd),
            ("Overshoot after last step (cm)", lambda m: cell_med(m, "overshoot_cm")),
            ("Settling time to 5 cm (s)", lambda m: cell_med(m, "settle_s", 2)),
            ("Hold RMS in CONVERGE (cm)", lambda m: cell_med(m, "hold_rms_cm")),
            ("σ sent in maneuver (cm)", lambda m: cell_med(m, "sigma_man_med_cm", 2)),
            ("Max tilt in maneuver (deg)", lambda m: cell_med(m, "tilt_max_deg")),
            ("Frames accepted in maneuver (%)", lambda m: cell_med(m, "man_accept_pct")),
            ("Latency capture→send (ms)", lambda m: cell_med(m, "latency_ms")),
            ("Battery at takeoff (V)", lambda m: cell_med(m, "battery_v", 2)),
        ]
        head = ["Metric (median [IQR])"] + [ARM_LABEL[m] for m in arms]
        body = [[name] + [cell(m, fn) for m in arms] for name, fn in table]

        tests = []
        if "adaptive" in arms:
            for ref in ("constant", "tuned"):
                if ref not in arms:
                    continue
                for key, label in (("td_radial_cm", "Touchdown radial (cm)"),
                                   ("overshoot_cm", "Overshoot (cm)"),
                                   ("settle_s", "Settling time (s)"),
                                   ("hold_rms_cm", "Hold RMS (cm)")):
                    c = compare_two(col("adaptive", key), col(ref, key))
                    tests.append((f"B vs {ARM_SHORT[ref]}", label, c))

        # ---------- console ----------
        widths = [max(len(str(r[i])) for r in [head] + body) for i in range(len(head))]
        line = "+".join("-" * (w + 2) for w in widths)
        print("\n" + "=" * len(line))
        print(f"  VQA-EKF COMPARISON — experiment {exp}"
              f"{'  (ALL sorties, incl. invalid)' if include_invalid else '  (valid sorties only)'}")
        print("=" * len(line))
        for i, r in enumerate([head] + body):
            print(" | ".join(str(c).ljust(widths[j]) for j, c in enumerate(r)))
            if i == 0:
                print(line)
        print("\n  Arm B compared with each reference (negative difference = B better)")
        for comp, label, c in tests:
            if c is None:
                print(f"  {comp:7s} {label:24s} not enough sorties yet")
                continue
            few = min(c["n"]) < MIN_N_FOR_TESTS
            p = "n/a" if not math.isfinite(c["p"]) else f"{c['p']:.3f}"
            print(f"  {comp:7s} {label:24s} diff {c['diff']:+6.2f}  95% CI [{c['lo']:+.2f}, {c['hi']:+.2f}]"
                  f"  Cliff's δ {c['delta']:+.2f}  p={p}  n={c['n'][0]}/{c['n'][1]}"
                  + ("  (too few — descriptive only)" if few else ""))
        excluded = [r for r in R if r["valid"] != "1"]
        if excluded and not include_invalid:
            print(f"\n  Excluded {len(excluded)} sortie(s) (aborted or failed σ check): "
                  + ", ".join(f"{r['sortie_id']}[{r['arm']}]" for r in excluded))
        try:
            import scipy  # noqa: F401
        except ImportError:
            print("  (p-values approximate: scipy not installed — pip install scipy for exact ones)")
        print("  Primary claim needs: B < A AND B < A' on touchdown, CI excluding 0, "
              f"n ≥ {MIN_N_FOR_TESTS} per arm (15 planned).")
        print("  Final paper statistics + EKF3 checks: analyze_vqa_ab.py analyze")

        # ---------- files ----------
        base = os.path.join(root, f"comparison_{exp}")
        with open(base + ".csv", "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow(head)
            w.writerows(body)
            w.writerow([])
            w.writerow(["comparison", "metric", "n_B", "n_ref", "median_diff", "ci_lo", "ci_hi",
                        "cliffs_delta", "p"])
            for comp, label, c in tests:
                if c:
                    w.writerow([comp, label, c["n"][0], c["n"][1], round(c["diff"], 3),
                                round(c["lo"], 3), round(c["hi"], 3), round(c["delta"], 3),
                                "" if not math.isfinite(c["p"]) else round(c["p"], 4)])
        with open(base + ".md", "w", encoding="utf-8") as f:
            f.write(f"# VQA-EKF comparison — experiment {exp}\n\n")
            f.write(f"Generated {datetime.datetime.now():%Y-%m-%d %H:%M}. "
                    f"{'All sorties.' if include_invalid else 'Valid sorties only.'}\n\n")
            f.write("| " + " | ".join(head) + " |\n|" + "---|" * len(head) + "\n")
            for r in body:
                f.write("| " + " | ".join(str(c) for c in r) + " |\n")
            f.write("\n| Comparison | Metric | Median diff | 95% CI | Cliff's δ | p | n (B/ref) |\n"
                    "|---|---|---|---|---|---|---|\n")
            for comp, label, c in tests:
                if c:
                    p = "n/a" if not math.isfinite(c["p"]) else f"{c['p']:.3f}"
                    f.write(f"| {comp} | {label} | {c['diff']:+.2f} | [{c['lo']:+.2f}, {c['hi']:+.2f}] "
                            f"| {c['delta']:+.2f} | {p} | {c['n'][0]}/{c['n'][1]} |\n")
        print(f"\n  Saved: {base}.csv and {base}.md")


# ═══════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description="VQA-EKF controlled flight experiment")
    ap.add_argument("--mode", choices=["constant", "tuned", "adaptive"])
    ap.add_argument("--bench", action="store_true", help="no flight; live pose read-out")
    ap.add_argument("--no-mavlink", action="store_true", help="bench without the FC")
    ap.add_argument("--sortie-id", default=datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    ap.add_argument("--sigma-tuned", type=float, default=0.017,
                    help="Arm A' sigma (m). Fix it BEFORE the campaign (see protocol).")
    ap.add_argument("--tilt-gate", type=float, default=25.0)
    ap.add_argument("--alt", type=float, default=TARGET_ALT_M)
    ap.add_argument("--no-auto-disarm", action="store_true",
                    help="v6.1a: do NOT force-disarm on touchdown (leave it to ArduPilot LAND)")
    ap.add_argument("--cam-yaw-deg", type=float, default=0.0,
                    help="image-top direction from nose, clockwise (bench test decides)")
    ap.add_argument("--axis", choices=["auto", "nose", "right"], default="auto",
                    help="maneuver axis; auto = the axis with the wider camera FOV")
    ap.add_argument("--amplitude", type=float, default=0.0, help="step size m; 0 = auto")
    ap.add_argument("--expected-tilt", type=float, default=20.0,
                    help="max tilt the steps produce (used for the FOV check)")
    ap.add_argument("--cycles", type=int, default=N_CYCLES)
    ap.add_argument("--ema-alpha", type=float, default=1.0,
                    help="1.0 = no pre-smoothing (recommended: EKF expects white noise)")
    ap.add_argument("--camera", choices=["rgb", "ir"], default=CAMERA_DEFAULT,
                    help="D455 stream: rgb = colour camera (v6 default), ir = left IR imager (v1-v5)")
    ap.add_argument("--cam-check", action="store_true",
                    help="no flight: marker detection rate over 100 frames + snapshot PNG, then exit")
    ap.add_argument("--exposure-us", type=int, default=0,
                    help="fixed exposure in microseconds (0 = auto). For rgb it is converted to the "
                         "colour sensor's 100-us steps. Use one value for the whole campaign.")
    ap.add_argument("--port", default=MAVLINK_PORT)
    ap.add_argument("--baud", type=int, default=MAVLINK_BAUD)
    ap.add_argument("--log-root", default="~/vqa_logs")
    ap.add_argument("--force", action="store_true", help="skip parameter refusal")
    ap.add_argument("--no-flow", action="store_true",
                    help="Experiment 3: optical flow OFF (EK3_SRC1_VELXY=0 required); "
                         "vision is the only horizontal aid. Fly low, pilot ready.")
    ap.add_argument("--lost-timeout", type=float, default=None,
                    help="seconds without an accepted frame before LAND "
                         "(default 5.0; 1.5 with --no-flow)")
    ap.add_argument("--experiment", default=None,
                    help="campaign label kept in the ledger (default E1; e.g. E2 = gate 40). "
                         "With --compare: show only this experiment")
    ap.add_argument("--compare", action="store_true",
                    help="print + save the A/A'/B comparison table from the ledger, then exit")
    ap.add_argument("--include-invalid", action="store_true",
                    help="with --compare: also include aborted / failed-σ-check sorties")
    ap.add_argument("--add-touchdown", nargs=3, metavar=("SORTIE_ID", "DX_CM", "DY_CM"),
                    help="add/replace the pad-grid touchdown of an earlier sortie, then exit")
    args = ap.parse_args()
    for stream in (sys.stdout, sys.stderr):          # Windows consoles: never crash on σ/δ/←
        try:
            stream.reconfigure(errors="replace")
        except (AttributeError, ValueError):
            pass

    if args.compare:
        print_comparison(args.log_root, args.experiment, args.include_invalid)
        return
    args.experiment = args.experiment or ("E3" if args.no_flow else "E1")
    if args.lost_timeout is None:
        args.lost_timeout = 1.5 if args.no_flow else MARKER_LOST_LAND_S
    if args.add_touchdown:
        sid, dx, dy = args.add_touchdown
        add_touchdown(args.log_root, sid, float(dx), float(dy))
        return
    if args.cam_check:
        self_test()
        camera_check(args)
        return
    if args.no_mavlink and not args.bench:
        ap.error("--no-mavlink only works with --bench (a flight needs the flight controller)")
    if not args.bench and args.mode is None:
        ap.error("--mode is required unless --bench")
    if args.bench and args.mode is None:
        args.mode = "adaptive"

    self_test()
    log = Logger(args.log_root, ("bench_" if args.bench else "") + args.mode, args.sortie_id)
    try:
        run(args, log)
    finally:
        log.close()


def run(args, log):
    print("=" * 60)
    print(f"  VQA-EKF EXPERIMENT {SCRIPT_VERSION} — {ARM_LABEL[args.mode]}"
          f"{'  [BENCH]' if args.bench else ''}")
    print(f"  sortie {args.sortie_id}   experiment {args.experiment}   camera {args.camera.upper()}"
          f"{'   OPTICAL FLOW OFF' if args.no_flow else ''}   logs: {log.dir}")
    print("=" * 60)

    link = None
    params = {}
    if not args.no_mavlink:
        link = Link(args.port, args.baud, require_v2=not args.bench)
        params = check_params(link, args) if not args.bench else {}
        ox, oy = params.get("VISO_POS_X"), params.get("VISO_POS_Y")
        args.cam_offset_xy = (ox, oy) if ox is not None and oy is not None else CAM_OFFSET_DEFAULT
        print(f"[PRE] landing: EKF only, straight down at {LAND_VZ_MPS:.2f} m/s to the EKF origin "
              f"(camera offset {args.cam_offset_xy[0]:.3f}, {args.cam_offset_xy[1]:.3f} m used for logging only)")
        M = mavutil.mavlink
        for mid, hz in ((M.MAVLINK_MSG_ID_ATTITUDE, 100), (M.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 30),
                        (M.MAVLINK_MSG_ID_SYS_STATUS, 2), (M.MAVLINK_MSG_ID_HEARTBEAT, 2),
                        (M.MAVLINK_MSG_ID_DISTANCE_SENSOR, 20),         # v6.1a: touchdown
                        (M.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 5),       # v6.1a: landed state
                        (M.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 5)):       # v7.3: EKF aiding status
            link.set_message_interval(mid, hz)
        link.send_origin()
        link.events_cb = log.event
    if args.no_flow and not args.bench:
        lt = time.localtime()
        hour = lt.tm_hour + lt.tm_min / 60.0
        if not E3_WINDOW_LOCAL[0] <= hour <= E3_WINDOW_LOCAL[1]:
            print(f"[PRE] WARNING: local time {lt.tm_hour:02d}:{lt.tm_min:02d} is outside the E3 window "
                  "10:00-15:30 (low sun -> marker glare -> detection gaps; with no optical flow a gap "
                  "of 1.5 s lands the drone). Fly only inside the window.")
            log.event("OUTSIDE_E3_WINDOW", local_time=f"{lt.tm_hour:02d}:{lt.tm_min:02d}")

    cam = start_camera(args.camera, args.exposure_us)
    pipe, hfov, vfov = cam["pipe"], cam["hfov"], cam["vfov"]
    print(f"[PRE] VQA model: sigma_base {VQA['sigma_base_m']:.4f} m, p {VQA['tilt_exp_p']:.2f}, "
          f"q {VQA['blur_exp_q']:.2f} (blur: {BLUR_FOR_MODEL})")
    B = cam_to_body(args.cam_yaw_deg)
    # FOV along each BODY axis (steps are flown along body axes, rotated by heading)
    img_x_body = B @ np.array([1.0, 0, 0])            # image width direction in body
    fov = {"nose": hfov if abs(img_x_body[0]) > 0.7 else vfov,
           "right": hfov if abs(img_x_body[1]) > 0.7 else vfov}
    axis = args.axis if args.axis != "auto" else max(fov, key=fov.get)
    a_max = max_step_amplitude(args.alt, fov[axis] / 2.0, args.expected_tilt)
    amp = args.amplitude if args.amplitude > 0 else round(0.9 * a_max, 2)
    print(f"[PRE] FOV nose={fov['nose']:.1f} right={fov['right']:.1f} deg -> axis '{axis}', "
          f"max safe step {a_max:.2f} m at {args.alt} m / {args.expected_tilt} deg tilt; using {amp:.2f} m")
    if not args.bench:
        plan, total = mission_plan(args.cycles)
        print(f"[PRE] Mission plan at {args.alt:.1f} m: " +
              ", ".join(f"{n} {d:g} s" for n, d in plan) + f"  -> about {total:.0f} s")
    if not args.bench and (amp <= 0.05 or amp > a_max + 1e-6):
        msg = "[PRE] Step amplitude violates the in-view constraint (marker would leave FOV)."
        if not args.force:
            try:
                pipe.stop()
            except Exception:
                pass
            sys.exit(msg + " Raise --alt, lower --expected-tilt or set --amplitude.")
        print(msg + " (--force)")

    shared = Shared()
    stop = threading.Event()          # stops vision + RX: set ONLY after disarm (or forced)
    abort = threading.Event()         # tells the flight thread to stop commanding (Ctrl-C)
    stats = PhaseStats()
    result = {}
    threads = []
    if link is not None:
        threads.append(threading.Thread(target=link.rx_loop, args=(stop,), daemon=True))
    threads.append(threading.Thread(target=vision_loop,
                                    args=(args, link, shared, log, stop, cam, B, stats),
                                    daemon=True))
    flight = None
    if not args.bench:
        flight = threading.Thread(target=flight_loop,
                                  args=(args, link, shared, log, abort, amp, axis, result),
                                  daemon=True)
        threads.append(flight)
    for t in threads:
        t.start()

    forced = supervise(args, link, log, flight, abort, stop)

    stop.set()
    for t in threads:
        t.join(timeout=3)
    try:
        pipe.stop()
    except Exception:
        pass
    if args.bench:
        return

    # final EKF position at disarm (self-reported, for interest only)
    (ex, ey, ez, *_), _ = link.get_ekf()
    with shared.lock:
        vp = shared.pose
    log.event("LANDED", ekf_x=ex, ekf_y=ey, ekf_z=ez, forced_stop=forced,
              vision_x=vp[0] if vp else None, vision_y=vp[1] if vp else None)
    result.setdefault("completed", False)
    result.update(ekf_final=(ex, ey), vision_final=vp[:2] if vp else None)
    st = stats.summary()
    ctx = dict(args=args, log=log, st=st, params=params, axis=axis, amp=amp, fov=fov,
               link=link, result=result, camera=camera_public(cam))

    # SAVE FIRST (summary + ledger row), THEN ask for the touchdown, THEN update.
    # A Ctrl-C or power cut at the prompt can no longer lose the sortie.
    summary = save_sortie(ctx, truth=None)
    row = update_ledger(ctx, summary, quiet=True)
    truth = ask_touchdown()
    if truth:
        summary = save_sortie(ctx, truth=truth)
        row = update_ledger(ctx, summary, quiet=True)
    print_sortie_summary(ctx, truth, row)


def supervise(args, link, log, flight, abort, stop):
    """Main-thread supervisor. Returns True if vision was force-stopped before disarm.
       Ctrl-C #1: abort the sequence -> LAND; vision keeps feeding EKF3 until DISARM.
       Ctrl-C #2: force-stop vision (pilot must land manually)."""
    if args.bench:
        try:
            while not stop.is_set():
                stop.wait(0.5)
        except KeyboardInterrupt:
            print("\n[MAIN] bench stopped")
        return False
    try:
        while flight.is_alive():
            flight.join(0.5)
    except KeyboardInterrupt:
        print("\n[MAIN] Ctrl-C — aborting the sequence and commanding LAND.")
        print("[MAIN] Vision keeps feeding EKF3 until DISARM.  Ctrl-C again = force stop.")
        log.event("CTRL_C")
        abort.set()
        try:
            if link.get_status()[1] == MODE_GUIDED:
                link.set_mode(MODE_LAND)
        except Exception:
            pass
    try:
        flight.join(15)
        return not wait_for_disarm(link, log)
    except KeyboardInterrupt:
        print("\n[MAIN] FORCE STOP — vision stopping NOW. Pilot: land manually.")
        log.event("FORCE_STOP")
        return True


def wait_for_disarm(link, log, report_every_s=5.0, sleep_s=0.05):
    """Block (vision still running) until the FC reports DISARMED. Returns True then.
    v6.1a: while the SCRIPT is landing the drone (link.auto_disarm, mode LAND/GUIDED - never
    after a pilot takeover), a confirmed touchdown triggers an immediate forced disarm."""
    t_last, td, sent = 0.0, TouchdownWatch(link), False
    while True:
        armed, mode, _ = link.get_status()
        if not armed and link.heartbeat_age() < HB_TIMEOUT_S:
            log.event("DISARMED")
            print("[MAIN] DISARMED — stopping vision.")
            return True
        if (getattr(link, "auto_disarm", False) and not sent
                and mode in (MODE_LAND, MODE_GUIDED) and td.update()):
            sent = True
            force_disarm(link, log, "LAND")
            continue
        if now() - t_last >= report_every_s:
            (_, _, ez, *_), _ = link.get_ekf()
            hb = link.heartbeat_age()
            print(f"[MAIN] waiting for DISARM — mode {mode}, height {-ez:.2f} m"
                  f"{'' if hb < HB_TIMEOUT_S else f', NO HEARTBEAT {hb:.0f} s'}"
                  " — vision still feeding EKF3 (Ctrl-C = force stop)")
            t_last = now()
        time.sleep(sleep_s)


def save_sortie(ctx, truth):
    args, log, st, link, result = ctx["args"], ctx["log"], ctx["st"], ctx["link"], ctx["result"]
    log.flush_data()
    summary = dict(
        version=SCRIPT_VERSION, sortie_id=args.sortie_id, mode=args.mode,
        arm=ARM_LABEL[args.mode], bench=args.bench, args=vars(args),
        script_sha256=script_sha256(), time_base="monotonic, offset to epoch seconds",
        mission_plan_s=dict(mission_plan(getattr(args, "cycles", N_CYCLES))[0]),
        model=VQA, blur_for_model=BLUR_FOR_MODEL, sigma_default_m=SIGMA_DEFAULT_M,
        sigma_floor_m=SIGMA_FLOOR_M, smooth_frames=SMOOTH_FRAMES, fc_params=ctx["params"],
        axis=ctx["axis"], amplitude_m=ctx["amp"], fov_deg=ctx["fov"], camera=ctx.get("camera"),
        fc_clock_offset_s=link.clock_off if link else None,
        fc_rx_errors=link.rx_errors if link else None,
        phase_stats=st, events=log.events, completed=result.get("completed"),
        abort_reason=result.get("abort_reason", ""),
        ekf_final_m=result.get("ekf_final"), vision_final_m=result.get("vision_final"),
        ekf_flags_at_start=result.get("ekf_flags_at_start"),
        touchdown_truth=truth)
    write_json_atomic(os.path.join(log.dir, "sortie_summary.json"), summary)
    return summary


def update_ledger(ctx, summary, quiet=False):
    args, log = ctx["args"], ctx["log"]
    try:
        row = build_ledger_row(summary, os.path.join(log.dir, "vision_frames.csv"))
        replaced = upsert_ledger(args.log_root, row)
        row["_replaced"] = replaced
        return row
    except Exception as e:                           # never lose the sortie over the ledger
        print(f"  LEDGER  could not update ({e}); sortie files are intact")
        return None


def print_sortie_summary(ctx, truth, row):
    args, log, st, result = ctx["args"], ctx["log"], ctx["st"], ctx["result"]
    ph = st.get("MANEUVER", {})
    lat = [v for s_ in st.values() if "latency_ms" in s_ for v in [s_["latency_ms"]["median"]]]

    def ss(phase, key, scale=1.0, unit=""):
        s_ = st.get(phase, {}).get(key)
        return "n/a" if not s_ else f"{s_['median']*scale:.1f}{unit} [{s_['p05']*scale:.1f}-{s_['p95']*scale:.1f}]"

    print("\n" + "=" * 60)
    print(f"  SORTIE SUMMARY - {ARM_LABEL[args.mode]}   ({args.sortie_id})")
    print("=" * 60)
    print(f"  1 Manipulation  sigma sent HOVER_BASE {ss('HOVER_BASE','sigma_sent',100,' cm')}")
    print(f"                  sigma sent MANEUVER   {ss('MANEUVER','sigma_sent',100,' cm')}")
    print("                  (A: flat 20 cm | A': flat tuned | B: must vary with tilt/blur;")
    print("                   confirm with VISP.PErr in the .bin)")
    print(f"  2 Maneuver      tilt {ss('MANEUVER','tilt_deg',1,' deg')}   max "
          f"{ph.get('tilt_deg',{}).get('max',float('nan')):.1f} deg   "
          f"accepted {ph.get('accepted',0)}/{ph.get('frames',0)} frames, "
          f"tilt-gated {ph.get('tilt_gated',0)}")
    print(f"  3 Pipeline      latency capture->send median {np.median(lat) if lat else float('nan'):.1f} ms"
          f"  (VISO_DELAY_MS should ~ this + camera/UART)")
    if result.get("ekf_final"):
        ex, ey = result["ekf_final"]
        print(f"  4 EKF at disarm  N={ex:+.3f} E={ey:+.3f} m  (self-reported - NOT ground truth)")
    if truth:
        print(f"  5 TRUTH touchdown  dx={truth['dx_cm']:+.1f} dy={truth['dy_cm']:+.1f} "
              f"radial={truth['radial_cm']:.1f} cm   <-- primary outcome")
    else:
        print(f"  5 TRUTH touchdown  not entered - later: --add-touchdown {args.sortie_id} DX_CM DY_CM")
    print(f"  completed={result.get('completed')}"
          f"{'' if result.get('completed') else '  reason=' + str(result.get('abort_reason'))}"
          f"   logs: {log.dir}")
    if row:
        n_arm = sum(r["mode"] == args.mode and r.get("experiment") == args.experiment
                    for r in read_ledger(args.log_root))
        print(f"  LEDGER  {'replaced' if row.get('_replaced') else 'added'} {args.sortie_id} "
              f"[{ARM_SHORT[args.mode]}] valid={row['valid']} "
              f"(sigma check {'OK' if row['manip_ok'] else 'FAILED'}) - "
              f"{n_arm} sortie(s) of this arm in {args.experiment}")
        if args.mode == "adaptive":
            rt = _f(row.get("sigma_man_ratio"))
            if math.isfinite(rt) and rt < ADAPT_WEAK_RATIO:
                print(f"  NOTE    B's sigma barely varied in the maneuver (x{rt:.3f}); max tilt "
                      f"{row.get('tilt_max_deg')} deg. The arm worked, but the maneuver was gentle —")
                print("          in the pilot phase raise WPNAV_ACCEL toward 15-22 deg of tilt.")
            if _f(row.get("sigma_pinned_frac")) >= 0.5:
                print("  WARNING B's sigma was stuck at its 1 cm / 50 cm limit on most frames — check the")
                print("          VQA constants and BLUR_FOR_MODEL (M3) before flying B again.")
        print(f"          table any time:  python3 {os.path.basename(sys.argv[0])} --compare")
    print("  NEXT: copy this flight's DataFlash .bin into the folder above.")
    print("=" * 60)


# ═══════════════════════════════════════════════════════════════════════════
# CHANGES vs v1 (kept here so the paper's methods section can cite them)
#  C1  Every arm sends an explicit covariance encoded for ArduPilot's cube-root
#      decoding; VISO_POS_M_NSE <= 0.01 is checked at start-up. In v1 every value
#      was clamped to 0.20 m, so A and B were identical inside EKF3.
#  C2  Only the XY sigma is encoded (POSZ comes from the rangefinder); v1 mixed
#      the scale-dependent var_z into the single scalar ArduPilot uses.
#  C3  2.0 m sweep replaced by +/-A steps sized so the marker stays in view.
#  C4  Touchdown truth entered from the pad grid; EKF offsets labelled as self-report.
#  M1  Only accepted frames are sent (v1 re-sent stale pose after outlier rejects).
#  M2  One message per accepted frame, stamped with camera capture time.
#  M3  Roll/pitch interpolated to the frame capture time (FC clock aligned).
#  M4  Yaw measured from the marker (v1 echoed the FC's own yaw back).
#  M5  Both blur metrics logged; model uses BLUR_FOR_MODEL (must match the fit).
#  --  VISION_SPEED_ESTIMATE removed; origin = marker centre (no bench clamp);
#      IPPE ambiguity resolved with the IMU; EMA pre-filter off by default.
# v3 (campaign ledger + peer-review fixes):
#  P1  attitude_at works on one locked snapshot (v2 race could kill the vision thread).
#  P2  A failure on one frame is logged and skipped; RX and flight threads also guarded.
#  P3  Vision runs until DISARM (Ctrl-C #1 = LAND, #2 = force stop; pilot takeover safe).
#  P4  Summary + ledger saved BEFORE the touchdown prompt; Ctrl-C at the prompt is safe.
#  P5  --no-mavlink only with --bench.     P6  MAVLink 2 verified on both sides.
#  P7  GUIDED accepted only when airborne; takeoff/settle timeouts abort the sortie.
#  P8  Jump check also after short dropouts; re-seed only after > REACQ_GAP_S.
#  P9  14 parameter checks (adds AHRS_EKF_TYPE, VISO_SCALE, VISO_QUAL_MIN, flow...).
#  P10 Monotonic time base for all timeouts and timestamps.  P11 UTF-8 file I/O.
#  P12 Fallback p-value tie-corrected and capped at 1.
#  P13 Overshoot/settling from a 0.5 s rolling median (noise no longer dominates).
#  P14 Steps flown along the drone's own axis (rotated by heading at GUIDED).
#  P15-17 version label; --add-touchdown messages; JSON found next to a copied ledger.
# v5:
#  Flight altitude 2.0 m (TARGET_ALT_M). 2-minute mission: takeoff <= 8 s, settle <= 7 s,
#  hover 30 s, maneuver 11 x 4 s, converge 15 s, descent <= 8 s + 2 s, land ~5 s.
#  Plan printed at start-up and saved in sortie_summary.json (mission_plan_s).
#  Fix: an abort during the final 2 s descent hold now marks the sortie incomplete
#  (v3 ignored it and could report completed=True with an abort reason).
#  Fix: Arm B is valid when every frame sent exactly the model's sigma and sigma is not
#  pinned at a limit. v3/v4 required >5 % variation, which discarded correctly working
#  B sorties whenever the maneuver was gentle. Weak variation is now a printed NOTE.
# v6:
#  R1  D455 colour camera by default (--camera rgb, 640x480@30, BGR8 -> grey). Its IR-cut
#      filter hides the MTF-01 infrared ToF light that washed out IR1 (detections fell to
#      ~14 % on the pad -> "VisOdom: not healthy"). --camera ir keeps the v5 path.
#  R2  Lens distortion: corners are undistorted with the exact RealSense model (incl. the
#      colour camera's inverse Brown-Conrady, which OpenCV lacks); PnP runs distortion-free.
#      Self-test checks the round trip at start-up.
#  R3  Colour exposure: --exposure-us converted to the sensor's 100-us steps and clamped to
#      its range; auto_exposure_priority = 0 so dim light cannot drop the frame rate.
#  R4  --cam-check: detection rate over 100 frames + snapshot PNG (target >= 90 %).
#  R5  Camera kind, lens model and intrinsics saved in sortie_summary.json; "camera" and
#      "version" columns in the ledger; --compare warns if an experiment mixes cameras.
#  R6  GUIDED refused / sortie aborted when FC attitude is older than 200 ms (tilt would be 0).
#  R7  MAVLINK_PORT = /dev/ttyAMA2 (Pi 5 UART2, pins 7/29) — GPIO14 TX on pin 8 is faulty.
#  NOTE the VQA sigma constants were fitted on IR1 images: refit on RGB before the campaign.
# v6.1:
#  R8  Link waits for the flight controller's OWN heartbeat (component 1, real autopilot)
#      and lists other devices it skipped. v6.0 used the first heartbeat of any device, so a
#      forwarded MTF-01 heartbeat (MAVLink 1) randomly triggered "MAVLink 2 required".
#      Clear error after 30 s with no FC heartbeat.
# v6.1a (built on v6.1; v6.2/v6.3 were not adopted):
#  A1  GUIDED accepted at any height (the >= 0.5 m rule is removed). If EXTENDED_SYS_STATE
#      says ON_GROUND (or is unknown) the script counts down 2 s and sends MAV_CMD_NAV_TAKEOFF,
#      because ArduCopter ignores position setpoints while landed; if the FC already counts as
#      flying (throttle raised in STABILIZE -> NAV_TAKEOFF fails with result 4, log 23) it
#      climbs with setpoints directly. Lift-off = EKF height +0.15 m within 8 s, else abort.
#  A2  Touchdown auto-disarm: ArduCopter refuses a normal MAVLink disarm until it detects the
#      landing itself (AP_Arming_Copter::disarm), which it never does while > 15 deg lean is
#      demanded on the ground (land_detector LAND_CHECK_LARGE_ANGLE_CD, log 16) -> skid.
#      While the script lands the drone, DISTANCE_SENSOR <= 0.25 m (landed: 0.19-0.21 m) and
#      EKF |vz| <= 0.15 m/s for 0.3 s -> forced disarm (param2 = 21196) at once, in the GUIDED
#      descent or in LAND. Not after a pilot takeover; --no-auto-disarm turns it off.
#      (The 19 cm sensor height itself does not block ArduPilot's landing detection: it only
#      needs the rangefinder < 2 m, LAND_RANGEFINDER_MIN_ALT_CM.)
# v6.1b (from T11 / log 29):
#  B1  Yaw: the yaw stick rested at 1560-1567 us (RC4_TRIM 1515, RC4_DZ 20) and GUIDED obeys
#      pilot yaw by default -> 11.7 deg/s, 1207 deg in 103 s; the drone followed that desired
#      yaw within 2.9 deg rms, so it was a command, not an EKF/vision fault. Pre-flight now
#      requires GUID_OPTIONS bit 2 (+4 IgnorePilotYaw). LAND has no such option, so the
#      landing is now finished in GUIDED (B2) and LAND is only the fallback.
#  B2  Landing: the fixed DESCENT target EKF z -0.40 m assumed the EKF reads -0.40 on the pad
#      (logs 15-24); in log 29 it read -0.22, so the drone stopped 18 cm above the pad, lost
#      the marker (camera blind below ~0.14 m), and hovered 6-14 cm up for ~2.5 s with flow
#      innovations rejected (0.5-3.3 rad/s) -> EKF "stationary" while it slid 60 cm. Now:
#      pad EKF z = median(EKF z + rangefinder - rangefinder-on-pad) over 1 s at hover
#      (fallback: EKF z on the pad at take-off; else the old fixed value + LAND); descend to
#      0.40 m above the pad + 2 s hold (vision still good), then FINAL: setpoint ramps down at
#      0.30 m/s to 0.20 m below the pad without stopping -> blind for ~0.5 s instead of ~3 s;
#      touchdown -> forced disarm in GUIDED. No touchdown 3 s after the ramp -> LAND.
#  B3  TouchdownWatch hardened: MTF-01 drop-outs (0.00-0.02 m, up to 0.4 s in flight, log 29)
#      are ignored (< 0.10 m), and the EKF height must be within 0.20 m of the pad, so a
#      drop-out during a hover hold can no longer force-disarm in the air.
# v6.1c (from T11 re-fly with v6.1b: radial 26 cm; the operator wants a marker-controlled
#      landing all the way down):
#  C1  MarkerLanding replaces DESCENT-to-gate + FINAL ramp. 20 Hz: CG position measured from
#      the MARKER (camera pose - VISO_POS_X/Y rotated by heading; log 29 check: CG within
#      2.8 cm median in the descent, camera itself 13 cm off because it is 11 cm ahead).
#      Descends at 0.20 m/s only while the CG is within 5 cm; otherwise freezes the height and
#      re-centres (0.3 s centred to resume; creeps at 0.10 m/s if not centred within 5 s).
#      Horizontal target = EKF-minus-marker offset (1 s low-pass, <= 10 cm), so an EKF bias
#      or lane-switch step cannot move the landing point away from the marker.
#  C2  Marker lost below 20 cm above the pad (camera 11 cm ahead of the CG cannot see the
#      marker below ~14 cm): straight down holding the last target -> touchdown -> disarm.
#      Lost above 20 cm: hold 1 s, climb <= 0.5 m at 0.15 m/s to find it; 6 s -> straight
#      down, LANDING_MARKER_NOT_FOUND, ledger valid = 0.
#  C3  Height for these decisions = rangefinder - rangefinder-on-pad (EKF height vs measured
#      pad if the rangefinder is missing or implausibly low). 1 Hz [LND] status line.
# v7 (from v6.1c):
#  L1  EKF-only landing replaces MarkerLanding. v6.1c steered the landing with the marker
#      (EKF-minus-marker correction, descend only while within 5 cm), which cancels the EKF
#      error before touchdown: in SITL an 8 x 6 cm EKF bias still landed 0.1 cm from the
#      marker, so touchdown could no longer tell the arms apart (the primary outcome).
#      v7: after the pad reference, the setpoint goes straight down at 0.20 m/s to EKF (0, 0),
#      0.20 m below the pad; touchdown -> forced disarm; no touchdown 3 s after the setpoint
#      reached the bottom (or 60 s in total) -> ArduPilot LAND. No marker feedback, no pauses,
#      no re-finding: identical in every arm. Marker loss in the landing is only logged.
#  L2  LANDING_END logs where the camera last saw the CG (last_fix_cg_x/y, last_fix_h), the
#      EKF at that moment and the blind time, so touchdown error = EKF error at the last fix
#      + blind drift can be separated in the analysis. Logging only — never fed back.
#  Everything else is unchanged from v6.1c (take-off, mission, pad reference, auto-disarm).
# v7.1 (robustness only — flight profile, landing and all numbers of the experiment unchanged):
#  F1  No forced disarm after a pilot takeover. v7 left link.auto_disarm on when the pilot
#      took over during the landing, so a pilot LAND-switch landing was force-disarmed.
#  F2  Rangefinder (DISTANCE_SENSOR / RANGEFINDER) accepted only from the flight controller
#      (component 1). The MTF-01's own message forwarded from TELEM2 could mix in.
#  F3  In flight, "attitude_lost" only after the FC attitude has been stale (> 200 ms) for
#      0.5 s continuously; one short serial gap no longer aborts a sortie. GUIDED start still
#      needs attitude younger than 200 ms.
# v7.2 (user): v7.1 + VQA model refitted on RGB data (sigma_base 0.0226 m, p 5.25, q 0.69).
#      Flown for the whole E1 campaign (45 sorties).
# v7.3 (Experiment 3 support; E1 profile, landing and per-arm sigma unchanged):
#  E1  EKF_STATUS_REPORT requested at 5 Hz and logged (EKF_FLAGS events on every change,
#      flags at the GUIDED start in the summary) in every experiment - logging only.
#  E2  --no-flow: take-off only after the EKF reports absolute horizontal position
#      (POS_HORIZ_ABS set, CONST_POS_MODE clear) for 2 s. Without flow, EKF3 has no other
#      horizontal aid (AP_NavEKF3_Control.cpp updateFilterStatus).
#  E3  --no-flow: in flight, abort -> LAND if that status is lost for 1 s ("ekf_no_position").
#  E4  --no-flow: warning + event if started outside 10:00-15:30 local (E1: low-sun glare on
#      the semi-gloss marker cut detection from 95 % to 77 %).
#  E5  Removed the v6 start-up note about IR-fitted constants; prints the model in use.
# ═══════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    main()
