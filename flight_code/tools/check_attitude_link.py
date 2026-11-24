#!/usr/bin/env python3
"""
check_attitude_link.py — is the flight controller's attitude reaching the VQA script?

Safe: it only LISTENS and asks for message rates. It never arms, changes mode or
sends vision. Uses the flight script's own Link class, so it tests the exact code path.

    python3 check_attitude_link.py                       (defaults: /dev/ttyAMA0, 921600)
    python3 check_attitude_link.py --script vqa_ekf_ab_test_v4.py --port /dev/ttyAMA0

While it runs (15 s), TILT THE DRONE BY HAND about 15-20 deg, then put it back level.
Stop the flight script first — only one program may use the serial port.
"""
import argparse
import collections
import importlib.util
import math
import os
import shutil
import subprocess
import sys
import threading
import time

RESULT = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED", 3: "UNSUPPORTED", 4: "FAILED",
          5: "IN_PROGRESS", 6: "CANCELLED"}


def load_script(path):
    spec = importlib.util.spec_from_file_location("vqa_flight", path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--script", default=os.path.join(here, "vqa_ekf_ab_test_v3.py"))
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=15.0)
    a = ap.parse_args()

    # 0. Is another program holding the serial port? (IRoC services, mavproxy, mavlink-router…)
    if a.port.startswith("/dev/") and shutil.which("fuser"):
        r = subprocess.run(["fuser", "-v", a.port], capture_output=True, text=True)
        users = (r.stdout + r.stderr).strip()
        if r.returncode == 0 and users:
            print(f"[0] WARNING: another program has {a.port} open — two readers split the bytes:\n{users}\n"
                  "    Stop it (e.g. sudo systemctl stop <service>) and run this again.\n")
        else:
            print(f"[0] Nothing else holds {a.port}: OK")

    v = load_script(a.script)
    print(f"[1] Using {os.path.basename(a.script)} ({getattr(v, 'SCRIPT_VERSION', '?')})")
    link = v.Link(a.port, a.baud, require_v2=False)

    counts = collections.Counter()
    acks = {}
    orig = link.handle

    def counting_handle(msg):
        t = msg.get_type()
        counts[t] += 1
        if t == "COMMAND_ACK":
            acks[msg.command] = msg.result
        orig(msg)
    link.handle = counting_handle

    stop = threading.Event()
    threading.Thread(target=link.rx_loop, args=(stop,), daemon=True).start()

    # 2. What arrives with no request at all (what the FC already streams on this port)
    time.sleep(3.0)
    base = dict(counts)
    print(f"[2] Already streaming on this port (3 s): ATTITUDE {base.get('ATTITUDE', 0) / 3:.0f} Hz, "
          f"LOCAL_POSITION_NED {base.get('LOCAL_POSITION_NED', 0) / 3:.0f} Hz, "
          f"HEARTBEAT {base.get('HEARTBEAT', 0) / 3:.1f} Hz")

    # 3. The same requests the flight script makes
    M = v.mavutil.mavlink
    for mid, hz in ((M.MAVLINK_MSG_ID_ATTITUDE, 100), (M.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 30)):
        link.set_message_interval(mid, hz)
    time.sleep(1.0)
    r = acks.get(M.MAV_CMD_SET_MESSAGE_INTERVAL)
    print(f"[3] SET_MESSAGE_INTERVAL reply from FC: {RESULT.get(r, 'NO REPLY') if r is not None else 'NO REPLY'}")

    # 4. Live: rates + the script's own attitude lookup. Tilt the drone now.
    print(f"[4] TILT THE DRONE NOW (about 15-20 deg), then level again — {a.seconds:.0f} s")
    c0 = dict(counts)
    t0 = time.time()
    tilts = []
    while time.time() - t0 < a.seconds:
        time.sleep(0.5)
        roll, pitch, age = link.attitude_at(v.now())
        tilt = v.true_tilt_deg(roll, pitch)
        tilts.append(tilt)
        with link.lock:
            n_att, off = len(link.att), link.clock_off
        print(f"    roll {math.degrees(roll):+6.1f}  pitch {math.degrees(pitch):+6.1f}  tilt {tilt:5.1f} deg"
              f"   samples={n_att:3d}  age={age if math.isfinite(age) else float('inf'):.0f} ms"
              f"   clock_off={'None' if off is None else 'ok'}")
    dt = time.time() - t0
    att_hz = (counts["ATTITUDE"] - c0.get("ATTITUDE", 0)) / dt
    pos_hz = (counts["LOCAL_POSITION_NED"] - c0.get("LOCAL_POSITION_NED", 0)) / dt
    err = getattr(link.m.mav, "total_receive_errors", 0)
    ok_pk = getattr(link.m.mav, "total_packets_received", 0)
    stop.set()

    print("\n" + "=" * 70)
    print(f"  ATTITUDE {att_hz:.0f} Hz   LOCAL_POSITION_NED {pos_hz:.0f} Hz   "
          f"max tilt seen {max(tilts):.1f} deg   bad packets {err} / good {ok_pk}   RX errors {link.rx_errors}")
    print("=" * 70)
    if ok_pk and err > 0.05 * ok_pk:
        print("  VERDICT: many corrupted packets -> another program is reading the same serial port,\n"
              "           or wrong baud. Stop other MAVLink services; check SERIALx_BAUD = 921.")
    elif att_hz < 5:
        print("  VERDICT: the FC is NOT sending ATTITUDE to the Pi.")
        print("  FIX (Mission Planner -> Full Parameter List), for the Pi's MAVLink channel x:")
        print("      SRx_EXTRA1 = 50     (ATTITUDE)      SRx_POSITION = 30   (LOCAL_POSITION_NED)")
        print("      SRx_EXT_STAT = 2    (SYS_STATUS)    -> Write, reboot, run this check again.")
        print("  x is the MAVLink channel, counted over ports with SERIALn_PROTOCOL = 1 or 2 in order")
        print("  (USB = 0). Pi on TELEM1 with TELEM2 not MAVLink -> SR1. If unsure, set SR1_ and SR2_.")
    elif max(tilts) < 3:
        print("  VERDICT: ATTITUDE arrives, but tilt never exceeded 3 deg.")
        print("  Did you tilt the drone? If yes, the Cube may not be the one on this airframe, or the")
        print("  AHRS isn't running (check HUD in Mission Planner while tilting).")
    else:
        print("  VERDICT: PASS — attitude reaches the script's own Link code and follows the tilt.")
        print("  Re-run the bench test: the tilt column must now follow your hand.")
    print("=" * 70)


if __name__ == "__main__":
    main()
