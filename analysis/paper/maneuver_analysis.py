"""Additional (exploratory) analysis of the 44 s manoeuvre phase and of in-flight reliability.

These measures were not among the pre-specified hypotheses; they are reported as additional analyses (Section 5.4 of the article).
With the camera as the position reference (frame-to-frame noise 1.55 mm), for every valid sortie:
  * waypoint error   - median distance of the controlled point from the commanded target in the last 1 s of each step
  * overshoot        - median along-track excursion past the target, over the steps
  * cross-track RMS  - deviation perpendicular to the step axis over the steps (corridor keeping)
  * EKF-vision RMS   - EKF3 position estimate, time-aligned to the image capture, minus the camera-derived position
  * step response    - along-track progress of the full (0.94 m) steps, normalised to the step length
and, from the DataFlash log, the EKF3 position-innovation RMS inside the manoeuvre window and the in-flight EKF warnings.
The controlled point is cg = camera - R(yaw) * offset (analyzer add_cg_columns), smoothed with the analyzer's
0.5 s running median. The first step (0 -> -A) and the return step (-> 0) are excluded from the step response;
the return step is excluded from all step measures (it is the pre-specified overshoot outcome of the analyzer).

Usage: maneuver_analysis.py --e1 RESULTS_E1 --e3 RESULTS_E3 --e1-logs vqa_paper_E1 --e3-logs vqa_paper_E3 --out DIR
(RESULTS_* are frozen analyzer folders with per_sortie.csv, bin_match.csv and _bin_cache/.)
Writes maneuver_per_sortie_E1/E3.csv, maneuver_numbers.json, reliability.json and Fig9.pdf/.png.
"""
import argparse, glob, json, math, os, sys
import numpy as np, pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
import analyze_vqa_ekf_final as AZ  # noqa: E402
import make_figures_ais as FG       # noqa: E402  (journal figure style, colours, markers)

ARMS = ('constant', 'tuned', 'adaptive')
TAIL_S = 1.0
TGRID = np.arange(0.0, 4.0001, 0.05)
SEED = 20261128
METRICS = ['wp_err_cm', 'overshoot_step_cm', 'crosstrack_rms_cm', 'ekf_vis_man_rms_cm', 'innov_man_rms_cm']


def sortie_dir(logs, sid):
    d = glob.glob(os.path.join(logs, f'*_{sid}'))
    return d[0] if d else None


def companion_metrics(d):
    s = json.load(open(os.path.join(d, 'sortie_summary.json'), encoding='utf-8'))
    df = AZ.add_cg_columns(s, AZ.load_frames(d))
    steps = [e for e in s['events'] if e['event'] == 'STEP']
    t_conv = AZ.event_t(s, 'CONVERGE')
    if len(steps) < 3 or t_conv is None:
        return {}, None
    vis = df[(df.detected == 1) & np.isfinite(df.cg_x) & np.isfinite(df.cg_y)].sort_values('t').copy()
    vis['sx'], vis['sy'] = AZ.rolling_median(vis.cg_x.to_numpy()), AZ.rolling_median(vis.cg_y.to_numpy())
    bounds = [e['t'] for e in steps] + [t_conv]
    wp, ov, xt, curves = [], [], [], []
    prev = (0.0, 0.0)
    for i, e in enumerate(steps):
        tgt = (float(e.get('target_x', 0.0)), float(e.get('target_y', 0.0)))
        t0, t1 = bounds[i], bounds[i + 1]
        w = vis[(vis.t >= t0) & (vis.t < t1)]
        u = AZ.unit(tgt[0] - prev[0], tgt[1] - prev[1])
        n = (-u[1], u[0])
        if len(w) > 10 and i < len(steps) - 1:                      # steps 0..9; step 10 returns to the origin
            ex, ey = w.sx - tgt[0], w.sy - tgt[1]
            tail = w.t >= t1 - TAIL_S
            wp.append(float(np.median(np.hypot(ex[tail], ey[tail]))))
            ov.append(max(0.0, float(np.max(ex * u[0] + ey * u[1]))))
            xt.append((ex * n[0] + ey * n[1]).to_numpy())
            if i >= 1:                                                # full +/-A -> -/+A steps
                L = math.hypot(tgt[0] - prev[0], tgt[1] - prev[1])
                prog = ((w.sx - prev[0]) * u[0] + (w.sy - prev[1]) * u[1]) / L
                curves.append(np.interp(TGRID, (w.t - t0).to_numpy(), prog.to_numpy()))
        prev = tgt
    m = {}
    if wp:
        m['wp_err_cm'] = float(np.median(wp)) * 100
        m['overshoot_step_cm'] = float(np.median(ov)) * 100
        m['crosstrack_rms_cm'] = float(np.sqrt(np.mean(np.concatenate(xt) ** 2))) * 100
    man = df[(df.t >= steps[0]['t']) & (df.t < t_conv)].copy()
    t_ekf = (man.t + man.latency_ms.fillna(man.latency_ms.median()) / 1000.0).to_numpy()
    o = np.argsort(t_ekf)
    ok = man.ekf_x.notna().to_numpy()[o]
    te, ex_, ey_ = t_ekf[o][ok], man.ekf_x.to_numpy()[o][ok], man.ekf_y.to_numpy()[o][ok]
    mv = man[(man.detected == 1) & np.isfinite(man.cg_x)]
    if len(te) > 10 and len(mv) > 10:
        ix, iy = np.interp(mv.t, te, ex_), np.interp(mv.t, te, ey_)
        m['ekf_vis_man_rms_cm'] = float(np.sqrt(np.mean((ix - mv.cg_x) ** 2 + (iy - mv.cg_y) ** 2))) * 100
        m['frame_interval_ms'] = float(np.median(np.diff(np.sort(man.t.to_numpy())))) * 1000
    curve = np.nanmedian(np.vstack(curves), axis=0) if curves else None
    return m, curve


def bin_metrics_man(d):
    """EKF3 position-innovation RMS inside the manoeuvre window (primary lane, analyzer windowing),
    and EKF warnings logged between arming and disarming."""
    k3 = AZ.arr(d, 'XKF3', 4); k4 = AZ.arr(d, 'XKF4', 9); gu = AZ.arr(d, 'GUIP', 3)
    r = {}
    if len(k3) and len(gu):
        k4p = k4[k4[:, 1] == k4[:, 8]] if len(k4) else k4
        lane = int(np.round(np.median(k4p[:, 8]))) if len(k4p) else 0
        k3p = k3[k3[:, 1] == lane]
        steps = [gu[i, 0] for i in range(1, len(gu)) if math.hypot(gu[i, 1] - gu[i - 1, 1], gu[i, 2] - gu[i - 1, 2]) > 20]
        if steps:
            w = k3p[(k3p[:, 0] > steps[0]) & (k3p[:, 0] < steps[-1] + 4.0)]
            if len(w):
                r['innov_man_rms_cm'] = float(np.sqrt(np.mean(w[:, 2] ** 2 + w[:, 3] ** 2))) * 100
    ev = d.get('EV', [])
    msgs = d.get('MSG', [])
    # logging starts at arming (the ARMED event itself may precede the log): flight window = log start .. DISARMED
    t_arm = next((e[0] for e in ev if int(e[1]) in (10, 15, 28)), msgs[0][0] if msgs else None)
    t_arm = min(t_arm, msgs[0][0]) if (msgs and t_arm is not None) else t_arm
    t_dis = next((e[0] for e in ev if int(e[1]) == 11 and (t_arm is None or e[0] > t_arm)), None)
    keys = ('EKF variance', 'EKF Failsafe', 'lane switch', 'stopped aiding')
    warn = [(m[0], m[1]) for m in msgs if any(k in m[1] for k in keys)]
    if t_arm is not None and t_dis is not None:
        r['ekf_warn_in_flight'] = int(sum(t_arm <= t <= t_dis for t, _ in warn))
        post = [t - t_dis for t, _ in warn if t > t_dis]
        r['ekf_warn_after_disarm'] = int(len(post))
        r['ekf_warn_after_disarm_s'] = float(min(post)) if post else np.nan
    return r


def campaign(results, logs):
    per = pd.read_csv(os.path.join(results, 'per_sortie.csv'))
    per = per[per.valid == True]
    bm = pd.read_csv(os.path.join(results, 'bin_match.csv')).set_index('sortie_id')
    cache = {}
    for f in glob.glob(os.path.join(results, '_bin_cache', '*.json')):
        d = json.load(open(f, encoding='utf-8')); cache[d.get('_file')] = d
    rows, curves = [], {a: [] for a in ARMS}
    for r in per.itertuples():
        m = {'sortie_id': r.sortie_id, 'mode': r.mode}
        d = sortie_dir(logs, r.sortie_id)
        if d:
            cm, cv = companion_metrics(d)
            m.update(cm)
            if cv is not None:
                curves[r.mode].append(cv)
        if r.sortie_id in bm.index and bm.loc[r.sortie_id, 'bin'] in cache:
            m.update(bin_metrics_man(cache[bm.loc[r.sortie_id, 'bin']]))
        rows.append(m)
    df = pd.DataFrame(rows).merge(per[['sortie_id', 'lane_switches', 'relpos_lost_samples', 'maneuver_tilt_gated',
                                       'rng_zero_readings', 'landing_result']], on='sortie_id', how='left')
    return df, {a: np.vstack(v) for a, v in curves.items() if v}


def fig9(res, curves, out):
    FG.style()
    fig, axes = FG.plt.subplots(2, 2, figsize=(FG.TW, 4.3), gridspec_kw=dict(wspace=0.32, hspace=0.42))
    for ax, exp, k in ((axes[0, 0], 'E1', 'a'), (axes[0, 1], 'E3', 'b')):
        ax.axhline(100.0, color=FG.INK2, lw=0.6, ls=':')
        for m in ARMS:
            c = curves[exp][m] * 100
            med, lo, hi = np.nanmedian(c, 0), np.nanpercentile(c, 25, 0), np.nanpercentile(c, 75, 0)
            ax.fill_between(TGRID, lo, hi, color=FG.COL[m], alpha=0.18, lw=0)
            ax.plot(TGRID, med, color=FG.COL[m], ls=FG.LS[m], lw=1.2)
            j = int(np.nanargmax(med))
            ax.plot(TGRID[j], med[j], marker=FG.MK[m], color=FG.COL[m], ms=4, mec='white', mew=0.4)
        ax.set_xlim(0, 4); ax.set_ylim(-5, 118)
        ax.set_xlabel('Time since step command (s)')
        ax.set_ylabel('Progress along step (%)')
        ax.text(0.97, 0.06, f'{exp}: ' + ('optical flow fused' if exp == 'E1' else 'vision only'),
                transform=ax.transAxes, ha='right', va='bottom', fontsize=7.5, color=FG.INK2)
        FG.letter(ax, k, x=-0.2)
    rng = np.random.default_rng(3)
    for ax, (col, lab), k in ((axes[1, 0], ('overshoot_step_cm', 'Overshoot per step (cm)'), 'c'),
                              (axes[1, 1], ('ekf_vis_man_rms_cm', 'EKF3 vs camera, RMS (cm)'), 'd')):
        for i, m in enumerate(ARMS):
            for j, (exp, face) in enumerate((('E1', 'white'), ('E3', FG.COL[m]))):
                v = res[exp].loc[res[exp]['mode'] == m, col].dropna().values
                x0 = i * 1.0 + (j - 0.5) * 0.36
                bp = ax.boxplot(v, positions=[x0], widths=0.3, patch_artist=True, showfliers=False,
                                medianprops=dict(color=FG.INK, lw=1.0), whiskerprops=dict(lw=0.7),
                                capprops=dict(lw=0.7), boxprops=dict(lw=0.7, edgecolor=FG.COL[m]))
                bp['boxes'][0].set_facecolor(face); bp['boxes'][0].set_alpha(0.85 if j else 1)
                ax.scatter(x0 + rng.uniform(-0.07, 0.07, len(v)), v, s=4, marker=FG.MK[m],
                           color=FG.INK if j else FG.COL[m], alpha=0.6, linewidths=0, zorder=3)
        ax.set_xticks(range(3)); ax.set_xticklabels([FG.SHORT[m] for m in ARMS])
        ax.set_ylabel(lab); ax.set_ylim(bottom=0)
        FG.letter(ax, k, x=-0.2)
    h = [FG.Line2D([], [], color=FG.COL[m], ls=FG.LS[m], marker=FG.MK[m], ms=4, label=FG.LAB[m]) for m in ARMS]
    fig.legend(handles=h, loc='lower center', ncol=3, frameon=False, bbox_to_anchor=(0.5, -0.045))
    hb = [FG.Patch(facecolor='white', edgecolor=FG.INK, label='E1: optical flow fused (open box)'),
          FG.Patch(facecolor='#9fb7a9', edgecolor=FG.INK, label='E3: vision only (filled box)')]
    fig.legend(handles=hb, loc='lower center', ncol=2, frameon=False, bbox_to_anchor=(0.5, -0.095))
    FG.save(fig, out, 9)


def summarise(df):
    row = {}
    for mt in METRICS + ['frame_interval_ms']:
        if mt not in df:
            continue
        g = {m: df.loc[df['mode'] == m, mt].dropna().to_numpy() for m in ARMS}
        r = {'median': {m: float(np.median(g[m])) if len(g[m]) else None for m in ARMS},
             'n': {m: int(len(g[m])) for m in ARMS}}
        for ref in ('constant', 'tuned'):
            if len(g['adaptive']) > 2 and len(g[ref]) > 2:
                c = AZ.compare(g['adaptive'], g[ref], seed=SEED)
                r[f'B_vs_{ref}'] = {k: (float(v) if isinstance(v, (int, float, np.floating)) else v) for k, v in c.items()}
        if all(len(g[m]) > 2 for m in ARMS):
            from scipy.stats import kruskal
            r['kruskal_p'] = float(kruskal(*g.values()).pvalue)
        row[mt] = r
    return row


def main():
    ap = argparse.ArgumentParser()
    for k in ('--e1', '--e3', '--e1-logs', '--e3-logs', '--out'):
        ap.add_argument(k, required=True)
    a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
    res, curves, nums, rel = {}, {}, {}, {}
    for exp, R, L in (('E1', a.e1, a.e1_logs), ('E3', a.e3, a.e3_logs)):
        df, cv = campaign(R, L)
        df.to_csv(os.path.join(a.out, f'maneuver_per_sortie_{exp}.csv'), index=False)
        res[exp], curves[exp], nums[exp] = df, cv, summarise(df)
        rel[exp] = {'sorties': int(len(df)),
                    'touchdowns': int((df.landing_result == 'touchdown').sum()),
                    'ekf_warnings_in_flight': int(df.ekf_warn_in_flight.fillna(0).sum()),
                    'sorties_with_warning_after_disarm': int((df.ekf_warn_after_disarm.fillna(0) > 0).sum()),
                    'first_warning_after_disarm_s_min': (float(df.ekf_warn_after_disarm_s.min())
                                                         if df.ekf_warn_after_disarm_s.notna().any() else None),
                    'lane_switches': int(df.lane_switches.sum()),
                    'relpos_lost_samples': int(df.relpos_lost_samples.sum()),
                    'tilt_gated_frames': int(df.maneuver_tilt_gated.sum()),
                    'rangefinder_zero_readings': int(df.rng_zero_readings.sum())}
    json.dump(nums, open(os.path.join(a.out, 'maneuver_numbers.json'), 'w'), indent=1, default=float)
    json.dump(rel, open(os.path.join(a.out, 'reliability.json'), 'w'), indent=1, default=float)
    fig9(res, curves, a.out)
    for exp in nums:
        print('==', exp, json.dumps(rel[exp]))
        for mt, row in nums[exp].items():
            med = row['median']
            line = f"{mt:20s} A {med['constant']:.2f}  A' {med['tuned']:.2f}  B {med['adaptive']:.2f}"
            for ref in ('constant', 'tuned'):
                c = row.get(f'B_vs_{ref}')
                if c:
                    line += (f" | B-{ref[:5]} {c['med_diff']:+.2f} [{c['ci_lo']:+.2f},{c['ci_hi']:+.2f}]"
                             f" p={c['p']:.4f} d={c['delta']:+.2f}")
            line += f" | KW {row.get('kruskal_p', float('nan')):.4f}"
            print(line)
        pk = {m: float(np.nanmax(np.nanmedian(curves[exp][m], 0))) * 100 for m in ARMS}
        print('step-response peak (%)', pk)


if __name__ == '__main__':
    main()
