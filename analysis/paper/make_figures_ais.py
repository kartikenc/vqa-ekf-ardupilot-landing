"""Data figures of the article (Figs. 2-11; Fig. 1 is the TikZ system diagram), in the journal's figure style.

Reads only frozen outputs and the released logs:
  --e1 / --e3       frozen analyzer folders (per_sortie.csv, bin_match.csv, _bin_cache/ written by the analyzer)
  --e1-logs / --e3-logs  companion-computer log folders (vqa_paper_E1 / vqa_paper_E3, one sub-folder per sortie)
Writes FigN.pdf and FigN.png into --out. EPS is made from the PDF with `pdftops -eps` (fonts embedded).

Style (Springer artwork guidelines): sans-serif lettering 7-9 pt at final size, no titles inside figures,
panel letters (a), (b), ..., arms told apart by colour AND marker / line style / hatching.
"""
import argparse, glob, json, math, os, sys
import numpy as np, pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from scipy import stats

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import analyze_vqa_ekf_final as AZ  # noqa: E402  (bin_metrics: sigma used vs tilt, NIS)

ARMS = ('constant', 'tuned', 'adaptive')
COL = {'constant': '#2c7be5', 'tuned': '#e8633a', 'adaptive': '#1aa179'}
AP = 'A\u2032'
LAB = {'constant': 'A: constant 20 cm', 'tuned': AP + ': tuned constant 1.14 cm', 'adaptive': 'B: VQA-EKF (adaptive)'}
SHORT = {'constant': 'A', 'tuned': AP, 'adaptive': 'B'}
MK = {'constant': 'o', 'tuned': 's', 'adaptive': '^'}
LS = {'constant': '--', 'tuned': '-.', 'adaptive': '-'}
HATCH = {'constant': '////', 'tuned': '....', 'adaptive': ''}
INK, INK2 = '#222222', '#666666'
FLOOR_CM, CAM_NOISE_CM = 1.0, 0.155
VQA = dict(sb=0.0226, p=5.25, q=0.69)
TW = 5.15  # \textwidth of sn-jnl in inches (372 pt)


def style():
    plt.rcParams.update({
        'font.family': 'sans-serif', 'font.sans-serif': ['Liberation Sans', 'Arial', 'Helvetica', 'DejaVu Sans'],
        'mathtext.fontset': 'custom', 'mathtext.rm': 'Liberation Sans', 'mathtext.it': 'Liberation Sans:italic',
        'mathtext.bf': 'Liberation Sans:bold',
        'font.size': 8, 'axes.labelsize': 8, 'xtick.labelsize': 7.5, 'ytick.labelsize': 7.5, 'legend.fontsize': 7.5,
        'axes.linewidth': 0.6, 'xtick.major.width': 0.6, 'ytick.major.width': 0.6, 'lines.linewidth': 1.2,
        'axes.spines.top': False, 'axes.spines.right': False, 'pdf.fonttype': 42, 'ps.fonttype': 42,
        'savefig.bbox': 'tight', 'savefig.pad_inches': 0.02})


def save(fig, out, n):
    for ext in ('pdf', 'png'):
        fig.savefig(os.path.join(out, f'Fig{n}.{ext}'), dpi=600 if ext == 'png' else None)
    plt.close(fig)


def letter(ax, k, x=-0.13, y=1.04):
    ax.text(x, y, f'({k})', transform=ax.transAxes, fontsize=8.5, fontweight='bold', va='bottom', ha='left')


def model_sigma_cm(tilt_deg, L):
    c = np.cos(np.radians(np.clip(tilt_deg, 0, 80)))
    var = VQA['sb'] ** 2 * (1 / c ** 2) ** VQA['p'] * (100 / np.maximum(L, 1.0)) ** VQA['q']
    return np.sqrt(np.clip(var, 1e-4, 0.25)) * 100


def load_per(path):
    d = pd.read_csv(os.path.join(path, 'per_sortie.csv'))
    return d[d.valid == True].copy()


def bin_data(path):
    """sigma-used-vs-tilt points and NIS samples per arm, from the analyzer's own bin_metrics()."""
    bm = pd.read_csv(os.path.join(path, 'bin_match.csv'))
    per = load_per(path)
    bm = bm[bm.sortie_id.isin(per.sortie_id)]
    cache = {}
    for f in glob.glob(os.path.join(path, '_bin_cache', '*.json')):
        d = json.load(open(f, encoding='utf-8'))
        cache[d.get('_file')] = d
    pts = {a: [] for a in ARMS}; nis = {a: [] for a in ARMS}
    for r in bm.itertuples():
        d = cache.get(r.bin)
        if d is None:
            continue
        m = AZ.bin_metrics(d)
        pts[r.mode] += m.get('_visp_man', [])
        nis[r.mode] += m.get('_nis', [])
    return {a: np.asarray(v, float).reshape(-1, 2) for a, v in pts.items()}, {a: np.asarray(v, float) for a, v in nis.items()}


def frames(logs, sid):
    f = glob.glob(os.path.join(logs, f'*_{sid}', 'vision_frames.csv'))
    return pd.read_csv(f[0]) if f else None


# ---------------------------------------------------------------------------------------------------------------
def fig2_sortie(per3, logs3, out):
    """Representative E3 sortie of arm B: the one whose touchdown error is the arm median."""
    b = per3[per3['mode'] == 'adaptive'].copy()
    b['dev'] = (b.td_radial_cm - b.td_radial_cm.median()).abs()
    sid = b.sort_values('dev').sortie_id.iloc[0]
    f = frames(logs3, sid)
    f = f[f.phase != 'WAIT_PILOT'].copy()
    t0 = f.t_capture.iloc[0]
    f['t'] = f.t_capture - t0
    acc = f[f.accepted == 1]
    fig, ax = plt.subplots(4, 1, figsize=(TW, 4.9), sharex=True, gridspec_kw=dict(hspace=0.18))
    # phase bands
    names = {'TAKEOFF': 'Take-off', 'HOVER_BASE': 'Baseline hover', 'MANEUVER': 'Manoeuvre', 'CONVERGE': 'Converge',
             'DESCENT': 'Landing', 'LANDING': 'Landing', 'TOUCHDOWN': ''}
    seg = []
    for ph, g in f.groupby((f.phase != f.phase.shift()).cumsum()):
        seg.append((g.phase.iloc[0], g.t.iloc[0], g.t.iloc[-1]))
    shade = {'HOVER_BASE': '#f2f2f2', 'MANEUVER': '#e6eef8', 'CONVERGE': '#f2f2f2'}
    for a_ in ax:
        for ph, s0, s1 in seg:
            if ph in shade:
                a_.axvspan(s0, s1, color=shade[ph], lw=0, zorder=0)
    done = set()
    for ph, s0, s1 in seg:
        lab = names.get(ph, '')
        if lab and lab not in done and s1 - s0 > 3:
            ax[0].text((s0 + s1) / 2, 2.78, lab, ha='center', va='bottom', fontsize=7, color=INK2)
            done.add(lab)
    z = -f.ekf_z if f.ekf_z.median() < 0 else f.ekf_z
    ax[0].plot(acc.t, -acc.z_raw, '.', color=COL['adaptive'], ms=1.6, label='camera height from marker fix', zorder=2)
    ax[0].plot(f.t, z, color=INK, lw=0.8, label='EKF3 height estimate', zorder=3)
    ax[0].set_ylabel('Height (m)'); ax[0].set_ylim(0, 2.75)
    ax[0].legend(loc='center', ncol=1, frameon=False, fontsize=7, markerscale=4, bbox_to_anchor=(0.5, 0.42))
    ax[1].plot(acc.t, acc.x_sent * 100, '.', color=COL['adaptive'], ms=1.6, label='marker fix, x', zorder=2)
    ax[1].plot(f.t, f.ekf_x * 100, color=INK, lw=0.8, label='EKF3 estimate, x', zorder=3)
    ax[1].plot(f.t, f.ekf_y * 100, color=INK2, lw=0.8, ls='--', label='EKF3 estimate, y', zorder=3)
    ax[1].set_ylabel('Position (cm)'); ax[1].set_ylim(-75, 95)
    ax[1].legend(loc='upper left', ncol=3, frameon=False, fontsize=7, handlelength=1.6, markerscale=4,
                 bbox_to_anchor=(0.06, 1.04))
    ax[2].plot(f.t, f.tilt_deg, color=INK, lw=0.9)
    ax[2].set_ylabel('Tilt (deg)')
    ax[3].plot(acc.t, acc.sigma_sent * 100, color=COL['adaptive'], lw=0.9, label='B: noise sent to EKF3')
    ax[3].axhline(1.14, color=COL['tuned'], ls=LS['tuned'], lw=1.0, label=AP + ': 1.14 cm')
    ax[3].axhline(FLOOR_CM, color=INK2, ls=':', lw=0.9, label='EKF3 floor')
    ax[3].set_ylabel(r'$\sigma$ (cm)'); ax[3].set_ylim(0.9, 1.95)
    ax[3].legend(loc='upper left', ncol=3, frameon=False, fontsize=7, bbox_to_anchor=(0.06, 1.04))
    ax[3].set_xlabel('Time since take-off (s)')
    for k, a_ in zip('abcd', ax):
        letter(a_, k, x=-0.10, y=0.82)
    save(fig, out, 3)
    return sid


def fig3_model(per1, per3, logs1, logs3, out):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(TW, 2.45), gridspec_kw=dict(wspace=0.32))
    th = np.linspace(0, 40, 200)
    for L, ls in ((400, ':'), (700, '--'), (1000, '-')):
        a1.plot(th, model_sigma_cm(th, L), color=COL['adaptive'], ls=ls, lw=1.2, label=f'L = {L}')
    a1.axhline(FLOOR_CM, color=INK2, ls=':', lw=0.8)
    a1.text(39.5, FLOOR_CM * 0.93, 'EKF3 floor', ha='right', va='top', fontsize=7, color=INK2)
    a1.axhline(20, color=COL['constant'], ls=LS['constant'], lw=0.9)
    a1.text(39.5, 20 * 0.86, 'A: 20 cm', ha='right', va='top', fontsize=7, color=COL['constant'])
    a1.axvspan(0, 19.5, color='#e6eef8', lw=0, zorder=0)
    a1.text(9.7, 36, 'tilt range in flight', ha='center', va='top', fontsize=7, color=INK2)
    a1.set_yscale('log'); a1.set_ylim(0.8, 40); a1.set_xlim(0, 40)
    a1.set_yticks([1, 2, 5, 10, 20]); a1.set_yticklabels(['1', '2', '5', '10', '20'])
    a1.set_xlabel('Vehicle tilt (deg)'); a1.set_ylabel(r'Model $\sigma$ (cm)')
    a1.legend(frameon=False, loc='center left', bbox_to_anchor=(0.0, 0.52), title='image sharpness', title_fontsize=7)
    # operating envelope of B: accepted frames, hover + manoeuvre + converge, both campaigns
    T, Lv = [], []
    for per, logs in ((per1, logs1), (per3, logs3)):
        for sid in per.loc[per['mode'] == 'adaptive', 'sortie_id']:
            f = frames(logs, sid)
            if f is None:
                continue
            g = f[(f.accepted == 1) & f.phase.isin(['HOVER_BASE', 'MANEUVER', 'CONVERGE'])]
            T.append(g.tilt_deg.values); Lv.append(g.blur_full.values)
    T, Lv = np.concatenate(T), np.concatenate(Lv)
    hb = a2.hexbin(T, Lv, gridsize=(34, 22), extent=(0, 25, 300, 1200), bins='log', cmap='Greys', mincnt=1,
                   linewidths=0.1)
    tt, ll = np.meshgrid(np.linspace(0, 25, 200), np.linspace(300, 1200, 200))
    cs = a2.contour(tt, ll, model_sigma_cm(tt, ll), levels=[1.0, 1.1, 1.25, 1.5, 2.0, 3.0],
                    colors=COL['adaptive'], linewidths=0.9)
    a2.clabel(cs, fmt=lambda v: f'{v:g} cm', fontsize=6.5, inline_spacing=2)
    a2.set_xlabel('Vehicle tilt (deg)'); a2.set_ylabel('Image sharpness L')
    cb = fig.colorbar(hb, ax=a2, pad=0.02, fraction=0.06)
    cb.set_label('Frames (log)', fontsize=7); cb.ax.tick_params(labelsize=6.5)
    letter(a1, 'a', x=-0.2); letter(a2, 'b', x=-0.24)
    save(fig, out, 2)
    return len(T), float(np.percentile(T, 99)), float(np.median(Lv)), float(np.percentile(Lv, 5)), float(np.percentile(Lv, 95))


def fig4_floor(per1, per3, out):
    fig, ax = plt.subplots(figsize=(TW, 2.5))
    x = 0; ticks, labels = [], []
    for exp, d in (('E1: optical flow fused', per1), ('E3: vision only', per3)):
        for m in ARMS:
            s = d[d['mode'] == m]
            med, p95, lo = s.visp_man_med_cm.median(), s.visp_man_p95_cm.median(), s.visp_perr_p05_cm.median()
            ax.vlines(x, lo, p95, color=COL[m], lw=2.4)
            ax.plot(x, med, MK[m], color=COL[m], ms=6, mec='white', mew=0.6, label=LAB[m] if exp.startswith('E1') else None)
            if m == 'adaptive':
                r = (s.visp_man_p95_cm / s.visp_man_med_cm).median()
                ax.annotate(f'p95/median = {r:.2f}', xy=(x, p95), xytext=(x + 0.15, 3.0), fontsize=7, ha='left',
                            arrowprops=dict(arrowstyle='-', lw=0.6, color=INK2))
            ticks.append(x); labels.append(SHORT[m]); x += 1
        ax.text(x - 2, 45, exp, ha='center', fontsize=7.5)
        x += 0.8
    gap = 3.4
    ax.axhline(FLOOR_CM, ls='--', lw=0.9, color=INK2)
    ax.text(gap, FLOOR_CM * 0.8, 'EKF3 floor (1 cm)', ha='center', va='top', fontsize=7, color=INK2)
    ax.axhline(CAM_NOISE_CM, ls=':', lw=0.9, color=INK2)
    ax.text(gap, CAM_NOISE_CM * 1.15, 'camera white noise (1.55 mm)', ha='center', fontsize=7, color=INK2)
    ax.set_yscale('log'); ax.set_ylim(0.1, 80); ax.set_xlim(-0.6, x - 0.4)
    ax.set_yticks([0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50]); ax.set_yticklabels(['0.1', '0.2', '0.5', '1', '2', '5', '10', '20', '50'])
    ax.set_xticks(ticks); ax.set_xticklabels(labels)
    ax.set_ylabel(r'$\sigma$ used by EKF3 (cm)')
    ax.legend(loc='center left', bbox_to_anchor=(0.0, 0.62), frameon=False)
    save(fig, out, 4)


def fig5_manip(pts1, pts3, out):
    fig, axes = plt.subplots(1, 2, figsize=(TW, 2.3), sharey=True, gridspec_kw=dict(wspace=0.08))
    rng = np.random.default_rng(0)
    for ax, pts, k in ((axes[0], pts1, 'a'), (axes[1], pts3, 'b')):
        for m in ('adaptive', 'tuned', 'constant'):
            p = pts[m]
            if len(p) > 5000:
                p = p[rng.choice(len(p), 5000, replace=False)]
            ax.scatter(p[:, 0], p[:, 1] * 100, s=3, marker=MK[m], color=COL[m], alpha=0.35, linewidths=0, rasterized=True)
        ax.axhline(FLOOR_CM, color=INK2, lw=0.8, ls=':')
        ax.set_yscale('log'); ax.set_ylim(0.8, 30)
        ax.set_yticks([1, 1.5, 2, 5, 10, 20]); ax.set_yticklabels(['1', '1.5', '2', '5', '10', '20'])
        ax.set_xlabel('Vehicle tilt (deg)'); ax.set_xlim(0, 25)
        letter(ax, k, x=-0.02 if k == 'b' else -0.16)
    axes[0].set_ylabel(r'$\sigma$ used by EKF3 (cm)')
    axes[1].text(24.5, 0.93, 'EKF3 floor', ha='right', va='top', fontsize=7, color=INK2)
    h = [Line2D([], [], ls='none', marker=MK[m], ms=4, color=COL[m], label=LAB[m]) for m in ARMS]
    fig.legend(handles=h, loc='lower center', ncol=3, frameon=False, bbox_to_anchor=(0.5, -0.17))
    save(fig, out, 5)


def fig6_ecdf(per1, per3, out):
    fig, axes = plt.subplots(2, 2, figsize=(TW, 4.0), sharey=True, gridspec_kw=dict(hspace=0.42, wspace=0.08))
    panels = [(per1, 'td_radial_cm', 'a', 'camera point'), (per1, 'td_radial_corr_cm', 'b', 'controlled point'),
              (per3, 'td_radial_cm', 'c', 'camera point'), (per3, 'td_radial_corr_cm', 'd', 'controlled point')]
    for ax, (d, col, k, what) in zip(axes.flat, panels):
        for m in ARMS:
            v = np.sort(d.loc[d['mode'] == m, col].dropna().values)
            y = np.arange(1, len(v) + 1) / len(v)
            ax.step(np.r_[0, v], np.r_[0, y], where='post', color=COL[m], ls=LS[m], lw=1.3)
            ax.plot(v, y, MK[m], color=COL[m], ms=3, mec='white', mew=0.4)
        ax.axhline(0.5, ls=':', lw=0.7, color=INK2)
        ax.set_xlim(left=0); ax.set_ylim(0, 1.02)
        ax.set_xlabel(f'Touchdown error, {what} (cm)')
        letter(ax, k, x=-0.02 if k in 'bd' else -0.16)
    for ax in axes[:, 0]:
        ax.set_ylabel('Fraction of sorties')
    h = [Line2D([], [], ls=LS[m], marker=MK[m], ms=3.5, color=COL[m], label=LAB[m]) for m in ARMS]
    fig.legend(handles=h, loc='lower center', ncol=3, frameon=False, bbox_to_anchor=(0.5, -0.05))
    save(fig, out, 6)


def fig7_scatter(per3, out):
    fig, axes = plt.subplots(1, 3, figsize=(TW, 2.05), sharey=True, gridspec_kw=dict(wspace=0.08))
    lim = 12
    for ax, m, k in zip(axes, ARMS, 'abc'):
        s = per3[per3['mode'] == m]
        dx, dy = s.td_corr_dx_cm.values, s.td_corr_dy_cm.values
        g = pd.DataFrame(dict(x=np.round(dx, 2), y=np.round(dy, 2))).groupby(['x', 'y']).size().reset_index(name='n')
        ax.scatter(g.x, g.y, s=14 * g.n, marker=MK[m], color=COL[m], edgecolors='white', linewidths=0.5, zorder=3)
        c = np.array([dx.mean(), dy.mean()])
        r = np.median(np.hypot(dx - c[0], dy - c[1]))
        ax.add_patch(plt.Circle(c, r, fill=False, color=COL[m], lw=1.0, ls='--', zorder=2))
        ax.plot(*c, 'x', color=INK, ms=6, mew=1.3, zorder=4)
        ax.plot(0, 0, '+', color=INK, ms=9, mew=1.0, zorder=4)
        ax.text(0.04, 0.96, f'{SHORT[m]}: dispersion {r:.1f} cm', transform=ax.transAxes, va='top', fontsize=7)
        ax.set_aspect('equal'); ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim)
        ax.set_xticks([-10, -5, 0, 5, 10]); ax.set_yticks([-10, -5, 0, 5, 10])
        ax.set_xlabel('Forward offset (cm)'); ax.grid(alpha=0.2, lw=0.4)
        letter(ax, k, x=-0.02 if k != 'a' else -0.26)
    axes[0].set_ylabel('Lateral offset (cm)')
    h = [Line2D([], [], ls='none', marker='+', ms=7, color=INK, label='marker centre'),
         Line2D([], [], ls='none', marker='x', ms=5, color=INK, label='arm centroid'),
         Line2D([], [], ls='--', color=INK2, label='median distance to centroid')]
    fig.legend(handles=h, loc='lower center', ncol=3, frameon=False, bbox_to_anchor=(0.5, -0.16))
    save(fig, out, 7)


def fig8_decomp(per1, per3, out):
    fig, axes = plt.subplots(1, 2, figsize=(TW, 2.5), sharey=True, gridspec_kw=dict(width_ratios=[3, 4], wspace=0.06))
    for ax, d, metrics, k in (
            (axes[0], per1, [('lastfix_radial_pad_cm', 'Last-fix\nerror'), ('blind_drift_cm', 'Blind\ndrift'),
                             ('td_radial_corr_cm', 'Touchdown\n(controlled)')], 'a'),
            (axes[1], per3, [('lastfix_radial_pad_cm', 'Last-fix\nerror'), ('blind_drift_cm', 'Blind\ndrift'),
                             ('td_spread_cm', 'Disper-\nsion'), ('td_radial_corr_cm', 'Touchdown\n(controlled)')], 'b')):
        w = 0.26
        for i, m in enumerate(ARMS):
            s = d[d['mode'] == m]
            for j, (col, _) in enumerate(metrics):
                v = s[col].dropna(); q1, med, q3 = np.percentile(v, [25, 50, 75])
                xx = j + (i - 1) * w
                ax.bar(xx, med, w * 0.9, color=COL[m], hatch=HATCH[m], edgecolor='white', lw=0.3)
                ax.vlines(xx, q1, q3, color=INK, lw=0.9)
        ax.set_xticks(range(len(metrics))); ax.set_xticklabels([t for _, t in metrics], fontsize=7)
        letter(ax, k, x=-0.02 if k == 'b' else -0.2)
    axes[0].set_ylabel('cm (median and IQR)')
    h = [Patch(facecolor=COL[m], hatch=HATCH[m], edgecolor='white', label=LAB[m]) for m in ARMS]
    fig.legend(handles=h, loc='lower center', ncol=3, frameon=False, bbox_to_anchor=(0.5, -0.14))
    save(fig, out, 8)


def fig9_nis(nis1, nis3, per1, per3, out):
    fig, axes = plt.subplots(1, 2, figsize=(TW, 2.35), sharey=True, gridspec_kw=dict(wspace=0.08))
    xs = np.logspace(-4, np.log10(30), 400)
    for ax, nis, per, k in ((axes[0], nis1, per1, 'a'), (axes[1], nis3, per3, 'b')):
        ax.plot(xs, stats.chi2.cdf(xs, 2), color=INK, lw=1.1, ls=(0, (1, 1)))
        for m in ARMS:
            x = np.sort(np.clip(nis[m], 1e-4, None))
            ax.step(x, np.arange(1, len(x) + 1) / len(x), where='post', color=COL[m], ls=LS[m], lw=1.2)
            med = per.loc[per['mode'] == m, 'nis_mean'].median()
            ax.plot([], [], ' ', label=f'{SHORT[m]}: {med:.3f}' if med < 0.01 else f'{SHORT[m]}: {med:.2f}')
        ax.axvline(5.991, color=INK2, lw=0.6, ls=':')
        ax.text(5.991 * 1.1, 0.05, r'$\chi^2_{2,0.95}$', fontsize=7, color=INK2)
        ax.set_xscale('log'); ax.set_xlim(1e-4, 30); ax.set_ylim(0, 1.02)
        ax.set_xlabel('Position NIS of each vision update')
        leg = ax.legend(frameon=False, loc='lower left', bbox_to_anchor=(0.01, 0.31), title='median sortie-mean NIS',
                        title_fontsize=6.5, fontsize=6.5, handlelength=0, handletextpad=0)
        letter(ax, k, x=-0.02 if k == 'b' else -0.16)
    axes[0].set_ylabel('Fraction of updates')
    h = [Line2D([], [], color=COL[m], ls=LS[m], label=LAB[m]) for m in ARMS] + \
        [Line2D([], [], color=INK, ls=(0, (1, 1)), label=r'$\chi^2_2$ (consistent filter)')]
    fig.legend(handles=h, loc='lower center', ncol=2, frameon=False, bbox_to_anchor=(0.5, -0.24), fontsize=7)
    save(fig, out, 9)


def fig10_flow(per1, per3, out):
    fig, axes = plt.subplots(1, 3, figsize=(TW, 2.3), gridspec_kw=dict(wspace=0.38))
    rng = np.random.default_rng(1)
    for ax, (col, lab), k in zip(axes, (('td_radial_cm', 'Touchdown error (cm)'), ('blind_drift_cm', 'Blind-descent drift (cm)'),
                                        ('pos_innov_rms_cm', 'Innovation RMS (cm)')), 'abc'):
        for i, m in enumerate(ARMS):
            for j, (d, face) in enumerate(((per1, 'white'), (per3, COL[m]))):
                v = d.loc[d['mode'] == m, col].dropna().values
                x0 = i * 1.0 + (j - 0.5) * 0.36
                bp = ax.boxplot(v, positions=[x0], widths=0.3, patch_artist=True, showfliers=False,
                                medianprops=dict(color=INK, lw=1.0), whiskerprops=dict(lw=0.7), capprops=dict(lw=0.7),
                                boxprops=dict(lw=0.7, edgecolor=COL[m]))
                bp['boxes'][0].set_facecolor(face); bp['boxes'][0].set_alpha(0.85 if j else 1)
                ax.scatter(x0 + rng.uniform(-0.07, 0.07, len(v)), v, s=4, marker=MK[m],
                           color=INK if j else COL[m], alpha=0.6, linewidths=0, zorder=3)
        ax.set_xticks(range(3)); ax.set_xticklabels([SHORT[m] for m in ARMS])
        ax.set_ylabel(lab)
        if col == 'pos_innov_rms_cm':
            from matplotlib.ticker import NullFormatter, NullLocator
            ax.set_yscale('log'); ax.yaxis.set_minor_locator(NullLocator()); ax.yaxis.set_minor_formatter(NullFormatter())
            ax.set_yticks([0.2, 0.5, 1, 2, 5]); ax.set_yticklabels(['0.2', '0.5', '1', '2', '5'])
        letter(ax, k, x=-0.3)
    h = [Patch(facecolor='white', edgecolor=INK, label='E1: optical flow fused (open box)'),
         Patch(facecolor='#9fb7a9', edgecolor=INK, label='E3: vision only (filled box)')]
    fig.legend(handles=h, loc='lower center', ncol=2, frameon=False, bbox_to_anchor=(0.5, -0.12))
    save(fig, out, 10)


def fig11_glare(per1, per3, out):
    fig, axes = plt.subplots(1, 2, figsize=(TW, 2.3), sharey=True, gridspec_kw=dict(wspace=0.12))
    for ax, d, k in ((axes[0], per1, 'a'), (axes[1], per3, 'b')):
        t = pd.to_datetime(d.date_time, unit='s') + pd.Timedelta(hours=5.5)
        hh = t.dt.hour + t.dt.minute / 60
        for m in ARMS:
            i = d['mode'] == m
            ax.scatter(hh[i], d.loc[i, 'maneuver_accept_frac'] * 100, marker=MK[m], s=16, color=COL[m],
                       edgecolors='white', linewidths=0.4, zorder=3)
        ax.axvspan(16.5, 18, color='#fbe9d7', lw=0, zorder=0)
        ax.text(17.2, 101.5, 'low sun', ha='center', fontsize=7, color=INK2)
        s38 = d[d.sortie_id == 'E3_S38']
        if len(s38):
            tt = pd.to_datetime(s38.date_time, unit='s') + pd.Timedelta(hours=5.5)
            x = float(tt.dt.hour.iloc[0] + tt.dt.minute.iloc[0] / 60); y = float(s38.maneuver_accept_frac.iloc[0] * 100)
            ax.annotate('only sortie with gated\nvision updates (' + AP + ')', xy=(x, y), xytext=(11.2, 78), fontsize=6.5,
                        arrowprops=dict(arrowstyle='-', lw=0.6, color=INK2))
        ax.set_xlim(10, 18); ax.set_ylim(70, 104)
        ax.set_xticks(range(11, 19, 2)); ax.set_xticklabels([f'{h}:00' for h in range(11, 19, 2)])
        ax.set_xlabel('Local time of take-off')
        letter(ax, k, x=-0.02 if k == 'b' else -0.16)
    axes[0].set_ylabel('Frames accepted in manoeuvre (%)')
    h = [Line2D([], [], ls='none', marker=MK[m], ms=4.5, color=COL[m], label=LAB[m]) for m in ARMS]
    fig.legend(handles=h, loc='lower center', ncol=3, frameon=False, bbox_to_anchor=(0.5, -0.17))
    save(fig, out, 11)


def main():
    ap = argparse.ArgumentParser()
    for k in ('--e1', '--e3', '--e1-logs', '--e3-logs', '--out'):
        ap.add_argument(k, required=True)
    a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
    style()
    per1, per3 = load_per(a.e1), load_per(a.e3)
    info = {}
    info['fig2_sortie'] = fig2_sortie(per3, a.e3_logs, a.out)
    info['fig3_envelope'] = fig3_model(per1, per3, a.e1_logs, a.e3_logs, a.out)
    fig4_floor(per1, per3, a.out)
    pts1, nis1 = bin_data(a.e1); pts3, nis3 = bin_data(a.e3)
    info['n_points'] = {k: [len(pts1[m]) for m in ARMS] for k in ('E1',)} | {'E3': [len(pts3[m]) for m in ARMS]}
    info['nis_n'] = {'E1': [len(nis1[m]) for m in ARMS], 'E3': [len(nis3[m]) for m in ARMS]}
    fig5_manip(pts1, pts3, a.out)
    fig6_ecdf(per1, per3, a.out)
    fig7_scatter(per3, a.out)
    fig8_decomp(per1, per3, a.out)
    fig9_nis(nis1, nis3, per1, per3, a.out)
    fig10_flow(per1, per3, a.out)
    fig11_glare(per1, per3, a.out)
    json.dump(info, open(os.path.join(a.out, 'figure_info.json'), 'w'), indent=1)
    print(json.dumps(info))


if __name__ == '__main__':
    main()
