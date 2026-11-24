"""New paper figure (Fig. 2): the noise level EKF3 actually used, per arm and experiment, against the EKF3 floor
and the camera's measured white noise. Reads the frozen per_sortie.csv files only."""
import argparse, os
import numpy as np, pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

COL = {'constant': '#2c7be5', 'tuned': '#e8633a', 'adaptive': '#1aa179'}
LAB = {'constant': 'A: constant 20 cm', 'tuned': "A$'$: tuned constant", 'adaptive': 'B: VQA-EKF'}
FLOOR_CM, CAM_NOISE_CM = 1.0, 0.155   # VISO_POS_M_NSE = 0.01 m; hand-held white noise 1.55 mm (gate G1)

def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--e1'); ap.add_argument('--e3'); ap.add_argument('--out')
    a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
    plt.rcParams.update({'font.family': 'serif', 'font.size': 9, 'axes.spines.top': False, 'axes.spines.right': False})
    fig, ax = plt.subplots(figsize=(6.3, 3.0))
    x = 0; ticks = []; labels = []
    for exp, path in (('E1 (flow on)', a.e1), ('E3 (flow off)', a.e3)):
        d = pd.read_csv(os.path.join(path, 'per_sortie.csv')); d = d[d.valid == True]
        for m in ('constant', 'tuned', 'adaptive'):
            s = d[d['mode'] == m]
            med, p95, lo = s.visp_man_med_cm.median(), s.visp_man_p95_cm.median(), s.visp_perr_p05_cm.median()
            ax.vlines(x, lo, p95, color=COL[m], lw=2.2)
            ax.plot(x, med, 'o', color=COL[m], ms=6, label=LAB[m] if exp.startswith('E1') else None)
            if m == 'adaptive':
                ratio = (s.visp_man_p95_cm / s.visp_man_med_cm).median()
                ax.annotate(f'p95/median = {ratio:.2f}', xy=(x, p95), xytext=(x + 0.12, 3.2), fontsize=7.5,
                            ha='left', arrowprops=dict(arrowstyle='-', lw=0.6, color='0.4'))
            ticks.append(x); labels.append({'constant': 'A', 'tuned': "A$'$", 'adaptive': 'B'}[m]); x += 1
        ax.text(x - 2, 45, exp, ha='center', fontsize=9)
        x += 0.8
    gap = 3.4  # centre of the gap between the two experiments
    ax.axhline(FLOOR_CM, ls='--', lw=1, color='0.3'); ax.text(gap, FLOOR_CM * 0.78, 'EKF3 floor\n(1 cm)', ha='center', va='top', fontsize=8, color='0.3')
    ax.axhline(CAM_NOISE_CM, ls=':', lw=1, color='0.3'); ax.text(gap, CAM_NOISE_CM * 1.15, 'camera white noise (1.55 mm)', ha='center', fontsize=8, color='0.3')
    ax.set_yscale('log'); ax.set_ylim(0.1, 80); ax.set_xlim(-0.6, x - 0.4)
    ax.set_xticks(ticks); ax.set_xticklabels(labels)
    ax.set_ylabel(r'$\sigma$ used by EKF3 (cm)')
    ax.legend(loc='center left', bbox_to_anchor=(0.0, 0.62), frameon=False, fontsize=8)
    fig.tight_layout()
    for ext in ('pdf', 'eps', 'png'):
        fig.savefig(os.path.join(a.out, f'Fig2_sigma_vs_floor.{ext}'), dpi=600 if ext == 'png' else None)
    error_decomposition(a.e1, a.e3, a.out)
    touchdown_ecdf(a.e1, a.e3, a.out)
    print('written')


def touchdown_ecdf(e1, e3, out):
    """Fig. 4: empirical CDF of the radial touchdown error, camera point and controlled point, E1 and E3."""
    plt.rcParams.update({'font.family': 'serif', 'font.size': 9, 'axes.spines.top': False, 'axes.spines.right': False})
    MK = {'constant': 'o', 'tuned': 's', 'adaptive': '^'}
    fig, axes = plt.subplots(2, 2, figsize=(7.0, 5.2), sharey=True)
    panels = [(e1, 'td_radial_cm', '(a) E1, camera point'), (e1, 'td_radial_corr_cm', '(b) E1, controlled point'),
              (e3, 'td_radial_cm', '(c) E3, camera point'), (e3, 'td_radial_corr_cm', '(d) E3, controlled point')]
    for ax, (path, col, title) in zip(axes.flat, panels):
        d = pd.read_csv(os.path.join(path, 'per_sortie.csv')); d = d[d.valid == True]
        for m in ('constant', 'tuned', 'adaptive'):
            v = np.sort(d.loc[d['mode'] == m, col].dropna().values)
            y = np.arange(1, len(v) + 1) / len(v)
            ax.step(np.r_[0, v], np.r_[0, y], where='post', color=COL[m], lw=1.4, label=LAB[m])
            ax.plot(v, y, MK[m], color=COL[m], ms=3.5)
        ax.axhline(0.5, ls=':', lw=0.8, color='0.4')
        ax.set_title(title, fontsize=9, loc='left'); ax.set_xlim(left=0); ax.grid(alpha=0.25)
        ax.set_xlabel('Radial touchdown error (cm)')
    for ax in axes[:, 0]:
        ax.set_ylabel('Fraction of sorties')
    axes[0, 1].legend(frameon=False, fontsize=7.5, loc='lower right')
    fig.tight_layout()
    for ext in ('pdf', 'eps', 'png'):
        fig.savefig(os.path.join(out, f'Fig4_touchdown_ecdf.{ext}'), dpi=600 if ext == 'png' else None)


def error_decomposition(e1, e3, out):
    """Fig. 6: where the touchdown error comes from (median and IQR per arm), E1 and E3.
    Dispersion about the arm centroid needs signed lateral offsets, so it is drawn for E3 only."""
    plt.rcParams.update({'font.family': 'serif', 'font.size': 9, 'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.8), sharey=True)
    for ax, (title, path, metrics) in zip(axes, (
            ('E1: optical flow on', e1, [('lastfix_radial_pad_cm', 'EKF error\nat last fix'), ('blind_drift_cm', 'Blind\ndrift'), ('td_radial_corr_cm', 'Touchdown\n(controlled)')]),
            ('E3: optical flow off', e3, [('lastfix_radial_pad_cm', 'EKF error\nat last fix'), ('blind_drift_cm', 'Blind\ndrift'), ('td_spread_cm', 'Dispersion'), ('td_radial_corr_cm', 'Touchdown\n(controlled)')]))):
        d = pd.read_csv(os.path.join(path, 'per_sortie.csv')); d = d[d.valid == True]
        w = 0.26
        for k, m in enumerate(('constant', 'tuned', 'adaptive')):
            s = d[d['mode'] == m]
            for j, (col, _) in enumerate(metrics):
                v = s[col].dropna(); q1, med, q3 = np.percentile(v, [25, 50, 75])
                xx = j + (k - 1) * w
                ax.bar(xx, med, w * 0.92, color=COL[m], label=LAB[m] if j == 0 and ax is axes[0] else None)
                ax.vlines(xx, q1, q3, color='0.25', lw=1)
        ax.set_xticks(range(len(metrics))); ax.set_xticklabels([t for _, t in metrics], fontsize=8)
        ax.set_title(title, fontsize=9)
    axes[0].set_ylabel('cm (median, IQR)')
    axes[0].legend(frameon=False, fontsize=7.5, loc='upper left')
    fig.tight_layout()
    for ext in ('pdf', 'eps', 'png'):
        fig.savefig(os.path.join(out, f'Fig6_error_decomposition.{ext}'), dpi=600 if ext == 'png' else None)


if __name__ == '__main__':
    main()
