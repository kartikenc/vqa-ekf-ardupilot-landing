"""Paper statistics computed from the FROZEN per-sortie tables (analyzer v1.1 outputs).
Nothing here re-processes flight logs. Every number that the manuscript quotes and that is not in an analyzer
report (non-inferiority intervals, power, E1-vs-E3 comparison, sigma ranges) comes from this script.

Usage: python paper_stats.py --e1 data/E1/results_E1_v11 --e3 data/E3/results_E3 --out analysis/paper/out
"""
import argparse, json, os
import numpy as np, pandas as pd
from scipy.stats import mannwhitneyu, norm

SEED = 20261124
ARMS = [('constant', 'A'), ('tuned', "A'"), ('adaptive', 'B')]
GLARE_E3 = ['E3_S35', 'E3_S36', 'E3_S37', 'E3_S38', 'E3_S39', 'E3_S40', 'E3_S41']

def cliff(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    return float(np.mean(np.sign(a[:, None] - b[None, :])))

def boot_median_diff(a, b, level, rng, n=10000):
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = [np.median(rng.choice(a, a.size)) - np.median(rng.choice(b, b.size)) for _ in range(n)]
    lo, hi = np.percentile(d, [(100 - level) / 2, 100 - (100 - level) / 2])
    return float(np.median(a) - np.median(b)), float(lo), float(hi)

def noether_n_per_arm(delta, alpha, power=0.8):
    p = 0.5 + delta / 2
    z = norm.ppf(1 - alpha / 2) + norm.ppf(power)
    return int(np.ceil(z ** 2 / (12 * 0.25 * (p - 0.5) ** 2) / 2))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--e1', required=True); ap.add_argument('--e3', required=True); ap.add_argument('--out', required=True)
    a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
    rng = np.random.default_rng(SEED)
    e1 = pd.read_csv(os.path.join(a.e1, 'per_sortie.csv')); e1 = e1[e1.valid == True]
    e3 = pd.read_csv(os.path.join(a.e3, 'per_sortie.csv')); e3 = e3[e3.valid == True]
    res = {'seed': SEED, 'n': {'E1': int(len(e1)), 'E3': int(len(e3))}}

    # 1. sigma actually used by EKF3 in the manoeuvre (VISP.PErr), per arm
    sig = {}
    for name, d in (('E1', e1), ('E3', e3)):
        for m, lab in ARMS:
            s = d[d['mode'] == m]
            sig[f'{name} {lab}'] = {'median_of_sortie_medians_cm': float(s.visp_man_med_cm.median()),
                                   'median_of_sortie_p95_cm': float(s.visp_man_p95_cm.median()),
                                   'p95_over_median': float((s.visp_man_p95_cm / s.visp_man_med_cm).median()),
                                   'min_frame_p05_cm': float(s.visp_perr_p05_cm.min())}
    res['sigma_used'] = sig

    # 2. post hoc non-inferiority: 90 % bootstrap CI of the median difference B - reference
    ni = {}
    for name, d in (('E1', e1), ('E3', e3)):
        b = d[d['mode'] == 'adaptive']
        for m, lab in ARMS[:2]:
            r = d[d['mode'] == m]
            for col in ('td_radial_cm', 'td_radial_corr_cm'):
                md, lo, hi = boot_median_diff(b[col].dropna(), r[col].dropna(), 90, rng)
                ni[f'{name} B-{lab} {col}'] = {'median_diff_cm': md, 'ci90_lo': lo, 'ci90_hi': hi}
    res['noninferiority_90ci'] = ni

    # 3. power (Noether, two-sided, 80 %)
    res['power_n_per_arm'] = {f'delta {dl} alpha {al}': noether_n_per_arm(dl, al)
                              for dl in (0.33, 0.44, 0.5) for al in (0.05, 0.025)}

    # 4. E1 (flow on) vs E3 (flow off), per arm, rotation-invariant measures only
    cols = ['td_radial_cm', 'td_radial_corr_cm', 'blind_drift_cm', 'lastfix_radial_pad_cm', 'pos_innov_rms_cm', 'maneuver_accept_frac']
    cmp = {}
    for m, lab in ARMS:
        x1, x3 = e1[e1['mode'] == m], e3[e3['mode'] == m]
        for c in cols:
            u, v = x3[c].dropna().values, x1[c].dropna().values
            md, lo, hi = boot_median_diff(u, v, 95, rng)
            cmp[f'{lab} {c}'] = {'E1_median': float(np.median(v)), 'E3_median': float(np.median(u)), 'diff_E3_minus_E1': md,
                                 'ci95_lo': lo, 'ci95_hi': hi, 'p_mw': float(mannwhitneyu(u, v).pvalue), 'cliff_delta': cliff(u, v)}
    res['E1_vs_E3'] = cmp

    # 5. touchdown lateral sign check (E1 dy recorded without sign)
    res['dy_negative_count'] = {'E1': int((e1.td_dy_cm < 0).sum()), 'E3': int((e3.td_dy_cm < 0).sum())}

    # 6. glare: E3 manoeuvre detection, glare sorties vs the rest
    g = e3.sortie_id.isin(GLARE_E3)
    res['E3_detection'] = {'glare_median_pct': float(100 * e3[g].maneuver_accept_frac.median()),
                           'glare_min_pct': float(100 * e3[g].maneuver_accept_frac.min()),
                           'other_median_pct': float(100 * e3[~g].maneuver_accept_frac.median()),
                           'glare_by_arm': e3[g]['mode'].value_counts().to_dict()}
    with open(os.path.join(a.out, 'paper_numbers.json'), 'w') as f:
        json.dump(res, f, indent=1)
    print(json.dumps(res, indent=1))

if __name__ == '__main__':
    main()
