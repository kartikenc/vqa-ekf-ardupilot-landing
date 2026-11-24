# ArduPilot DataFlash logs

The flight-controller logs (`.bin`, one per sortie, about 3-5 MB each) are too large for this
repository and are archived with the Zenodo record of this release (see the DOI badge and
`CITATION.cff`):

| File in Zenodo | Content |
|---|---|
| `E1_bins_92-103.zip`, `E1_bins_104-115.zip`, `E1_bins_116-127.zip`, `E1_bins_128-136.zip` | E1 (optical flow on), 45 valid sorties |
| `E3_bins.zip` | E3 (optical flow off), 45 valid sorties plus the invalidated attempt |

`analysis/analyze_vqa_ekf_final.py match-bins` assigns every .bin to its sortie by content;
`data/*/results_frozen*/bin_match.csv` lists the verified assignment used in the article.
