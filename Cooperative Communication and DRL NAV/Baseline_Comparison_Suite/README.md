# Baseline Comparison Suite

This folder compares your DRL navigation policy against baseline policies on the same source-destination missions.

## Included Models
- `DRL`: your trained agent (`getAction`).
- `Straight`: always moves toward destination.
- `NearestRelay`: tries nearest relay, then destination when close.
- `SNRGreedy`: one-step lookahead trading off destination progress and best relay SINR.

## Files
- `run_baseline_comparison.m` : main runner (simulates + saves metrics + generates plots).
- `plot_baseline_results.m` : plotting utility.

## Quick Start
From MATLAB:

```matlab
cd("C:\Users\apaas\OneDrive\Documents\MATLAB\FYP Sem 8\Cooperative Communication and DRL NAV\Baseline_Comparison_Suite");
[results, summaryTable] = run_baseline_comparison("..\drl_nav_final.mat", 50);
```

Optional:

```matlab
[results, summaryTable] = run_baseline_comparison("..\drl_nav_final.mat", 100, "my_outputs", 123);
```

## Outputs
Inside `comparison_outputs` (or your custom output folder):
- `comparison_results.mat` : full run-level data.
- `summary_table.csv` : mean comparison table.
- `src_dst_pairs.mat` : mission pairs used (same for all models).
- `01_summary_means.png` : success + mean metrics (`mean +- SEM`).
- `02_distributions.png` : boxplots for time, SINR, and handoffs.

## Notes
- All models are evaluated on identical mission pairs for fair comparison.
- Randomness is controlled by `randomSeed` argument.
