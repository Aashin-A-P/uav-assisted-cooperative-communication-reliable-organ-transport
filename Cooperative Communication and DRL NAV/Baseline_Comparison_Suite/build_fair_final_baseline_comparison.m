function [pairTable, summaryTable] = build_fair_final_baseline_comparison(comparisonDir, outputDir)
%BUILD_FAIR_FINAL_BASELINE_COMPARISON
% Honest comparison of DRL vs Final Baselines-style metrics:
% SINR, outage probability, reliability.
%
% Usage:
%   build_fair_final_baseline_comparison()
%   [pairTable, summaryTable] = build_fair_final_baseline_comparison("comparison_outputs", "final_baseline_fair_plots")

if nargin < 1 || strlength(string(comparisonDir)) == 0
    comparisonDir = fullfile(fileparts(mfilename("fullpath")), "comparison_outputs");
end
if nargin < 2 || strlength(string(outputDir)) == 0
    outputDir = fullfile(fileparts(mfilename("fullpath")), "final_baseline_fair_plots");
end

mkdir(outputDir);

% ---------- Load DRL run outputs ----------
cmp = load(fullfile(comparisonDir, "comparison_results.mat"), "results");
pairsS = load(fullfile(comparisonDir, "src_dst_pairs.mat"), "srcDstPairs");
results = cmp.results;
srcDstPairs = pairsS.srcDstPairs;

n = size(srcDstPairs, 1);
if numel(results.DRL.avgSINR) < n
    n = numel(results.DRL.avgSINR);
    srcDstPairs = srcDstPairs(1:n, :);
end

% ---------- Load baseline matrices ----------
rootDir = fileparts(fileparts(mfilename("fullpath")));

distanceMat = read_labeled_numeric_matrix(fullfile(rootDir, "..", "Baselines", "direct_comm_baseline_results", "pairwise_distance_matrix_km.csv"));
sinrDirect = read_labeled_numeric_matrix(fullfile(rootDir, "..", "Baselines", "direct_comm_baseline_results", "pairwise_SINR_matrix_dB.csv"));
sinrDF = read_labeled_numeric_matrix(fullfile(rootDir, "..", "Baselines", "single_uav_df_baseline_results", "pairwise_SINR_matrix_dB.csv"));
sinrAF = read_labeled_numeric_matrix(fullfile(rootDir, "..", "Baselines", "single_uav_af_baseline_results", "pairwise_E2E_SINR_AF_matrix_dB.csv"));
sinrTwoHop = read_labeled_numeric_matrix(fullfile(rootDir, "..", "Baselines", "twohop_ground_baseline_results", "pairwise_E2E_SINR_matrix_dB.csv"));

% ---------- Build paired vectors ----------
modelNames = ["DRL", "Direct", "UAV_DF", "TwoHop", "UAV_AF"];
sinr = nan(n, numel(modelNames));
distanceKm = nan(n, 1);

for i = 1:n
    s = srcDstPairs(i, 1);
    d = srcDstPairs(i, 2);

    distanceKm(i) = safe_lookup(distanceMat, s, d);
    sinr(i, 1) = results.DRL.avgSINR(i);
    sinr(i, 2) = safe_lookup(sinrDirect, s, d);
    sinr(i, 3) = safe_lookup(sinrDF, s, d);
    sinr(i, 4) = safe_lookup(sinrTwoHop, s, d);
    sinr(i, 5) = safe_lookup(sinrAF, s, d);
end

sinrThreshDb = 10;
outage = sinr < sinrThreshDb;
reliability = exp(-1 ./ max(10.^(sinr / 10), 1e-12));
reliability = min(max(reliability, 0), 1);

pairTable = table((1:n)', srcDstPairs(:, 1), srcDstPairs(:, 2), distanceKm, ...
    sinr(:, 1), sinr(:, 2), sinr(:, 3), sinr(:, 4), sinr(:, 5), ...
    outage(:, 1), outage(:, 2), outage(:, 3), outage(:, 4), outage(:, 5), ...
    reliability(:, 1), reliability(:, 2), reliability(:, 3), reliability(:, 4), reliability(:, 5), ...
    'VariableNames', { ...
    'Run', 'Source', 'Destination', 'Distance_km', ...
    'SINR_DRL', 'SINR_Direct', 'SINR_UAV_DF', 'SINR_TwoHop', 'SINR_UAV_AF', ...
    'Outage_DRL', 'Outage_Direct', 'Outage_UAV_DF', 'Outage_TwoHop', 'Outage_UAV_AF', ...
    'Reliability_DRL', 'Reliability_Direct', 'Reliability_UAV_DF', 'Reliability_TwoHop', 'Reliability_UAV_AF'});

writetable(pairTable, fullfile(outputDir, "paired_metrics.csv"));

% ---------- Summary table ----------
Model = modelNames';
SINR_Mean_dB = zeros(numel(modelNames), 1);
SINR_CI95_dB = zeros(numel(modelNames), 1);
Outage_Mean = zeros(numel(modelNames), 1);
Outage_CI95 = zeros(numel(modelNames), 1);
Reliability_Mean = zeros(numel(modelNames), 1);
Reliability_CI95 = zeros(numel(modelNames), 1);
PValue_vs_DRL_SINR = nan(numel(modelNames), 1);
PValue_vs_DRL_Outage = nan(numel(modelNames), 1);

for m = 1:numel(modelNames)
    xSinr = sinr(:, m);
    xOut = outage(:, m);
    xRel = reliability(:, m);

    SINR_Mean_dB(m) = mean(xSinr, "omitnan");
    SINR_CI95_dB(m) = ci95(xSinr);
    Outage_Mean(m) = mean(xOut, "omitnan");
    Outage_CI95(m) = ci95(xOut);
    Reliability_Mean(m) = mean(xRel, "omitnan");
    Reliability_CI95(m) = ci95(xRel);

    if m ~= 1
        PValue_vs_DRL_SINR(m) = paired_pvalue(sinr(:, 1), xSinr);
        PValue_vs_DRL_Outage(m) = paired_pvalue(double(outage(:, 1)), double(xOut));
    end
end

summaryTable = table(Model, SINR_Mean_dB, SINR_CI95_dB, Outage_Mean, Outage_CI95, ...
    Reliability_Mean, Reliability_CI95, PValue_vs_DRL_SINR, PValue_vs_DRL_Outage);
writetable(summaryTable, fullfile(outputDir, "summary_with_ci_and_pvalues.csv"));

% ---------- Plot 1: SINR/Outage/Reliability vs distance ----------
[binCenters, mSinr, mOut, mRel] = distance_binned_metrics(distanceKm, sinr, outage, reliability, 6);

fig1 = figure("Color", "w", "Position", [120 120 1280 420]);
tiledlayout(1, 3, "Padding", "compact", "TileSpacing", "compact");

nexttile;
plot(binCenters, mSinr, "LineWidth", 1.8);
grid on;
xlabel("Source-Destination Distance (km)");
ylabel("Average End-to-End SINR (dB)");
title("SINR Comparison (Paired Missions)");
legend(modelNames, "Location", "southwest");

nexttile;
plot(binCenters, mOut, "LineWidth", 1.8);
grid on;
xlabel("Source-Destination Distance (km)");
ylabel("Outage Probability");
title("Outage Probability (Paired Missions)");
ylim([0 1]);
legend(modelNames, "Location", "northwest");

nexttile;
plot(binCenters, mRel, "LineWidth", 1.8);
grid on;
xlabel("Source-Destination Distance (km)");
ylabel("Reliability (Success Probability)");
title("Reliability Comparison (Paired Missions)");
ylim([0 1]);
legend(modelNames, "Location", "southwest");

save_plot(fig1, fullfile(outputDir, "fair_comparison_distance_curves.png"));

% ---------- Plot 2: Mean +- CI ----------
fig2 = figure("Color", "w", "Position", [130 130 1280 430]);
tiledlayout(1, 3, "Padding", "compact", "TileSpacing", "compact");

nexttile;
bar(SINR_Mean_dB); hold on;
errorbar(1:numel(modelNames), SINR_Mean_dB, SINR_CI95_dB, "k.", "LineWidth", 1.2); hold off;
xticklabels(modelNames); ylabel("dB"); title("Mean SINR +- 95% CI"); grid on;

nexttile;
bar(Outage_Mean); hold on;
errorbar(1:numel(modelNames), Outage_Mean, Outage_CI95, "k.", "LineWidth", 1.2); hold off;
xticklabels(modelNames); ylabel("Probability"); title("Mean Outage +- 95% CI"); ylim([0 1]); grid on;

nexttile;
bar(Reliability_Mean); hold on;
errorbar(1:numel(modelNames), Reliability_Mean, Reliability_CI95, "k.", "LineWidth", 1.2); hold off;
xticklabels(modelNames); ylabel("Probability"); title("Mean Reliability +- 95% CI"); ylim([0 1]); grid on;

save_plot(fig2, fullfile(outputDir, "fair_comparison_mean_ci.png"));

% ---------- Plot 3: Paired delta vs baselines ----------
baselineNames = modelNames(2:end);
dSinr = zeros(1, numel(baselineNames));
dOut = zeros(1, numel(baselineNames));
dRel = zeros(1, numel(baselineNames));
for b = 1:numel(baselineNames)
    dSinr(b) = mean(sinr(:, 1) - sinr(:, b + 1), "omitnan");
    dOut(b) = mean(outage(:, b + 1) - outage(:, 1), "omitnan"); % positive means DRL lower outage
    dRel(b) = mean(reliability(:, 1) - reliability(:, b + 1), "omitnan");
end

fig3 = figure("Color", "w", "Position", [140 140 1050 390]);
tiledlayout(1, 3, "Padding", "compact", "TileSpacing", "compact");

nexttile; bar(dSinr); yline(0, "k-"); xticklabels(baselineNames); ylabel("dB");
title("Delta SINR: DRL - Baseline"); grid on;

nexttile; bar(dOut); yline(0, "k-"); xticklabels(baselineNames); ylabel("Probability");
title("Delta Outage: Baseline - DRL"); grid on;

nexttile; bar(dRel); yline(0, "k-"); xticklabels(baselineNames); ylabel("Probability");
title("Delta Reliability: DRL - Baseline"); grid on;

save_plot(fig3, fullfile(outputDir, "fair_paired_deltas.png"));

save(fullfile(outputDir, "fair_comparison_workspace.mat"), ...
    "pairTable", "summaryTable", "distanceKm", "sinr", "outage", "reliability", "modelNames");

fprintf("Fair comparison outputs saved in: %s\n", outputDir);
end

function M = read_labeled_numeric_matrix(csvPath)
raw = readcell(csvPath, "TextType", "string");
M = cell2mat(raw(2:end, 2:end));
end

function v = safe_lookup(M, r, c)
if r >= 1 && c >= 1 && r <= size(M, 1) && c <= size(M, 2)
    v = M(r, c);
else
    v = NaN;
end
if isinf(v)
    v = NaN;
end
end

function c = ci95(x)
x = x(~isnan(x));
n = numel(x);
if n <= 1
    c = NaN;
else
    c = 1.96 * std(x) / sqrt(n);
end
end

function p = paired_pvalue(x, y)
ok = ~isnan(x) & ~isnan(y);
x = x(ok); y = y(ok);
if numel(x) < 3
    p = NaN;
    return;
end

try
    p = signrank(x, y);
catch
    % Toolbox fallback: paired t-test.
    [~, p] = ttest(x, y);
end
end

function [centers, mSinr, mOut, mRel] = distance_binned_metrics(d, sinr, out, rel, nBins)
d = d(:);
valid = ~isnan(d);
dv = d(valid);

if isempty(dv)
    centers = linspace(1, nBins, nBins)';
    mSinr = nan(nBins, size(sinr, 2));
    mOut = nan(nBins, size(out, 2));
    mRel = nan(nBins, size(rel, 2));
    return;
end

edges = linspace(min(dv), max(dv), nBins + 1);
centers = (edges(1:end-1) + edges(2:end)) / 2;
mSinr = nan(nBins, size(sinr, 2));
mOut = nan(nBins, size(out, 2));
mRel = nan(nBins, size(rel, 2));

for b = 1:nBins
    if b < nBins
        idx = d >= edges(b) & d < edges(b + 1);
    else
        idx = d >= edges(b) & d <= edges(b + 1);
    end

    if any(idx)
        mSinr(b, :) = mean(sinr(idx, :), 1, "omitnan");
        mOut(b, :) = mean(out(idx, :), 1, "omitnan");
        mRel(b, :) = mean(rel(idx, :), 1, "omitnan");
    end
end
end

function save_plot(fig, outPath)
if exist("exportgraphics", "file")
    exportgraphics(fig, outPath, "Resolution", 300);
else
    saveas(fig, outPath);
end
end
