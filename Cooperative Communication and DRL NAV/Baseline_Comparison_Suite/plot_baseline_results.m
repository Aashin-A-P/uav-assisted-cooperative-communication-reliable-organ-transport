function plot_baseline_results(results, models, summaryTable, outputDir)
%PLOT_BASELINE_RESULTS Generate publication-friendly comparison figures.

if nargin < 4 || strlength(string(outputDir)) == 0
    outputDir = pwd;
end
mkdir(outputDir);

n = numel(models);

successPct = zeros(n, 1);
timeMean = zeros(n, 1);
distMean = zeros(n, 1);
battMean = zeros(n, 1);
sinrMean = zeros(n, 1);
handoffMean = zeros(n, 1);

timeSem = zeros(n, 1);
distSem = zeros(n, 1);
battSem = zeros(n, 1);
sinrSem = zeros(n, 1);
handoffSem = zeros(n, 1);

for m = 1:n
    model = models(m);
    successPct(m) = 100 * mean(results.(model).success);

    t = results.(model).timeSec;
    d = results.(model).distanceKm;
    b = results.(model).batteryPct;
    s = results.(model).avgSINR;
    h = results.(model).handoffs;

    timeMean(m) = mean(t); distMean(m) = mean(d); battMean(m) = mean(b);
    sinrMean(m) = mean(s); handoffMean(m) = mean(h);

    timeSem(m) = std(t) / sqrt(numel(t));
    distSem(m) = std(d) / sqrt(numel(d));
    battSem(m) = std(b) / sqrt(numel(b));
    sinrSem(m) = std(s) / sqrt(numel(s));
    handoffSem(m) = std(h) / sqrt(numel(h));
end

f1 = figure("Color", "w", "Position", [100 100 1100 600]);
tiledlayout(2, 3, "TileSpacing", "compact", "Padding", "compact");

nexttile; bar(successPct); xticklabels(models); ylabel("Success (%)"); title("Mission Success");
grid on;

nexttile; bar(timeMean); hold on; errorbar(1:n, timeMean, timeSem, "k.", "LineWidth", 1); hold off;
xticklabels(models); ylabel("Seconds"); title("Delivery Time (mean \pm SEM)"); grid on;

nexttile; bar(distMean); hold on; errorbar(1:n, distMean, distSem, "k.", "LineWidth", 1); hold off;
xticklabels(models); ylabel("km"); title("Distance (mean \pm SEM)"); grid on;

nexttile; bar(battMean); hold on; errorbar(1:n, battMean, battSem, "k.", "LineWidth", 1); hold off;
xticklabels(models); ylabel("Battery (%)"); title("Battery Left (mean \pm SEM)"); grid on;

nexttile; bar(sinrMean); hold on; errorbar(1:n, sinrMean, sinrSem, "k.", "LineWidth", 1); hold off;
xticklabels(models); ylabel("dB"); title("Average SINR (mean \pm SEM)"); grid on;

nexttile; bar(handoffMean); hold on; errorbar(1:n, handoffMean, handoffSem, "k.", "LineWidth", 1); hold off;
xticklabels(models); ylabel("count"); title("Relay Handoffs (mean \pm SEM)"); grid on;

save_figure(f1, fullfile(outputDir, "01_summary_means.png"));

f2 = figure("Color", "w", "Position", [120 120 1100 450]);
tiledlayout(1, 3, "TileSpacing", "compact", "Padding", "compact");

[xTime, gTime] = grouped_vectors(results, models, "timeSec");
[xSinr, gSinr] = grouped_vectors(results, models, "avgSINR");
[xHand, gHand] = grouped_vectors(results, models, "handoffs");

nexttile; boxchart(gTime, xTime); xticklabels(models); title("Time Distribution"); ylabel("Seconds"); grid on;
nexttile; boxchart(gSinr, xSinr); xticklabels(models); title("SINR Distribution"); ylabel("dB"); grid on;
nexttile; boxchart(gHand, xHand); xticklabels(models); title("Handoff Distribution"); ylabel("count"); grid on;

save_figure(f2, fullfile(outputDir, "02_distributions.png"));

writetable(summaryTable, fullfile(outputDir, "summary_table.csv"));
end

function [x, g] = grouped_vectors(results, models, fieldName)
x = [];
g = [];
for m = 1:numel(models)
    v = results.(models(m)).(fieldName);
    x = [x; v(:)]; %#ok<AGROW>
    g = [g; m * ones(numel(v), 1)]; %#ok<AGROW>
end
end

function save_figure(fig, pathPng)
if exist("exportgraphics", "file")
    exportgraphics(fig, pathPng, "Resolution", 300);
else
    saveas(fig, pathPng);
end
end
