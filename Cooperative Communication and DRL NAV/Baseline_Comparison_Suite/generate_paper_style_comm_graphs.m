function metrics = generate_paper_style_comm_graphs(outputDir, randomSeed)
%GENERATE_PAPER_STYLE_COMM_GRAPHS
% Create paper-style AF/DF/Hybrid outage + throughput comparison graphs.
%
% Usage:
%   metrics = generate_paper_style_comm_graphs();
%   metrics = generate_paper_style_comm_graphs("paper_style_outputs", 7);

if nargin < 1 || strlength(string(outputDir)) == 0
    outputDir = fullfile(fileparts(mfilename("fullpath")), "paper_style_outputs");
end
if nargin < 2 || isempty(randomSeed)
    randomSeed = 42;
end

rng(randomSeed);
mkdir(outputDir);

% Simulation controls
SNRdB = -10:2:20;
Kset = 1:5;                % relay counts
MC = 4000;                 % Monte Carlo samples per SNR point
gammaTh = 1;               % outage threshold in linear scale (0 dB)
alpha = 2.2;               % path-loss exponent
totalDistanceM = 22000;    % source-to-destination equivalent range
bwHz = 10e6;
eta = 0.85;                % spectral efficiency factor

nK = numel(Kset);
nSNR = numel(SNRdB);

PoutAF = zeros(nK, nSNR);
PoutDF = zeros(nK, nSNR);
PoutHY = zeros(nK, nSNR);

ThrAF = zeros(nK, nSNR);
ThrDF = zeros(nK, nSNR);
ThrHY = zeros(nK, nSNR);

for ik = 1:nK
    k = Kset(ik);
    numHops = k + 1;
    hopMean = totalDistanceM / numHops;

    for is = 1:nSNR
        snrLin = 10^(SNRdB(is) / 10);
        gammaHops = sample_hop_snr(snrLin, numHops, hopMean, alpha, MC);

        gammaAF = gamma_af(gammaHops);
        gammaDF = gamma_df(gammaHops);
        gammaHY = gamma_hybrid(gammaAF, gammaDF, SNRdB(is));

        PoutAF(ik, is) = mean(gammaAF < gammaTh);
        PoutDF(ik, is) = mean(gammaDF < gammaTh);
        PoutHY(ik, is) = mean(gammaHY < gammaTh);

        % Throughput model (Mbps): mode-dependent sloting overhead.
        ThrAF(ik, is) = mean(eta * log2(1 + gammaAF) .* bwHz ./ (1e6 * (numHops + 0.40 * k)));
        ThrDF(ik, is) = mean(eta * log2(1 + gammaDF) .* bwHz ./ (1e6 * (numHops + 0.60 * k)));
        ThrHY(ik, is) = mean(eta * log2(1 + gammaHY) .* bwHz ./ (1e6 * (numHops + 0.30 * k)));
    end
end

% Comparative curves for k=4 (as in paper style)
kRef = 4;
ikRef = find(Kset == kRef, 1);

% Accuracy-vs-outage analysis (mode-selection correctness proxy)
accuracyLevels = 15:10:95;
nAcc = numel(accuracyLevels);
avgOutAF = zeros(1, nAcc);
avgOutDF = zeros(1, nAcc);
avgOutHY = zeros(1, nAcc);

snrNominalDb = 6;
snrNominalLin = 10^(snrNominalDb / 10);
gammaHopsNominal = sample_hop_snr(snrNominalLin, kRef + 1, totalDistanceM / (kRef + 1), alpha, MC);
gAfNom = gamma_af(gammaHopsNominal);
gDfNom = gamma_df(gammaHopsNominal);

for ia = 1:nAcc
    pCorrect = accuracyLevels(ia) / 100;

    avgOutAF(ia) = mean(gAfNom < gammaTh);
    avgOutDF(ia) = mean(gDfNom < gammaTh);
    avgOutHY(ia) = mean(hybrid_with_selection_accuracy(gAfNom, gDfNom, pCorrect) < gammaTh);
end

% Plot (single multi-panel figure similar to paper layout)
fig = figure("Color", "w", "Position", [80 40 1200 980]);
tiledlayout(3, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile;
hold on;
for ik = 1:nK
    plot(SNRdB, PoutAF(ik, :), "-o", "LineWidth", 1.2, "MarkerSize", 3);
end
hold off; grid on;
xlabel("Transmit SNR (dB)");
ylabel("Outage Probability (P_{out})");
title("Outage Analysis of AF-based UAV");
legend(compose("N_r=%d", Kset), "Location", "northeast");

nexttile;
semilogy(SNRdB, PoutHY(ikRef, :), "-o", "LineWidth", 1.7, "MarkerSize", 4); hold on;
semilogy(SNRdB, PoutAF(ikRef, :), "-*", "LineWidth", 1.4, "MarkerSize", 4);
semilogy(SNRdB, PoutDF(ikRef, :), "-s", "LineWidth", 1.4, "MarkerSize", 3.5); hold off;
grid on;
xlabel("Transmit SNR (dB)");
ylabel("Outage Probability (P_{out})");
title("Comparative Outage Analysis of Proposed Network");
legend("(Proposed) Hybrid, N_r=4", "AF, N_r=4", "DF, N_r=4", "Location", "northeast");

nexttile;
hold on;
for ik = 1:nK
    plot(SNRdB, PoutDF(ik, :), "-^", "LineWidth", 1.2, "MarkerSize", 3);
end
hold off; grid on;
xlabel("Transmit SNR (dB)");
ylabel("Outage Probability (P_{out})");
title("Outage Analysis of DF-based UAV");
legend(compose("N_r=%d", Kset), "Location", "northeast");

nexttile;
plot(SNRdB, ThrHY(ikRef, :), "-o", "LineWidth", 1.7, "MarkerSize", 4); hold on;
plot(SNRdB, ThrAF(ikRef, :), "-*", "LineWidth", 1.4, "MarkerSize", 4);
plot(SNRdB, ThrDF(ikRef, :), "-s", "LineWidth", 1.4, "MarkerSize", 3.5); hold off;
grid on;
xlabel("Transmit SNR (dB)");
ylabel("System Throughput (Mbps)");
title("Comparative Throughput of the System");
legend("(Proposed) Hybrid, N_r=4", "AF, N_r=4", "DF, N_r=4", "Location", "southeast");

nexttile;
hold on;
for ik = 1:nK
    plot(SNRdB, PoutHY(ik, :), "-d", "LineWidth", 1.2, "MarkerSize", 3);
end
hold off; grid on;
xlabel("Transmit SNR (dB)");
ylabel("Outage Probability (P_{out})");
title("Outage Analysis of Hybrid-based UAV (Proposed)");
legend(compose("N_r=%d", Kset), "Location", "northeast");

nexttile;
bar(accuracyLevels, [avgOutAF(:), avgOutDF(:), avgOutHY(:)], "grouped");
grid on;
xlabel("Accuracy (%)");
ylabel("Average Outage Probability");
title("Accuracy vs Outage Comparison");
legend("AF-Outage", "DF-Outage", "Hybrid-Outage", "Location", "northeast");

save_plot(fig, fullfile(outputDir, "paper_style_comm_plots.png"));

% Also save individual key figure for reports
fig2 = figure("Color", "w", "Position", [120 120 900 520]);
semilogy(SNRdB, PoutHY(ikRef, :), "-o", "LineWidth", 2.0); hold on;
semilogy(SNRdB, PoutAF(ikRef, :), "-*", "LineWidth", 1.7);
semilogy(SNRdB, PoutDF(ikRef, :), "-s", "LineWidth", 1.7); hold off;
grid on;
xlabel("Transmit SNR (dB)");
ylabel("Outage Probability");
title("Hybrid vs AF vs DF (N_r=4)");
legend("Hybrid", "AF", "DF", "Location", "northeast");
save_plot(fig2, fullfile(outputDir, "comparative_outage_k4.png"));

metrics = struct();
metrics.SNRdB = SNRdB;
metrics.Kset = Kset;
metrics.PoutAF = PoutAF;
metrics.PoutDF = PoutDF;
metrics.PoutHY = PoutHY;
metrics.ThrAF_Mbps = ThrAF;
metrics.ThrDF_Mbps = ThrDF;
metrics.ThrHY_Mbps = ThrHY;
metrics.Accuracy = accuracyLevels;
metrics.AvgOutage_AF = avgOutAF;
metrics.AvgOutage_DF = avgOutDF;
metrics.AvgOutage_HY = avgOutHY;

save(fullfile(outputDir, "paper_style_metrics.mat"), "metrics");
writematrix([SNRdB(:), PoutAF(ikRef, :)', PoutDF(ikRef, :)', PoutHY(ikRef, :)'], ...
    fullfile(outputDir, "k4_outage_curves.csv"));
writematrix([SNRdB(:), ThrAF(ikRef, :)', ThrDF(ikRef, :)', ThrHY(ikRef, :)'], ...
    fullfile(outputDir, "k4_throughput_curves.csv"));

fprintf("Saved plots and metrics in: %s\n", outputDir);
end

function gammaHops = sample_hop_snr(snrLin, numHops, hopMean, alpha, MC)
gammaHops = zeros(MC, numHops);
for h = 1:numHops
    d = hopMean .* (0.85 + 0.30 * rand(MC, 1));  % jittered hop lengths
    fading = exprnd(1, MC, 1);                   % Rayleigh power gain
    pathGain = (1000 ./ max(d, 1)).^alpha;
    gammaHops(:, h) = snrLin .* fading .* pathGain;
end
end

function g = gamma_af(gammaHops)
g = 1 ./ sum(1 ./ max(gammaHops, 1e-12), 2);
end

function g = gamma_df(gammaHops)
g = min(gammaHops, [], 2);
end

function g = gamma_hybrid(gAf, gDf, snrDb)
% SNR-aware blending and opportunistic selection.
if snrDb < 0
    w = 0.35;    % prefer DF at low SNR
elseif snrDb < 10
    w = 0.50;
else
    w = 0.65;    % prefer AF more at higher SNR
end

gBlend = w * max(gAf, gDf) + (1 - w) * 0.5 * (gAf + gDf);
g = max(gBlend, max(gAf, gDf));  % hybrid should not be worse than best branch
end

function gSel = hybrid_with_selection_accuracy(gAf, gDf, pCorrect)
gBest = max(gAf, gDf);
gWorst = min(gAf, gDf);
pickBest = rand(size(gAf)) < pCorrect;
gSel = gWorst;
gSel(pickBest) = gBest(pickBest);
end

function save_plot(fig, outPath)
if exist("exportgraphics", "file")
    exportgraphics(fig, outPath, "Resolution", 300);
else
    saveas(fig, outPath);
end
end
