clc; clear; close all;

%% ================= CONFIGURATION =================
outDir = "relay_coverage_latest";
if ~exist(outDir,'dir')
    mkdir(outDir);
end

rng(10);   % Reproducibility

numRelays = 8;
numUsers  = 600;           % Enough users to induce load
areaSize  = 50000;         % 50 km x 50 km area (meters)

radii_km = 1:10;
radii_m  = radii_km * 1000;

%% ================= PHY PARAMETERS =================
Pt_dBm = 30;                          % UAV Tx power
Pt     = 10^((Pt_dBm-30)/10);         % Watts

Noise_dBm = -100;
Noise = 10^((Noise_dBm-30)/10);       % Watts

BW = 10e6;                            % 10 MHz total per relay
pathLossExp = 2.4;                    % Urban A2G
PL0_dB = 30;                          % Reference PL

SINR_thresh_dB = 5;
SINR_thresh = 10^(SINR_thresh_dB/10);

%% ================= DEPLOYMENT =================
RelayXY = areaSize * rand(numRelays,2);
UserXY  = areaSize * rand(numUsers,2);

%% ================= STORAGE =================
coveragePct   = zeros(size(radii_m));
uncoveredPct  = zeros(size(radii_m));
avgSINR_dB    = zeros(size(radii_m));
avgThroughput = zeros(size(radii_m));

%% ================= MAIN LOOP =================
for r = 1:length(radii_m)

    radius = radii_m(r);

    usersPerRelay = zeros(numRelays,1);
    servingRelay  = zeros(numUsers,1);

    % ---------- Association Phase ----------
    for u = 1:numUsers
        dists = vecnorm(RelayXY - UserXY(u,:), 2, 2);
        [dmin, idx] = min(dists);

        if dmin <= radius
            servingRelay(u) = idx;
            usersPerRelay(idx) = usersPerRelay(idx) + 1;
        end
    end

    coveredUsers = 0;
    sinrVals = [];
    thrVals  = [];

    % ---------- SINR & Throughput ----------
    for u = 1:numUsers

        idx = servingRelay(u);
        if idx == 0
            continue
        end

        d0 = norm(RelayXY(idx,:) - UserXY(u,:));

        % Desired signal
        PL_dB = PL0_dB + 10*pathLossExp*log10(d0);
        Pr = Pt * 10^(-PL_dB/10);

        % Interference
        interf = 0;
        for k = 1:numRelays
            if k ~= idx
                dk = norm(RelayXY(k,:) - UserXY(u,:));
                PLi_dB = PL0_dB + 10*pathLossExp*log10(dk);
                interf = interf + Pt * 10^(-PLi_dB/10);
            end
        end

        SINR = Pr / (interf + Noise);

        if SINR >= SINR_thresh
            coveredUsers = coveredUsers + 1;

            % --------- LOAD-AWARE BANDWIDTH ---------
            BW_eff = BW / usersPerRelay(idx);

            sinrVals(end+1) = 10*log10(SINR);
            thrVals(end+1)  = BW_eff * log2(1 + SINR) / 1e6;
        end
    end

    coveragePct(r)   = coveredUsers / numUsers * 100;
    uncoveredPct(r)  = 100 - coveragePct(r);
    avgSINR_dB(r)    = mean(sinrVals);
    avgThroughput(r) = mean(thrVals);
end

%% ================= PLOTS =================

figure;
plot(radii_km, coveragePct,'-o','LineWidth',2);
xlabel("Coverage Radius (km)");
ylabel("Coverage (%)");
title("Coverage % vs Relay Radius");
grid on;
saveas(gcf, outDir + "/coverage_vs_radius.png");

figure;
plot(radii_km, uncoveredPct,'-s','LineWidth',2);
xlabel("Coverage Radius (km)");
ylabel("Uncovered (%)");
title("Uncovered % vs Relay Radius");
grid on;
saveas(gcf, outDir + "/uncovered_vs_radius.png");

figure;
plot(radii_km, avgSINR_dB,'-^','LineWidth',2);
xlabel("Coverage Radius (km)");
ylabel("Average SINR (dB)");
title("Average SINR vs Radius");
grid on;
saveas(gcf, outDir + "/sinr_vs_radius.png");

figure;
plot(radii_km, avgThroughput,'-d','LineWidth',2);
xlabel("Coverage Radius (km)");
ylabel("Average Throughput (Mbps)");
title("Throughput vs Radius");
grid on;
saveas(gcf, outDir + "/throughput_vs_radius.png");

disp("✅ Load-aware SINR & throughput simulation completed.");