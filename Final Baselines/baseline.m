clc; clear; close all;

%% ================== BASELINES ==================
% B1: Direct Link (No Relay)
% B2: Single UAV DF Relay
% B3: Two-Hop Ground Routing
% B4: Single UAV AF Relay

%% ================== CHANNEL PARAMETERS ==================
Pt_dBm    = 30;        % Transmit power (dBm)
Noise_dBm = -100;      % Noise floor (dBm)
fc        = 2.4e9;     % Carrier frequency (Hz)
SINR_th   = 10;        % Outage threshold (dB)

%% ================== CHANNEL MODEL ==================
PL  = @(d) 32.4 + 20*log10(fc/1e9) + 20*log10(d*1000); % FSPL (d in km)
SNR = @(d) Pt_dBm - PL(d) - Noise_dBm;

% Reliability model (success probability)
Psucc = @(snr_dB) exp(-1 ./ (10.^(snr_dB/10)));

%% ================== DISTANCE RANGE ==================
dvals = linspace(5, 40, 30);   % km

%% ================== STORAGE ==================
SINR_direct = zeros(size(dvals));
SINR_df     = zeros(size(dvals));
SINR_twohop = zeros(size(dvals));
SINR_af     = zeros(size(dvals));

P_direct = zeros(size(dvals));
P_df     = zeros(size(dvals));
P_twohop = zeros(size(dvals));
P_af     = zeros(size(dvals));

Out_direct = zeros(size(dvals));
Out_df     = zeros(size(dvals));
Out_twohop = zeros(size(dvals));
Out_af     = zeros(size(dvals));

%% ================== COMPUTATION ==================
for i = 1:length(dvals)
    d = dvals(i);

    %% ---------- B1: Direct ----------
    SINR_direct(i) = SNR(d);
    P_direct(i)    = Psucc(SINR_direct(i));
    Out_direct(i)  = SINR_direct(i) < SINR_th;

    %% ---------- B2: Single UAV DF ----------
    SINR_su = SNR(d/2);
    SINR_ud = SNR(d/2);
    SINR_df(i) = min(SINR_su, SINR_ud);

    P_df(i)   = min(Psucc(SINR_su), Psucc(SINR_ud));
    Out_df(i) = SINR_df(i) < SINR_th;

    %% ---------- B3: Two-Hop Ground ----------
    d1 = 0.4 * d;
    d2 = 0.6 * d;
    SINR_twohop(i) = min(SNR(d1), SNR(d2));

    P_twohop(i)   = Psucc(SNR(d1)) * Psucc(SNR(d2));
    Out_twohop(i) = SINR_twohop(i) < SINR_th;

    %% ---------- B4: Single UAV AF ----------
    s1 = 10^(SNR(d/2)/10);
    s2 = 10^(SNR(d/2)/10);
    SINR_af_lin = (s1 * s2) / (s1 + s2 + 1);
    SINR_af(i)  = 10 * log10(SINR_af_lin);

    P_af(i)   = Psucc(SINR_af(i));
    Out_af(i) = SINR_af(i) < SINR_th;
end

%% ================== PLOT 1: SINR ==================
fig1 = figure('Color','w','Position',[200 200 820 480]);
plot(dvals, SINR_direct,'-o','LineWidth',2); hold on;
plot(dvals, SINR_df,    '-s','LineWidth',2);
plot(dvals, SINR_twohop,'-^','LineWidth',2);
plot(dvals, SINR_af,    '-d','LineWidth',2);

yline(SINR_th,'r--','Outage Threshold','LineWidth',1.5);
grid on;
xlabel('Source–Destination Distance (km)');
ylabel('Average End-to-End SINR (dB)');
title('SINR Comparison of UAV-Assisted Baselines');
legend('Direct','UAV DF','Two-Hop Ground','UAV AF','Location','southwest');

%% ================== PLOT 2: OUTAGE PROBABILITY ==================
fig2 = figure('Color','w','Position',[220 220 820 480]);
plot(dvals, Out_direct,'-o','LineWidth',2); hold on;
plot(dvals, Out_df,    '-s','LineWidth',2);
plot(dvals, Out_twohop,'-^','LineWidth',2);
plot(dvals, Out_af,    '-d','LineWidth',2);

grid on;
xlabel('Source–Destination Distance (km)');
ylabel('Outage Probability');
title('Outage Probability Comparison');
ylim([0 1]);
legend('Direct','UAV DF','Two-Hop Ground','UAV AF','Location','northwest');

%% ================== PLOT 3: RELIABILITY ==================
fig3 = figure('Color','w','Position',[240 240 820 480]);
plot(dvals, P_direct,'-o','LineWidth',2); hold on;
plot(dvals, P_df,    '-s','LineWidth',2);
plot(dvals, P_twohop,'-^','LineWidth',2);
plot(dvals, P_af,    '-d','LineWidth',2);

grid on;
xlabel('Source–Destination Distance (km)');
ylabel('Reliability (Success Probability)');
title('Reliability Comparison of Baselines');
ylim([0 1]);
legend('Direct','UAV DF','Two-Hop Ground','UAV AF','Location','southwest');

%% ================== SAVE RESULTS ==================
outDir = "results";
if ~exist(outDir,"dir"), mkdir(outDir); end

saveas(fig1, fullfile(outDir,"Baseline_SINR_Comparison.png"));
saveas(fig2, fullfile(outDir,"Baseline_Outage_Comparison.png"));
saveas(fig3, fullfile(outDir,"Baseline_Reliability_Comparison.png"));

disp("✅ SINR, Outage, and Reliability plots generated and saved.");