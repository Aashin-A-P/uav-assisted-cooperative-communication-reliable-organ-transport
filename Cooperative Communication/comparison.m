clc; clear; close all;
rng(10);

%% ================== CONFIG ==================
numHops = 8;
MC = 12000;

SNRdB = -20:2:20;
SNRlin = 10.^(SNRdB/10);

gamma_th_dB = 3;                 % outage threshold
gamma_th = 10^(gamma_th_dB/10);

gamma_sw_dB = 8;                 % hybrid switching threshold
gamma_sw = 10^(gamma_sw_dB/10);

alpha = 3.2;                     % path-loss exponent
df_gain = 1.15;                  % DF regeneration gain (cleaning/coding gain)

% Slot/processing cost model (key for hybrid benefit)
t_AF = 1;                        % AF hop cost
t_DF = 2;                        % DF hop cost (decode+re-encode overhead)

%% ================== OUTPUTS ==================
Pout_AF = zeros(size(SNRlin));
Pout_DF = zeros(size(SNRlin));
Pout_HY = zeros(size(SNRlin));

Good_AF = zeros(size(SNRlin));
Good_DF = zeros(size(SNRlin));
Good_HY = zeros(size(SNRlin));

Delay_AF = zeros(size(SNRlin));
Delay_DF = zeros(size(SNRlin));
Delay_HY = zeros(size(SNRlin));

%% ================== MONTE CARLO ==================
for s = 1:length(SNRlin)

    outAF = 0; outDF = 0; outHY = 0;

    sumGoodAF = 0; sumGoodDF = 0; sumGoodHY = 0;
    sumDelAF  = 0; sumDelDF  = 0; sumDelHY  = 0;

    for mc = 1:MC

        %% ---- Build heterogeneous hops (strong + weak) ----
        gamma_hops = zeros(numHops,1);
        for h = 1:numHops
            fading = exprnd(1);
            if h == 3 || h == 6
                d = 5200;     % weak hop
            else
                d = 1200;     % strong hop
            end
            gamma_hops(h) = SNRlin(s) * fading * d^(-alpha);
        end

        %% ---- AF end-to-end (recursive) ----
        gAF = gamma_hops(1);
        for i = 2:numHops
            gAF = (gAF * gamma_hops(i)) / (gAF + gamma_hops(i) + 1);
        end

        %% ---- DF end-to-end (bottleneck + regen gain) ----
        gDF = df_gain * min(gamma_hops);

        %% ---- HYBRID: DF on weak hops, AF on good hops ----
        mode = ones(numHops,1);               % 1=AF, 0=DF
        mode(gamma_hops < gamma_sw) = 0;

        % Hybrid equivalent SNR:
        % AF segments collapse via AF recursion; DF hops are clean links.
        bottlenecks = [];

        % DF hops are direct bottlenecks (after regen gain)
        if any(mode==0)
            bottlenecks = [bottlenecks; df_gain * gamma_hops(mode==0)];
        end

        % AF segments
        idx = 1;
        while idx <= numHops
            if mode(idx)==0
                idx = idx + 1;
            else
                seg = [];
                while idx <= numHops && mode(idx)==1
                    seg = [seg; gamma_hops(idx)];
                    idx = idx + 1;
                end
                gseg = seg(1);
                for k = 2:length(seg)
                    gseg = (gseg * seg(k)) / (gseg + seg(k) + 1);
                end
                bottlenecks = [bottlenecks; gseg];
            end
        end

        gHY = min(bottlenecks);

        %% ---- Outage events (SNR-based) ----
        if gAF < gamma_th, outAF = outAF + 1; end
        if gDF < gamma_th, outDF = outDF + 1; end
        if gHY < gamma_th, outHY = outHY + 1; end

        %% ---- Delay model (time slots) ----
        D_AF = numHops * t_AF;
        D_DF = numHops * t_DF;
        D_HY = sum(mode==1)*t_AF + sum(mode==0)*t_DF;

        %% ---- Goodput model (bits/s/Hz per time-slot) ----
        % if outage -> goodput = 0 for that run
        rAF = log2(1 + gAF) / D_AF;
        rDF = log2(1 + gDF) / D_DF;
        rHY = log2(1 + gHY) / D_HY;

        if gAF < gamma_th, rAF = 0; end
        if gDF < gamma_th, rDF = 0; end
        if gHY < gamma_th, rHY = 0; end

        sumGoodAF = sumGoodAF + rAF;
        sumGoodDF = sumGoodDF + rDF;
        sumGoodHY = sumGoodHY + rHY;

        sumDelAF = sumDelAF + D_AF;
        sumDelDF = sumDelDF + D_DF;
        sumDelHY = sumDelHY + D_HY;
    end

    Pout_AF(s) = outAF/MC;
    Pout_DF(s) = outDF/MC;
    Pout_HY(s) = outHY/MC;

    Good_AF(s) = sumGoodAF/MC;
    Good_DF(s) = sumGoodDF/MC;
    Good_HY(s) = sumGoodHY/MC;

    Delay_AF(s) = sumDelAF/MC;
    Delay_DF(s) = sumDelDF/MC;
    Delay_HY(s) = sumDelHY/MC;
end

%% ================== FIG 1: OUTAGE ==================
figure('Color','w','Position',[120 120 900 550]);
semilogy(SNRdB, Pout_AF, '-o','LineWidth',2); hold on;
semilogy(SNRdB, Pout_DF, '-s','LineWidth',2);
semilogy(SNRdB, Pout_HY, '-^','LineWidth',2);
grid on;
xlabel('Transmit SNR (dB)');
ylabel('Outage Probability (SNR < \gamma_{th})');
title('Outage Probability: AF vs DF vs Hybrid');
legend({'AF','DF','Hybrid AF-DF'}, 'Location','southwest');

%% ================== FIG 2: GOODPUT (THIS WILL SEPARATE) ==================
figure('Color','w','Position',[120 120 900 550]);
plot(SNRdB, Good_AF, '-o','LineWidth',2); hold on;
plot(SNRdB, Good_DF, '-s','LineWidth',2);
plot(SNRdB, Good_HY, '-^','LineWidth',2);
grid on;
xlabel('Transmit SNR (dB)');
ylabel('Goodput (bits/s/Hz per time-slot)');
title('Goodput Comparison (Reliability + Delay): AF vs DF vs Hybrid');
legend({'AF','DF','Hybrid AF-DF'}, 'Location','northwest');

%% ================== FIG 3: DELAY (OPTIONAL) ==================
figure('Color','w','Position',[120 120 900 550]);
plot(SNRdB, Delay_AF, '-o','LineWidth',2); hold on;
plot(SNRdB, Delay_DF, '-s','LineWidth',2);
plot(SNRdB, Delay_HY, '-^','LineWidth',2);
grid on;
xlabel('Transmit SNR (dB)');
ylabel('End-to-End Delay (time-slot units)');
title('Delay Cost: AF vs DF vs Hybrid');
legend({'AF','DF','Hybrid AF-DF'}, 'Location','northeast');
