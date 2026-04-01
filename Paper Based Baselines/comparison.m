%% ================== AF vs DF COMPARISON (MONTE CARLO) ==================
SNRdB = -10:2:20;
SNRlin = 10.^(SNRdB/10);

MC = 1e4;
gamma_th = 1;          % 0 dB outage threshold

Pout_AF = zeros(size(SNRlin));
Pout_DF = zeros(size(SNRlin));

for s = 1:length(SNRlin)
    outAF = 0;
    outDF = 0;

    for mc = 1:MC
        gamma_hops = zeros(numHops,1);

        for h = 1:numHops
            d = norm(hopXY(h+1,:) - hopXY(h,:));
            fading = exprnd(1);

            if h == 1
                % access hop scaled by transmit SNR
                gamma_hops(h) = SNRlin(s) * fading * d^(-alpha_access);
            else
                % backhaul hop scaled by transmit SNR
                gamma_hops(h) = SNRlin(s) * fading * d^(-alpha_backhaul);
            end
        end

        gamma_AF = 1 / sum(1 ./ gamma_hops);
        gamma_DF = min(gamma_hops);

        if gamma_AF < gamma_th, outAF = outAF + 1; end
        if gamma_DF < gamma_th, outDF = outDF + 1; end
    end

    Pout_AF(s) = outAF / MC;
    Pout_DF(s) = outDF / MC;
end

%% ================== PLOT OUTAGE COMPARISON ==================
figure('Color','w');
semilogy(SNRdB, Pout_AF, '-o','LineWidth',2); hold on;
semilogy(SNRdB, Pout_DF, '-s','LineWidth',2);
grid on;
xlabel('Transmit SNR (dB)');
ylabel('Outage Probability');
title('AF vs DF Baseline Comparison (Trajectory-based Multi-hop)');
legend('AF','DF','Location','southwest');