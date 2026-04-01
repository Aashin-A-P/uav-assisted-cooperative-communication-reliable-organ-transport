function plot_handoff_timeline(bestRelay, bestSNR)

T = numel(bestRelay);

figure('Color','w','Position',[120 120 1200 480]);

yyaxis left
plot(1:T, bestRelay, '-o','LineWidth',1.2);
ylabel('Selected Access Relay Index');

yyaxis right
plot(1:T, bestSNR, '-','LineWidth',1.2);
ylabel('Access Link SNR (dB)');

xlabel('Time Step');
title('Access Relay Handoff Timeline');
grid on;

end