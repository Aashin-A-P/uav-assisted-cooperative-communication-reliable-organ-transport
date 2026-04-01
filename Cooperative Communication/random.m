clc; clear; close all;

%% ================== RANDOM HYBRID AF–DF SIMULATION ==================
rng(7);

%% ------------------ CONFIG ------------------
area_km = 50;
numHosp = 15;
numRelay = 17;
accessRange = 7000;
step_m = 500;
switchMargin = 250;

alpha_access = 3.0;
alpha_backhaul = 2.7;
gain_access = 1.0;
gain_backhaul = 2.0;

txSNR_dB = 0;
txSNR = 10^(txSNR_dB/10);

gamma_sw_dB = 5;
gamma_sw = 10^(gamma_sw_dB/10);

gamma_th_dB = 0;
gamma_th = 10^(gamma_th_dB/10);

makeVideo = true;
videoName = 'RANDOM_HYBRID_AF_DF.mp4';
fps = 12;

%% ------------------ RANDOM HOSPITALS & RELAYS ------------------
L = area_km * 1000;

HospXY  = [rand(numHosp,1)*L rand(numHosp,1)*L];
RelayXY = [rand(numRelay,1)*L rand(numRelay,1)*L];

srcHosp = randi(numHosp);
dstHosp = randi(numHosp);
while dstHosp == srcHosp
    dstHosp = randi(numHosp);
end

srcXY = HospXY(srcHosp,:);
dstXY = HospXY(dstHosp,:);

%% ------------------ DELIVERY UAV TRAJECTORY ------------------
distTot = norm(dstXY - srcXY);
numSteps = max(2, ceil(distTot/step_m) + 1);

trajX = linspace(srcXY(1), dstXY(1), numSteps);
trajY = linspace(srcXY(2), dstXY(2), numSteps);
traj = [trajX(:) trajY(:)];

%% ------------------ RELAY ASSOCIATION ------------------
connectedRelay = nan(numSteps,1);
currentRelay = nan;

for t = 1:numSteps
    p = traj(t,:);
    dists = vecnorm(RelayXY - p,2,2);
    [dmin, idxMin] = min(dists);

    if isnan(currentRelay)
        if dmin <= accessRange
            currentRelay = idxMin;
        end
    else
        dCur = dists(currentRelay);
        if dCur > accessRange && dmin <= accessRange
            currentRelay = idxMin;
        elseif idxMin ~= currentRelay && (dmin + switchMargin < dCur)
            currentRelay = idxMin;
        end
    end
    connectedRelay(t) = currentRelay;
end

relaySeq = connectedRelay(~isnan(connectedRelay));
relaySeq = relaySeq([true; diff(relaySeq)~=0]);

if isempty(relaySeq)
    error('No relay association occurred.');
end

%% ------------------ HOP CHAIN ------------------
hopXY = [srcXY; RelayXY(relaySeq,:); dstXY];
numHops = size(hopXY,1) - 1;

%% ------------------ PER-HOP SNR ------------------
gamma_hops = zeros(numHops,1);

for h = 1:numHops
    d = norm(hopXY(h+1,:) - hopXY(h,:));
    fading = exprnd(1);

    if h == 1
        gamma_hops(h) = txSNR * gain_access * fading * d^(-alpha_access);
    else
        gamma_hops(h) = txSNR * gain_backhaul * fading * d^(-alpha_backhaul);
    end
end

%% ------------------ AF / DF / HYBRID ------------------
gamma_AF = gamma_hops(1);
for i = 2:numHops
    gamma_AF = (gamma_AF * gamma_hops(i)) / (gamma_AF + gamma_hops(i) + 1);
end

gamma_DF = min(gamma_hops);

modePerHop = ones(numHops,1);
modePerHop(gamma_hops < gamma_sw) = 0;

if any(modePerHop==0)
    gamma_DF_part = min(gamma_hops(modePerHop==0));
else
    gamma_DF_part = inf;
end

idxAF = find(modePerHop==1);
if isempty(idxAF)
    gamma_AF_part = 0;
else
    gamma_AF_part = gamma_hops(idxAF(1));
    for k = 2:length(idxAF)
        i = idxAF(k);
        gamma_AF_part = (gamma_AF_part * gamma_hops(i)) / ...
                        (gamma_AF_part + gamma_hops(i) + 1);
    end
end

gamma_HYB = min(gamma_DF_part, gamma_AF_part);
if isinf(gamma_DF_part)
    gamma_HYB = gamma_AF_part;
end

%% ------------------ PRINT RESULTS ------------------
fprintf('\n===== End-to-End SNR =====\n');
fprintf('AF     : %.2f dB\n',10*log10(gamma_AF));
fprintf('DF     : %.2f dB\n',10*log10(gamma_DF));
fprintf('Hybrid : %.2f dB\n',10*log10(gamma_HYB));

%% ------------------ VISUALIZATION ------------------
figure('Color','w','Position',[100 80 1400 800]);
hold on; grid on; axis equal;
xlim([0 L]); ylim([0 L]);
xlabel('X (m)'); ylabel('Y (m)');
title('Random Hybrid AF–DF Cooperative Communication');

scatter(HospXY(:,1),HospXY(:,2),90,'y','filled');
for i=1:numHosp
    text(HospXY(i,1)+200,HospXY(i,2)+200,sprintf('H%d',i));
end

scatter(RelayXY(:,1),RelayXY(:,2),60,'b','filled');
for i=1:numRelay
    text(RelayXY(i,1)+200,RelayXY(i,2)+200,sprintf('R%d',i));
end

plot(traj(:,1),traj(:,2),'k--','LineWidth',1.5);
scatter(srcXY(1),srcXY(2),150,'g','filled');
scatter(dstXY(1),dstXY(2),150,'m','filled');

for h = 1:numHops
    if modePerHop(h)==1
        plot(hopXY(h:h+1,1),hopXY(h:h+1,2),'b-','LineWidth',3);
    else
        plot(hopXY(h:h+1,1),hopXY(h:h+1,2),'Color',[1 0.5 0],'LineWidth',3);
    end
end

legend({'Hospitals','Relays','Trajectory','AF Hop','DF Hop'}, ...
       'Location','northoutside');

%% ------------------ VIDEO ------------------
if makeVideo
    v = VideoWriter(videoName,'MPEG-4');
    v.FrameRate = fps;
    open(v);
    writeVideo(v,getframe(gcf));
    close(v);
    fprintf('MP4 saved: %s\n',videoName);
end
