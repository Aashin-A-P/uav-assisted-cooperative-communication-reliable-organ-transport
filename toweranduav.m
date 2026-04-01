clc; clear; close all;

%% ===================== SETTINGS =====================
Kmax = 25;
gridStep_km = 2.0;

Rtarget = 0.85;           % target reliability threshold (for coverage)
coverageRadius_km = 6;    % same idea you used earlier

outDir = "better_metrics_results";
if ~exist(outDir,"dir"), mkdir(outDir); end

%% ===================== LOAD NODES =====================
[names, lat, lon, airportLat, airportLon, HospitalsXY] = hospital_data();

R = 6371000;
lat0 = mean(lat)*pi/180;
lon0 = mean(lon)*pi/180;

xA = R*((airportLon*pi/180)-lon0)*cos(lat0);
yA = R*((airportLat*pi/180)-lat0);

NodesXY = [HospitalsXY; xA yA];
numNodes = size(NodesXY,1);

%% ===================== LAND CANDIDATES =====================
idxHull = convhull(NodesXY(:,1),NodesXY(:,2));
landPoly = polyshape(NodesXY(idxHull,1),NodesXY(idxHull,2));

gs = gridStep_km*1000;
[xg,yg] = meshgrid(min(NodesXY(:,1)):gs:max(NodesXY(:,1)), ...
                   min(NodesXY(:,2)):gs:max(NodesXY(:,2)));
Cand = [xg(:) yg(:)];
CandXY = Cand(isinterior(landPoly,Cand(:,1),Cand(:,2)),:);

%% ===================== STORAGE =====================
Ks = 1:Kmax;

coverageRatio = zeros(size(Ks));      % metric-1
worstNodeDist_km = zeros(size(Ks));   % metric-2
p5Reliability = zeros(size(Ks));      % metric-3

%% ===================== MAIN LOOP =====================
for kIdx = 1:numel(Ks)
    K = Ks(kIdx);

    % Place relays
    RelayXY = greedy_kcenter(CandXY,K,mean(NodesXY,1));

    % -------- Metric-2: worst-case node distance to nearest relay ----------
    dNode = zeros(numNodes,1);
    for n = 1:numNodes
        dNode(n) = min(vecnorm(RelayXY - NodesXY(n,:),2,2));
    end
    worstNodeDist_km(kIdx) = max(dNode)/1000;

    % -------- Metric-1: Coverage ratio over land points ----------
    % reliability(point) = exp(-dNearest/rc)
    dGrid = zeros(size(CandXY,1),1);
    for p = 1:size(CandXY,1)
        dGrid(p) = min(vecnorm(RelayXY - CandXY(p,:),2,2));
    end
    relGrid = exp(-(dGrid)/(coverageRadius_km*1000));
    coverageRatio(kIdx) = mean(relGrid >= Rtarget);

    % -------- Metric-3: 5th percentile end-to-end reliability ----------
    vals = [];
    for i = 1:numNodes
        for j = i+1:numNodes
            d1 = min(vecnorm(RelayXY - NodesXY(i,:),2,2));
            d2 = min(vecnorm(RelayXY - NodesXY(j,:),2,2));
            rel_ij = exp(-(d1+d2)/(coverageRadius_km*1000)); % e2e
            vals(end+1) = rel_ij;
        end
    end
    p5Reliability(kIdx) = prctile(vals,5);
end

%% ===================== PLOT-1: COVERAGE RATIO =====================
fig1 = figure('Color','w');
plot(Ks, coverageRatio, '-o', 'LineWidth', 2);
grid on;
xlabel('Number of Relays (K)');
ylabel('Coverage Ratio');
title(sprintf('Coverage Ratio vs K (Threshold R \\ge %.2f)', Rtarget));
saveas(fig1, fullfile(outDir,"CoverageRatio_vs_K.png"));
savefig(fig1, fullfile(outDir,"CoverageRatio_vs_K.fig"));

%% ===================== PLOT-2: WORST NODE DISTANCE =====================
fig2 = figure('Color','w');
plot(Ks, worstNodeDist_km, '-s', 'LineWidth', 2);
grid on;
xlabel('Number of Relays (K)');
ylabel('Worst-case Distance to Nearest Relay (km)');
title('Worst-case Node Distance vs K (k-center radius)');
saveas(fig2, fullfile(outDir,"WorstNodeDistance_vs_K.png"));
savefig(fig2, fullfile(outDir,"WorstNodeDistance_vs_K.fig"));

%% ===================== PLOT-3: 5th PERCENTILE RELIABILITY =====================
fig3 = figure('Color','w');
plot(Ks, p5Reliability, '-^', 'LineWidth', 2); hold on;
yline(Rtarget,'r--','Target');
grid on;
xlabel('Number of Relays (K)');
ylabel('5th Percentile End-to-End Reliability');
title('Tail Reliability (5th Percentile) vs K');
legend('p5 Reliability','Target','Location','southeast');
saveas(fig3, fullfile(outDir,"P5Reliability_vs_K.png"));
savefig(fig3, fullfile(outDir,"P5Reliability_vs_K.fig"));

%% ===================== SAVE DATA =====================
save(fullfile(outDir,"BetterMetrics_Data.mat"), ...
     "Ks","coverageRatio","worstNodeDist_km","p5Reliability", ...
     "Rtarget","coverageRadius_km","gridStep_km");

disp("✅ Saved 3 better-metric plots + .mat in: " + outDir);

%% ===================== LOCAL FUNCTION =====================
function RelayXY = greedy_kcenter(CandXY,K,seed)
    RelayXY=zeros(K,2);
    RelayXY(1,:)=seed;
    d=vecnorm(CandXY-seed,2,2);
    for i=2:K
        [~,idx]=max(d);
        RelayXY(i,:)=CandXY(idx,:);
        d=min(d,vecnorm(CandXY-RelayXY(i,:),2,2));
    end
end