clc; clear; close all;

%% ================= PARAMETERS =================
Kmax = 20;                 % Max number of relay UAVs to test
coverageRadius_km = 6;     % Effective UAV coverage radius
targetReliability = 0.85;  % Required worst-case reliability

%% ================= LOAD DATA =================
% hospital_data() must return:
% names, lat, lon, airportLat, airportLon, HospitalsXY (in meters)
[names, lat, lon, airportLat, airportLon, HospitalsXY] = hospital_data();

%% ================= ADD AIRPORT =================
R = 6371000;
lat0 = mean(lat) * pi/180;
lon0 = mean(lon) * pi/180;

xA = R * ((airportLon*pi/180) - lon0) * cos(lat0);
yA = R * ((airportLat*pi/180) - lat0);

NodesXY = [HospitalsXY; xA yA];
numNodes = size(NodesXY,1);

%% ================= STORAGE =================
Ks       = 1:Kmax;
avgRel   = zeros(size(Ks));
worstRel = zeros(size(Ks));
avgOut   = zeros(size(Ks));
worstOut = zeros(size(Ks));

%% ================= MAIN ANALYSIS LOOP =================
for kIdx = 1:length(Ks)

    K = Ks(kIdx);

    % Place K relays using greedy k-center
    RelayXY = greedy_kcenter(NodesXY, K);

    reliabilities = zeros(numNodes);

    for i = 1:numNodes
        for j = 1:numNodes
            if i == j
                reliabilities(i,j) = 1;
                continue;
            end

            % Distance via nearest relay
            d1 = min(vecnorm(RelayXY - NodesXY(i,:),2,2));
            d2 = min(vecnorm(RelayXY - NodesXY(j,:),2,2));

            % Reliability model
            rel_ij = exp(-(d1 + d2)/(coverageRadius_km*1000));
            reliabilities(i,j) = min(rel_ij,1);
        end
    end

    vals = reliabilities(~eye(numNodes));

    avgRel(kIdx)   = mean(vals);
    worstRel(kIdx) = min(vals);
    avgOut(kIdx)   = mean(1 - vals);
    worstOut(kIdx) = max(1 - vals);
end

%% ================= CREATE OUTPUT FOLDER =================
outDir = "relay_count_analysis_results";
if ~exist(outDir,"dir")
    mkdir(outDir);
end

%% ================= PLOT: RELIABILITY =================
fig1 = figure('Color','w');
plot(Ks, avgRel, '-o','LineWidth',2); hold on;
plot(Ks, worstRel, '-s','LineWidth',2);
yline(targetReliability,'r--','Target','LineWidth',1.5);

grid on;
xlabel('Number of Relay UAVs (K)');
ylabel('Reliability');
title('Reliability vs Number of Relay UAVs');
legend('Average Reliability','Worst-case Reliability','Target',...
       'Location','southeast');

saveas(fig1, fullfile(outDir,"Reliability_vs_K.png"));
savefig(fig1, fullfile(outDir,"Reliability_vs_K.fig"));

%% ================= PLOT: OUTAGE =================
fig2 = figure('Color','w');
plot(Ks, avgOut, '-o','LineWidth',2); hold on;
plot(Ks, worstOut, '-s','LineWidth',2);

grid on;
xlabel('Number of Relay UAVs (K)');
ylabel('Outage Probability');
title('Outage Probability vs Number of Relay UAVs');
legend('Average Outage','Worst-case Outage',...
       'Location','northeast');

saveas(fig2, fullfile(outDir,"Outage_vs_K.png"));
savefig(fig2, fullfile(outDir,"Outage_vs_K.fig"));

%% ================= SAVE NUMERICAL DATA =================
save(fullfile(outDir,"RelayCount_Analysis_Data.mat"), ...
     "Ks","avgRel","worstRel","avgOut","worstOut","targetReliability");

%% ================= DECISION =================
idx = find(worstRel >= targetReliability,1,'first');

if isempty(idx)
    disp("❌ Target not met up to Kmax");
else
    fprintf("✅ REQUIRED NUMBER OF RELAY UAVs: K = %d\n", Ks(idx));
    fprintf("Worst-case Reliability = %.2f\n", worstRel(idx));
    fprintf("Average Outage = %.2f\n", avgOut(idx));
end

disp("📁 Results saved in folder: relay_count_analysis_results");

%% ================= LOCAL FUNCTION =================
function RelayXY = greedy_kcenter(NodesXY, K)
    RelayXY = zeros(K,2);
    RelayXY(1,:) = mean(NodesXY,1);

    dist = vecnorm(NodesXY - RelayXY(1,:),2,2);

    for i = 2:K
        [~,idx] = max(dist);
        RelayXY(i,:) = NodesXY(idx,:);
        newDist = vecnorm(NodesXY - RelayXY(i,:),2,2);
        dist = min(dist,newDist);
    end
end