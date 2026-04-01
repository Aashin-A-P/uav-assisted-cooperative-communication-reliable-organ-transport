function animate_smooth_drl_with_handoff(agentFile)

clc; close all;

load(agentFile,"agent");

ENV0 = environment_engine_cached();
P    = get_comm_params();

%% ================= CONFIG =================
cfg.stepMeters_nom = 200;
cfg.uavSpeedMS     = 25;
cfg.dtSec          = cfg.stepMeters_nom / cfg.uavSpeedMS;
cfg.maxSteps       = 350;

SINR_min = 5;   % 🔥 minimum acceptable SINR

allLat = [ENV0.Hospitals.Latitude; ENV0.Relays.Latitude];
allLon = [ENV0.Hospitals.Longitude; ENV0.Relays.Longitude];

cfg.latMin = min(allLat) - 0.01;
cfg.latMax = max(allLat) + 0.01;
cfg.lonMin = min(allLon) - 0.01;
cfg.lonMax = max(allLon) + 0.01;

%% ================= CONTROL CENTER =================
CTRL.lat = 12.9965918;
CTRL.lon = 80.1708076;

%% ================= BATTERY =================
energy_total = 100000;      % Joules
energy_per_meter = 1.25;

energy_remaining = energy_total;
battery_remaining = 100;

%% ================= RELAY FAILURE =================
enableFailure = 1;

failureRelays = [6, 7, 9];
failureTimes  = [2600, 1400, 1500];

failedRelays = false(height(ENV0.Relays),1);

%% ================= RELAY BACKBONE =================
nRelays = height(ENV0.Relays);
relayDelay = inf(nRelays,nRelays);
relaySNR   = -inf(nRelays,nRelays);

for a = 1:nRelays
    relayDelay(a,a) = 0;
    relaySNR(a,a) = inf;
    for b = a+1:nRelays
        mab = link_metrics_snr_prob( ...
            ENV0.Relays.Latitude(a), ENV0.Relays.Longitude(a), ...
            ENV0.Relays.Latitude(b), ENV0.Relays.Longitude(b), P);

        % Connectivity gate for relay backbone
        if mab.SNR_dB > 0
            relayDelay(a,b) = mab.Delay_s;
            relayDelay(b,a) = mab.Delay_s;
            relaySNR(a,b) = mab.SNR_dB;
            relaySNR(b,a) = mab.SNR_dB;
        end
    end
end

% Gateway relay nearest to command center
dCtrl = arrayfun(@(rlat, rlon) haversine_km(rlat, rlon, CTRL.lat, CTRL.lon), ...
    ENV0.Relays.Latitude, ENV0.Relays.Longitude);
[~, ctrlRelayIdx] = min(dCtrl);

%% ================= SRC DST =================
src = 9;
dst = 15;

lat = ENV0.Hospitals.Latitude(src);
lon = ENV0.Hospitals.Longitude(src);

dstLat = ENV0.Hospitals.Latitude(dst);
dstLon = ENV0.Hospitals.Longitude(dst);

obsDim = agent.ObservationInfo.Dimension(1);

fprintf("Source: %d | Destination: %d\n",src,dst);

%% ================= DRL PATH =================
latPath = zeros(cfg.maxSteps,1);
lonPath = zeros(cfg.maxSteps,1);

for step = 1:cfg.maxSteps

    latPath(step) = lat;
    lonPath(step) = lon;

    obs = zeros(obsDim,1);

    [xN,yN] = norm_xy(lat,lon,cfg);
    [dstxN,dstyN] = norm_xy(dstLat,dstLon,cfg);
    distKm = haversine_km(lat,lon,dstLat,dstLon);
    distN  = min(distKm/20,1);

    obs(1:5) = [xN; yN; dstxN; dstyN; distN];

    act = getAction(agent,obs);
    a   = extract_action(act);

    stepMeters = cfg.uavSpeedMS * cfg.dtSec;
    [lat,lon] = apply_discrete_action(lat,lon,a,stepMeters);

    if haversine_km(lat,lon,dstLat,dstLon) < 0.3
        latPath = latPath(1:step);
        lonPath = lonPath(1:step);
        break;
    end
end

%% ================= SMOOTH =================
validIdx = latPath ~= 0;
latPath = latPath(validIdx);
lonPath = lonPath(validIdx);

nCtrl = min(10,length(latPath));
idx   = round(linspace(1,length(latPath),nCtrl));

latCtrl = latPath(idx);
lonCtrl = lonPath(idx);

latCtrl(end+1) = dstLat;
lonCtrl(end+1) = dstLon;

t  = 1:length(latCtrl);
tt = linspace(1,length(latCtrl), length(latPath));

latSmooth = interp1(t,latCtrl,tt,'pchip');
lonSmooth = interp1(t,lonCtrl,tt,'pchip');

%% ================= TRUE TOTAL DISTANCE =================
trueTotalDist_km = 0;

for k = 2:length(latSmooth)
    trueTotalDist_km = trueTotalDist_km + ...
        haversine_km(latSmooth(k-1), lonSmooth(k-1), ...
                     latSmooth(k),   lonSmooth(k));
end

%% ================= VISUAL =================
figure;
gx = geoaxes;
hold(gx,'on');
geobasemap(gx,'satellite');
disableDefaultInteractivity(gx);

geoscatter(gx,ENV0.Relays.Latitude,ENV0.Relays.Longitude,20,'k','filled');
geoscatter(gx,ENV0.Hospitals.Latitude,ENV0.Hospitals.Longitude,40,'r','filled');
geoscatter(gx,CTRL.lat,CTRL.lon,140,'c','p','filled');

geoscatter(gx,latCtrl(1),lonCtrl(1),150,'g','filled');
geoscatter(gx,dstLat,dstLon,150,'m','filled');

uavPlot = geoscatter(gx,latSmooth(1),lonSmooth(1),100,'b','filled');
pathPlot = geoplot(gx,latSmooth(1),lonSmooth(1),'b','LineWidth',3);

relayColors = repmat([0 0 0], height(ENV0.Relays),1);
relayPlot = geoscatter(gx,...
    ENV0.Relays.Latitude,...
    ENV0.Relays.Longitude,...
    30,relayColors,'filled');

commPathPlot = geoplot(gx,nan,nan,'c--','LineWidth',2.2);
packetPlot   = geoscatter(gx,nan,nan,90,'y','filled');

prevRelayIdx = -1;
initialConnected = false;
noLinkCounter = 0;
packetId = 0;

%% ================= TIME =================
CIT_total = 15000;

%% ================= DIST =================
totalDist_km = 0;
prevLat = latSmooth(1);
prevLon = lonSmooth(1);

latHist = latSmooth(1);
lonHist = lonSmooth(1);

%% ================= MOVE =================
for i = 2:length(latSmooth)

    lat = latSmooth(i);
    lon = lonSmooth(i);

    %% DISTANCE (REAL PHYSICS)
    stepDist_km = haversine_km(prevLat, prevLon, lat, lon);
    totalDist_km = totalDist_km + stepDist_km;

    %% TIME (REAL)
    timeElapsed = (totalDist_km * 1000) / cfg.uavSpeedMS;
    CIT_remaining = max(CIT_total - timeElapsed,0);

    %% RELAY FAILURE
    if enableFailure
        for k = 1:length(failureRelays)
            if timeElapsed >= failureTimes(k) && ~failedRelays(failureRelays(k))
                failedRelays(failureRelays(k)) = true;
                fprintf("💥 RELAY %d FAILED at %.2f sec\n", failureRelays(k), timeElapsed);
            end
        end
    end

    %% ENERGY
    energy_used = stepDist_km * 1000 * energy_per_meter;
    energy_remaining = max(energy_remaining - energy_used,0);
    battery_remaining = (energy_remaining / energy_total) * 100;

    prevLat = lat;
    prevLon = lon;

    %% RELAY TABLE
    relayTable = uav_to_relays(ENV0, P, lat, lon);

    %% APPLY FAILURES
    for r = find(failedRelays)'
        if r <= height(relayTable)
            relayTable.SNR_dB(r) = -100;
            relayTable.Delay_s(r) = inf;
        end
    end

    %% VALID RELAYS
    validIdx = (relayTable.SNR_dB > SINR_min) & (relayTable.Delay_s < 5);
    pkt = sample_data_packet();

    if any(validIdx)
        validRelays = relayTable(validIdx,:);
        [~, bestLocalIdx] = max(validRelays.SNR_dB);
        originalIdx = find(validIdx);
        bestRelayIdx = originalIdx(bestLocalIdx);

        snr = relayTable.SNR_dB(bestRelayIdx);
        mode = adaptive_AF_DF_switch(snr, relayTable.Delay_s(bestRelayIdx), pkt.priorityLevel);
    else
        bestRelayIdx = -1;
        snr = -100;
        mode = "NO LINK";
    end

    %% NETWORK FAILURE (STRICT)
    if (bestRelayIdx == -1 || snr < SINR_min)

        noLinkCounter = noLinkCounter + 1;
        fprintf("📡 LINK ISSUE | SINR: %.2f | Count: %d\n", snr, noLinkCounter);

        if noLinkCounter >= 3   % 🔥 STRICT CONDITION
            disp("🚫 NETWORK FAILURE");
            print_landing(lat, lon, "Network Failure");
            break;
        end
    else
        noLinkCounter = 0;
    end

    %% HANDOFF
    if bestRelayIdx ~= prevRelayIdx && bestRelayIdx ~= -1

        if ~initialConnected
            fprintf("🔗 Initial Connection → Relay %d | Mode: %s\n", bestRelayIdx, mode);
            initialConnected = true;
        else
            fprintf("🔁 Relay Handoff: %d → %d | Mode: %s\n", prevRelayIdx, bestRelayIdx, mode);
        end

        prevRelayIdx = bestRelayIdx;
    end

    %% PACKET MULTIHOP: UAV -> ACCESS RELAY -> RELAYS -> CONTROL CENTER
    packetId = packetId + 1;
    [pathRelayIds, e2eDelay, minPathSNR, delivered] = control_multihop_path( ...
        bestRelayIdx, ctrlRelayIdx, failedRelays, relayDelay, relaySNR, relayTable, SINR_min);

    if delivered
        routeLat = [lat; ENV0.Relays.Latitude(pathRelayIds); CTRL.lat];
        routeLon = [lon; ENV0.Relays.Longitude(pathRelayIds); CTRL.lon];
        set(commPathPlot,'LatitudeData',routeLat,'LongitudeData',routeLon);
        set(packetPlot,'LatitudeData',CTRL.lat,'LongitudeData',CTRL.lon);

        relayChainText = "R" + string(pathRelayIds(1));
        for q = 2:numel(pathRelayIds)
            relayChainText = relayChainText + "->R" + string(pathRelayIds(q));
        end

        fprintf(['📦 PKT %04d | %s | %s | Priority:%s | Mode:%s | ' ...
            'Path: UAV->%s->CTRL(R%d) | Hops:%d\n'], ...
            packetId, pkt.category, pkt.type, pkt.priorityName, mode, ...
            relayChainText, ctrlRelayIdx, numel(pathRelayIds));
    else
        set(commPathPlot,'LatitudeData',nan,'LongitudeData',nan);
        set(packetPlot,'LatitudeData',nan,'LongitudeData',nan);
        fprintf(['📦 PKT %04d | %s | %s | Priority:%s | Mode:%s | ' ...
            'Path: UAV->R%d->CTRL(R%d) | STATUS:DROPPED (No multihop route)\n'], ...
            packetId, pkt.category, pkt.type, pkt.priorityName, mode, bestRelayIdx, ctrlRelayIdx);
    end

    %% COLORS
    relayColors = repmat([0 0 0], height(ENV0.Relays),1);

    for r = find(failedRelays)'
        relayColors(r,:) = [1 0.5 0];
    end

    if bestRelayIdx > 0
        relayColors(bestRelayIdx,:) = [0 1 0];
    end

    set(relayPlot,'CData',relayColors);

    %% UPDATE PATH
    latHist = [latHist lat];
    lonHist = [lonHist lon];

    set(uavPlot,'LatitudeData',lat,'LongitudeData',lon);
    set(pathPlot,'LatitudeData',latHist,'LongitudeData',lonHist);

    %% TITLE
    title(gx, sprintf([ ...
        "Relay: %d | Mode: %s | SINR: %.1f dB\n" + ...
        "Time: %.1f sec | CIT: %.1f sec\n" + ...
        "Dist: %.2f km | Battery: %.1f %%"], ...
        prevRelayIdx, mode, snr, timeElapsed, CIT_remaining, ...
        totalDist_km, battery_remaining));

    drawnow limitrate;
    pause(0.05);

    %% STOP CONDITIONS
    if CIT_remaining <= 0
        print_landing(lat, lon, "CIT Expired");
        break;
    end

    if battery_remaining <= 5
        print_landing(lat, lon, "Battery Critical");
        break;
    end

end

fprintf("\n🚁 TOTAL DISTANCE: %.2f km\n", totalDist_km);
fprintf("🔋 BATTERY LEFT: %.1f %%\n", battery_remaining);

disp("✅ Simulation Complete");

end

%% ================= HELPER =================
function print_landing(lat, lon, reason)
fprintf("\n🚁 DRONE LANDED\n");
fprintf("📍 Location: (%.6f, %.6f)\n", lat, lon);
fprintf("⚠️ Reason: %s\n\n", reason);
end

function pkt = sample_data_packet()
r = rand();
if r < 0.25
    pkt.category = "Critical Medical";
    pkt.type = "Organ Telemetry";
    pkt.priorityLevel = 3;
    pkt.priorityName = "HIGH";
elseif r < 0.40
    pkt.category = "Critical Medical";
    pkt.type = "Emergency Alert";
    pkt.priorityLevel = 3;
    pkt.priorityName = "HIGH";
elseif r < 0.60
    pkt.category = "Control";
    pkt.type = "Command & Control";
    pkt.priorityLevel = 2;
    pkt.priorityName = "MEDIUM";
elseif r < 0.72
    pkt.category = "Control";
    pkt.type = "Relay Control";
    pkt.priorityLevel = 2;
    pkt.priorityName = "MEDIUM";
elseif r < 0.80
    pkt.category = "Control";
    pkt.type = "Delivery Confirmation";
    pkt.priorityLevel = 2;
    pkt.priorityName = "MEDIUM";
elseif r < 0.90
    pkt.category = "Support";
    pkt.type = "UAV Status";
    pkt.priorityLevel = 1;
    pkt.priorityName = "LOW";
elseif r < 0.97
    pkt.category = "Support";
    pkt.type = "Environmental Data";
    pkt.priorityLevel = 1;
    pkt.priorityName = "LOW";
else
    pkt.category = "Support";
    pkt.type = "Relay Beacon";
    pkt.priorityLevel = 1;
    pkt.priorityName = "LOW";
end
end

function [pathRelayIds, e2eDelay, minPathSNR, delivered] = ...
    control_multihop_path(accessRelayIdx, ctrlRelayIdx, failedRelays, relayDelay, relaySNR, relayTable, SINR_min)

pathRelayIds = [];
e2eDelay = inf;
minPathSNR = -inf;
delivered = false;

if accessRelayIdx <= 0 || ctrlRelayIdx <= 0
    return;
end
if failedRelays(accessRelayIdx) || failedRelays(ctrlRelayIdx)
    return;
end
if relayTable.SNR_dB(accessRelayIdx) < SINR_min
    return;
end

alive = ~failedRelays(:);
aliveIds = find(alive);
srcNew = find(aliveIds == accessRelayIdx,1);
dstNew = find(aliveIds == ctrlRelayIdx,1);

if isempty(srcNew) || isempty(dstNew)
    return;
end

W = relayDelay(aliveIds, aliveIds);
[ri, ci] = find(triu(W,1) < inf);
if isempty(ri)
    return;
end
wi = W(sub2ind(size(W), ri, ci));
wi = double(wi(:));
G = graph(ri, ci, wi);

[pathNew, pathCost] = shortestpath(G, srcNew, dstNew);
if isempty(pathNew) || ~isfinite(pathCost)
    return;
end

pathRelayIds = aliveIds(pathNew);
e2eDelay = relayTable.Delay_s(accessRelayIdx) + pathCost;

minPathSNR = relayTable.SNR_dB(accessRelayIdx);
for k = 1:(numel(pathRelayIds)-1)
    ii = pathRelayIds(k);
    jj = pathRelayIds(k+1);
    minPathSNR = min(minPathSNR, relaySNR(ii,jj));
end

delivered = true;
end
