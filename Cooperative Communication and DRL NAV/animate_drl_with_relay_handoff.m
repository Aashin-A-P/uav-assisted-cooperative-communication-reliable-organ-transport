function animate_drl_with_relay_handoff(agentFile)

clc;
close all;

load(agentFile,"agent");

%% ================= ENV =================
ENV0 = environment_engine_cached();
P    = get_comm_params();

%% ================= CONFIG =================
cfg.stepMeters_nom = 300;
cfg.uavSpeedMS     = 25;
cfg.dtSec          = cfg.stepMeters_nom / cfg.uavSpeedMS;
cfg.maxSteps       = 1000;

allLat = [ENV0.Hospitals.Latitude; ENV0.Relays.Latitude];
allLon = [ENV0.Hospitals.Longitude; ENV0.Relays.Longitude];

cfg.latMin = min(allLat) - 0.01;
cfg.latMax = max(allLat) + 0.01;
cfg.lonMin = min(allLon) - 0.01;
cfg.lonMax = max(allLon) + 0.01;

%% ================= RANDOM SRC/DST =================
Hn = height(ENV0.Hospitals);

src = randi(Hn);
dst = randi(Hn);
while dst == src
    dst = randi(Hn);
end

lat = ENV0.Hospitals.Latitude(src);
lon = ENV0.Hospitals.Longitude(src);

dstLat = ENV0.Hospitals.Latitude(dst);
dstLon = ENV0.Hospitals.Longitude(dst);

obsDim = agent.ObservationInfo.Dimension(1);

fprintf("Source: %d | Destination: %d\n",src,dst);

%% ================= FIGURE =================
figure('Name','UAV Relay Handoff Animation');

gx = geoaxes;
hold(gx,'on');
geobasemap(gx,'satellite');

disableDefaultInteractivity(gx); % FIX CRASH

% Hospitals
geoscatter(gx,ENV0.Hospitals.Latitude,ENV0.Hospitals.Longitude,40,'r','filled');

% Source & Destination
geoscatter(gx,lat,lon,150,'g','filled');
geoscatter(gx,dstLat,dstLon,150,'m','filled');

% Relay INITIAL COLORS
relayColors = repmat([0 0 0], height(ENV0.Relays),1);

relayPlot = geoscatter(gx,...
    ENV0.Relays.Latitude,...
    ENV0.Relays.Longitude,...
    25,relayColors,'filled');

% UAV
uavPlot = geoscatter(gx,lat,lon,100,'b','filled');

% Path
pathPlot = geoplot(gx,lat,lon,'b','LineWidth',2);

title(gx,'DRL-NAV + Relay Handoff');

%% ================= SIMULATION =================
latHist = lat;
lonHist = lon;

prevRelayIdx = -1;

for step = 1:cfg.maxSteps

    %% ===== DRL OBS =====
    obs = zeros(obsDim,1);

    [xN,yN] = norm_xy(lat,lon,cfg);
    [dstxN,dstyN] = norm_xy(dstLat,dstLon,cfg);

    distKm = haversine_km(lat,lon,dstLat,dstLon);
    distN  = min(distKm/20,1);

    baseObs = [xN; yN; dstxN; dstyN; distN];
    obs(1:length(baseObs)) = baseObs;

    %% ===== ACTION =====
    act = getAction(agent,obs);
    a   = extract_action(act);

    stepMeters = cfg.uavSpeedMS * cfg.dtSec;
    [lat,lon] = apply_discrete_action(lat,lon,a,stepMeters);

    %% ===== COMMUNICATION (BACKGROUND) =====
    relayTable = uav_to_relays(ENV0, P, lat, lon);

    if ~isempty(relayTable)

        bestRelay = select_best_relay(relayTable);

        % HANDOFF
        if bestRelay.idx ~= prevRelayIdx
            fprintf("🔁 Relay Handoff: %d → %d\n",prevRelayIdx,bestRelay.idx);
            prevRelayIdx = bestRelay.idx;
        end

        % Update relay colors (NO DELETE)
        relayColors = repmat([0 0 0], height(ENV0.Relays),1);

        if bestRelay.idx <= size(relayColors,1)
            relayColors(bestRelay.idx,:) = [0 1 0]; % GREEN
        end

        set(relayPlot,...
            'LatitudeData', ENV0.Relays.Latitude,...
            'LongitudeData', ENV0.Relays.Longitude,...
            'CData', relayColors);
    end

    %% ===== UPDATE PATH =====
    latHist(end+1) = lat;
    lonHist(end+1) = lon;

    set(uavPlot,'LatitudeData',lat,'LongitudeData',lon);
    set(pathPlot,'LatitudeData',latHist,'LongitudeData',lonHist);

    title(gx, sprintf("Step: %d | Relay: %d", step, prevRelayIdx));

    drawnow limitrate;

    pause(0.05);

    %% ===== TERMINATION =====
    if haversine_km(lat,lon,dstLat,dstLon) < 0.3
        disp("✅ Destination Reached");
        break;
    end

end

end