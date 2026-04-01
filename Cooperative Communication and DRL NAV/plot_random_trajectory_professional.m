function plot_random_trajectory_professional(agentFile)

clc;

if nargin < 1
    error("Provide trained agent MAT file.");
end

load(agentFile,"agent");

ENV0 = environment_engine_cached();

P = get_comm_params();

cfg.stepMeters_nom = 200;
cfg.uavSpeedMS     = 15;
cfg.dtSec          = cfg.stepMeters_nom / cfg.uavSpeedMS;
cfg.maxSteps       = 350;

cfg.airportLat = 12.9965918;
cfg.airportLon = 80.1708076;

allLat = [ENV0.Hospitals.Latitude; ENV0.Relays.Latitude];
allLon = [ENV0.Hospitals.Longitude; ENV0.Relays.Longitude];

cfg.latMin = min(allLat) - 0.01;
cfg.latMax = max(allLat) + 0.01;
cfg.lonMin = min(allLon) - 0.01;
cfg.lonMax = max(allLon) + 0.01;

Hn = height(ENV0.Hospitals);

src = 2;
dst = 15;
while dst == src
    dst = randi(Hn);
end

lat = ENV0.Hospitals.Latitude(src);
lon = ENV0.Hospitals.Longitude(src);

dstLat = ENV0.Hospitals.Latitude(dst);
dstLon = ENV0.Hospitals.Longitude(dst);

latPath = zeros(cfg.maxSteps,1);
lonPath = zeros(cfg.maxSteps,1);

obsDim = agent.ObservationInfo.Dimension(1);

for step = 1:cfg.maxSteps

    latPath(step) = lat;
    lonPath(step) = lon;

    % ================= COMMUNICATION =================
    relayTable = uav_to_relays(ENV0, P, lat, lon);

    if ~isempty(relayTable)

        bestRelay = select_best_relay(relayTable);
        priority = get_packet_priority();
        DL_conf = rand();

        mode = adaptive_AF_DF(priority, DL_conf, bestRelay);

        fprintf("Step %d | Relay %d | %s | %s\n",...
            step, bestRelay.idx, priority, mode);
    end

    % ================= DRL =================
    obs = zeros(obsDim,1);

    [xN,yN] = norm_xy(lat,lon,cfg);
    [dstxN,dstyN] = norm_xy(dstLat,dstLon,cfg);
    distKm = haversine_km(lat,lon,dstLat,dstLon);

    obs(1:5) = [xN; yN; dstxN; dstyN; min(distKm/20,1)];

    act = getAction(agent,obs);
    a   = extract_action(act);

    stepMeters = cfg.uavSpeedMS * cfg.dtSec;

    [lat,lon] = apply_discrete_action(lat,lon,a,stepMeters);

    lat = min(max(lat,cfg.latMin),cfg.latMax);
    lon = min(max(lon,cfg.lonMin),cfg.lonMax);

    if haversine_km(lat,lon,dstLat,dstLon) < 0.3
        latPath = latPath(1:step);
        lonPath = lonPath(1:step);
        break;
    end

end

figure;
gx = geoaxes;
hold(gx,'on');
geobasemap(gx,'satellite');

geoscatter(gx,ENV0.Relays.Latitude,ENV0.Relays.Longitude,15,'k','filled');
geoscatter(gx,ENV0.Hospitals.Latitude,ENV0.Hospitals.Longitude,40,'r','filled');

geoplot(gx,latPath,lonPath,'b','LineWidth',2);

geoscatter(gx,dstLat,dstLon,150,'m','filled');

title('DRL + Relay + AF/DF');

end