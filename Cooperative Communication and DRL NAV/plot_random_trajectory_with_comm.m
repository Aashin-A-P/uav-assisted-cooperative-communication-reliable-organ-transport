function plot_random_trajectory_with_comm(agentFile)

clc;

if nargin < 1
    error("Provide trained agent MAT file.");
end

load(agentFile,"agent");

%% ================= LOAD ENV =================
ENV0 = environment_engine_cached();

%% ================= COMM PARAMS =================
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

%% ================= RANDOM SOURCE & DEST =================
Hn = height(ENV0.Hospitals);

src = randi(Hn);
dst = randi(Hn);
while dst == src
    dst = randi(Hn);
end

fprintf("\nSource Hospital ID: %d\n",src);
fprintf("Destination Hospital ID: %d\n\n",dst);

lat = ENV0.Hospitals.Latitude(src);
lon = ENV0.Hospitals.Longitude(src);

dstLat = ENV0.Hospitals.Latitude(dst);
dstLon = ENV0.Hospitals.Longitude(dst);

latPath = zeros(cfg.maxSteps,1);
lonPath = zeros(cfg.maxSteps,1);

SNR_log   = zeros(cfg.maxSteps,1);
Delay_log = zeros(cfg.maxSteps,1);

obsDim = agent.ObservationInfo.Dimension(1);

%% ================= SIMULATION =================
for step = 1:cfg.maxSteps

    latPath(step) = lat;
    lonPath(step) = lon;

    %% ===== DRL NAVIGATION (UNCHANGED) =====
    obs = zeros(obsDim,1);

    [xN,yN] = norm_xy(lat,lon,cfg);
    [dstxN,dstyN] = norm_xy(dstLat,dstLon,cfg);
    distKm = haversine_km(lat,lon,dstLat,dstLon);
    distN  = min(distKm/20,1);

    baseObs = [xN; yN; dstxN; dstyN; distN];
    obs(1:min(length(baseObs),obsDim)) = ...
        baseObs(1:min(length(baseObs),obsDim));

    act = getAction(agent,obs);
    a   = extract_action(act);

    % Movement
    stepMeters = cfg.uavSpeedMS * cfg.dtSec;
    [lat,lon] = apply_discrete_action(lat,lon,a,stepMeters);

    lat = min(max(lat,cfg.latMin),cfg.latMax);
    lon = min(max(lon,cfg.lonMin),cfg.lonMax);

    %% ===== COMMUNICATION (PASSIVE ONLY) =====
    relayTable = uav_to_relays(ENV0, P, lat, lon);

    if ~isempty(relayTable)

        bestRelay = select_best_relay(relayTable);
        priority  = get_packet_priority();
        DL_conf   = rand();

        mode = adaptive_AF_DF(priority, DL_conf, bestRelay);

        SNR_log(step)   = bestRelay.SNR_dB;
        Delay_log(step) = bestRelay.Delay_s;

        fprintf("Step %d | Relay %d | %s | %s \n",...
            step, bestRelay.idx, priority, mode);
    end

    %% ===== TERMINATION =====
    newDist = haversine_km(lat,lon,dstLat,dstLon);

    if newDist < 0.3
        latPath = latPath(1:step);
        lonPath = lonPath(1:step);
        SNR_log = SNR_log(1:step);
        Delay_log = Delay_log(1:step);
        break;
    end

end

%% ================= SMOOTH PATH =================
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
tt = linspace(1,length(latCtrl),500);

latSmooth = spline(t,latCtrl,tt);
lonSmooth = spline(t,lonCtrl,tt);

%% ================= MAP =================
figure('Name','DRL-NAV + Communication Monitoring');
clf;

gx = geoaxes('Parent',gcf);
hold(gx,'on');
geobasemap(gx,'satellite');

geoscatter(gx,ENV0.Relays.Latitude,ENV0.Relays.Longitude,15,'k','filled');
geoscatter(gx,ENV0.Hospitals.Latitude,ENV0.Hospitals.Longitude,40,'r','filled');

geoplot(gx,latSmooth,lonSmooth,'b','LineWidth',3);

geoscatter(gx,ENV0.Hospitals.Latitude(src),ENV0.Hospitals.Longitude(src),150,'g','filled');
geoscatter(gx,ENV0.Hospitals.Latitude(dst),ENV0.Hospitals.Longitude(dst),150,'m','filled');

title('DRL Navigation with Passive Communication Monitoring');
legend('Relays','Hospitals','UAV Path','Source','Destination');

%% ================= COMM PLOTS =================
figure;
plot(SNR_log,'LineWidth',2);
title('SNR over Time');
xlabel('Step'); ylabel('SNR (dB)');

figure;
plot(Delay_log,'LineWidth',2);
title('Delay over Time');
xlabel('Step'); ylabel('Delay (s)');

end