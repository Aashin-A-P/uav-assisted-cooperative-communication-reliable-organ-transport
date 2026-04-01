function plot_random_trajectory_professional(agentFile)

clc;

if nargin < 1
    error("Provide trained agent MAT file.");
end

load(agentFile,"agent");

%% ================= LOAD REAL ENV =================
ENV0 = environment_engine_cached();

cfg.stepMeters_nom = 200;
cfg.uavSpeedMS     = 15;
cfg.dtSec          = cfg.stepMeters_nom / cfg.uavSpeedMS;
cfg.maxSteps       = 350;

cfg.airportLat = 12.9965918;
cfg.airportLon = 80.1708076;

allLat = [ENV0.Hospitals.Latitude; ENV0.Relays.Latitude];
allLon = [ENV0.Hospitals.Longitude; ENV0.Relays.Longitude];

cfg.boundMarginDeg = 0.01;
cfg.latMin = min(allLat) - cfg.boundMarginDeg;
cfg.latMax = max(allLat) + cfg.boundMarginDeg;
cfg.lonMin = min(allLon) - cfg.boundMarginDeg;
cfg.lonMax = max(allLon) + cfg.boundMarginDeg;

%% ================= RANDOM SOURCE & DESTINATION =================
Hn = height(ENV0.Hospitals);

src = 2;
dst = 15;
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

obsDim = agent.ObservationInfo.Dimension(1);

%% ================= SIMULATION =================
for step = 1:cfg.maxSteps

    latPath(step) = lat;
    lonPath(step) = lon;

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

    % Detect action type
    if isa(agent.ActionInfo,'rlNumericSpec')
        % Continuous heading
        headingRad = a*pi;
        stepMeters = cfg.uavSpeedMS * cfg.dtSec;

        dx = stepMeters*sin(headingRad);
        dy = stepMeters*cos(headingRad);

        lat = lat + dy/111320;
        lon = lon + dx/(111320*cosd(lat));
    else
        % Discrete action
        stepMeters = cfg.uavSpeedMS * cfg.dtSec;
        [lat,lon] = apply_discrete_action(lat,lon,a,stepMeters);
    end

    lat = min(max(lat,cfg.latMin),cfg.latMax);
    lon = min(max(lon,cfg.lonMin),cfg.lonMax);

    newDist = haversine_km(lat,lon,dstLat,dstLon);

    if newDist < 0.3
        latPath = latPath(1:step);
        lonPath = lonPath(1:step);
        break;
    end

end

%% ================= PROFESSIONAL SMOOTHING =================

%% ================= PROFESSIONAL SMOOTHING + FINAL CONNECTION =================

%% ================= ULTRA-SMOOTH PROFESSIONAL VISUAL =================

% Remove unused points
validIdx = latPath ~= 0;
latPath = latPath(validIdx);
lonPath = lonPath(validIdx);

% Keep only 10 evenly spaced control points
nCtrl = min(10,length(latPath));
idx   = round(linspace(1,length(latPath),nCtrl));

latCtrl = latPath(idx);
lonCtrl = lonPath(idx);

% Add exact destination as final control point
latCtrl(end+1) = dstLat;
lonCtrl(end+1) = dstLon;

% Parametric interpolation (cubic spline)
t  = 1:length(latCtrl);
tt = linspace(1,length(latCtrl),500);

latSmooth = spline(t,latCtrl,tt);
lonSmooth = spline(t,lonCtrl,tt);

%% ================= GEO MAP VISUALIZATION =================

figure('Name','DRL-NAV Professional Trajectory');
clf; % clears any existing axes

gx = geoaxes('Parent',gcf); % attach explicitly to current figure
hold(gx,'on');

geobasemap(gx,'satellite');

% Relays
geoscatter(gx,ENV0.Relays.Latitude,ENV0.Relays.Longitude,...
    15,'k','filled');

% Hospitals
geoscatter(gx,ENV0.Hospitals.Latitude,ENV0.Hospitals.Longitude,...
    40,'r','filled');

% Control Center (Airport)
geoscatter(gx,cfg.airportLat,cfg.airportLon,...
    150,'c','filled');

% UAV Path (Smooth)
geoplot(gx,latSmooth,lonSmooth,'b','LineWidth',3);

% Source
geoscatter(gx,...
    ENV0.Hospitals.Latitude(src),...
    ENV0.Hospitals.Longitude(src),...
    150,'g','filled');

% Destination
geoscatter(gx,...
    ENV0.Hospitals.Latitude(dst),...
    ENV0.Hospitals.Longitude(dst),...
    150,'m','filled');

title('DRL-NAV UAV Trajectory (Smoothed Professional Visualization)');
legend('Relays','Hospitals','Control Center','UAV Path','Source','Destination');

end

%% ================= HELPER FUNCTIONS =================

function a = extract_action(act)
if iscell(act)
    act = act{1};
end
if isa(act,'dlarray')
    act = extractdata(act);
end
a = double(act);
if numel(a) > 1
    a = a(1);
end
end

function [lat2, lon2] = apply_discrete_action(lat, lon, a, stepMeters)
dLat = stepMeters / 111320;
dLon = stepMeters / (111320 * cosd(lat));
a = round(a);
a = max(1,min(9,a));
switch a
    case 1, lat2 = lat + dLat; lon2 = lon;
    case 2, lat2 = lat - dLat; lon2 = lon;
    case 3, lat2 = lat; lon2 = lon + dLon;
    case 4, lat2 = lat; lon2 = lon - dLon;
    case 5, lat2 = lat + dLat; lon2 = lon + dLon;
    case 6, lat2 = lat + dLat; lon2 = lon - dLon;
    case 7, lat2 = lat - dLat; lon2 = lon + dLon;
    case 8, lat2 = lat - dLat; lon2 = lon - dLon;
    otherwise, lat2 = lat; lon2 = lon;
end
end

function d_km = haversine_km(lat1, lon1, lat2, lon2)
R = 6371;
phi1 = deg2rad(lat1);
phi2 = deg2rad(lat2);
dphi = deg2rad(lat2-lat1);
dlmb = deg2rad(lon2-lon1);
a = sin(dphi/2).^2 + cos(phi1).*cos(phi2).*sin(dlmb/2).^2;
c = 2*atan2(sqrt(a), sqrt(1-a));
d_km = R*c;
end

function [xN,yN] = norm_xy(lat,lon,cfg)
yN = (lat - cfg.latMin)/(cfg.latMax - cfg.latMin);
xN = (lon - cfg.lonMin)/(cfg.lonMax - cfg.lonMin);
yN = min(max(yN,0),1);
xN = min(max(xN,0),1);
end