function plot_random_trajectory_on_map_smooth(agentFile)

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

allLat = [ENV0.Hospitals.Latitude; ENV0.Relays.Latitude];
allLon = [ENV0.Hospitals.Longitude; ENV0.Relays.Longitude];

cfg.boundMarginDeg = 0.01;
cfg.latMin = min(allLat) - cfg.boundMarginDeg;
cfg.latMax = max(allLat) + cfg.boundMarginDeg;
cfg.lonMin = min(allLon) - cfg.boundMarginDeg;
cfg.lonMax = max(allLon) + cfg.boundMarginDeg;

%% ================= RANDOM HOSPITALS =================
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

%% ================= SIMULATION =================
for step = 1:cfg.maxSteps

    latPath(step) = lat;
    lonPath(step) = lon;

    distKm = haversine_km(lat,lon,dstLat,dstLon);
    distN  = min(distKm/20,1);

    [xN,yN] = norm_xy(lat,lon,cfg);
    [dstxN,dstyN] = norm_xy(dstLat,dstLon,cfg);

    % Observation structure (minimal required for policy)
    obs = [xN; yN; dstxN; dstyN; distN; 0; 0; 0];

    act = getAction(agent,obs);
    a   = extract_action(act);

    % Detect if action is continuous or discrete
    if isContinuousAction(agent)
        headingRad = a*pi;
        stepMeters = cfg.uavSpeedMS * cfg.dtSec;

        dx = stepMeters*sin(headingRad);
        dy = stepMeters*cos(headingRad);

        lat = lat + dy/111320;
        lon = lon + dx/(111320*cosd(lat));
    else
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

%% ================= SMOOTH (VISUAL ONLY) =================
latSmooth = smoothdata(latPath,'gaussian',9);
lonSmooth = smoothdata(lonPath,'gaussian',9);

%% ================= GEO MAP PLOT =================
figure('Name','DRL-NAV Trajectory (Satellite View)');
gx = geoaxes;
hold(gx,'on');

geobasemap(gx,'satellite');

% Relays
geoscatter(gx,ENV0.Relays.Latitude,ENV0.Relays.Longitude,...
    10,'k','filled');

% Hospitals
geoscatter(gx,ENV0.Hospitals.Latitude,ENV0.Hospitals.Longitude,...
    40,'r','filled');

% UAV Path
geoplot(gx,latSmooth,lonSmooth,'b','LineWidth',2);

% Source
geoscatter(gx,...
    ENV0.Hospitals.Latitude(src),...
    ENV0.Hospitals.Longitude(src),...
    100,'g','filled');

% Destination
geoscatter(gx,...
    ENV0.Hospitals.Latitude(dst),...
    ENV0.Hospitals.Longitude(dst),...
    100,'m','filled');

title('DRL-NAV UAV Trajectory (Smoothed Visualization)');
legend('Relays','Hospitals','UAV Path','Source','Destination');

end

%% ==========================================================
%% =================== HELPER FUNCTIONS =====================
%% ==========================================================

function flag = isContinuousAction(agent)
try
    flag = isa(agent.AgentOptions.ActionInfo,'rlNumericSpec');
catch
    flag = false;
end
end

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