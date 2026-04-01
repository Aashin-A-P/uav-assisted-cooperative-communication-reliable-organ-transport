function plot_random_trajectory_from_trained_agent(agentFile)

% Example:
% plot_random_trajectory_from_trained_agent("drl_nav_priority_final_real.mat")

clc;

if nargin < 1
    error("Provide trained agent MAT file.");
end

load(agentFile,"agent");

% ================= REAL ENV =================
ENV0 = environment_engine_cached();
P    = net_params_option1(); %#ok<NASGU>

cfg.stepMeters_nom = 200;
cfg.uavSpeedMS     = 15;
cfg.dtSec          = cfg.stepMeters_nom / cfg.uavSpeedMS;
cfg.maxSteps       = 350;

cfg.windCap   = 20;
cfg.tempMin   = -10;
cfg.tempMax   = 50;
cfg.energyMax = 1.0;

allLat = [ENV0.Hospitals.Latitude; ENV0.Relays.Latitude];
allLon = [ENV0.Hospitals.Longitude; ENV0.Relays.Longitude];

cfg.boundMarginDeg = 0.01;
cfg.latMin = min(allLat) - cfg.boundMarginDeg;
cfg.latMax = max(allLat) + cfg.boundMarginDeg;
cfg.lonMin = min(allLon) - cfg.boundMarginDeg;
cfg.lonMax = max(allLon) + cfg.boundMarginDeg;

% ================= RANDOM HOSPITALS =================
Hn = height(ENV0.Hospitals);
src = randi(Hn);
dst = randi(Hn);
while dst == src
    dst = randi(Hn);
end

fprintf("\nSource Hospital ID: %d\n",src);
fprintf("Destination Hospital ID: %d\n",dst);

lat = ENV0.Hospitals.Latitude(src);
lon = ENV0.Hospitals.Longitude(src);

dstLat = ENV0.Hospitals.Latitude(dst);
dstLon = ENV0.Hospitals.Longitude(dst);

[organ, CITsec] = drl_nav_sample_organ_CIT();

elapsedSec = 0;
energy = 1.0;
disconnectCount = 0;
success = false;

latPath = zeros(cfg.maxSteps,1);
lonPath = zeros(cfg.maxSteps,1);

% ================= SIMULATION =================
for step = 1:cfg.maxSteps
    
    latPath(step) = lat;
    lonPath(step) = lon;
    
    distKm = haversine_km(lat,lon,dstLat,dstLon);
    distN  = min(distKm/20,1);
    
    [tempC,~,~,windSpeed] = weather_at(ENV0,lat,lon,1.0);
    windSpeedN = min(max(windSpeed/cfg.windCap,0),1);
    tempN = min(max((tempC - cfg.tempMin)/(cfg.tempMax-cfg.tempMin),0),1);
    
    energyN = min(max(energy,0),1);
    citLeft = max(0,CITsec - elapsedSec);
    citN = min(citLeft/CITsec,1);
    
    [xN,yN] = norm_xy(lat,lon,cfg);
    [dstxN,dstyN] = norm_xy(dstLat,dstLon,cfg);
    
    obs = [xN; yN; dstxN; dstyN; distN; ...
           0;0;0;0; ...
           disconnectCount; ...
           windSpeedN; 0; tempN; ...
           energyN; citN; 1];
    
    act = getAction(agent,obs);
    a = extract_action(act);
    
    stepMeters = cfg.uavSpeedMS * cfg.dtSec;
    [lat,lon] = apply_action(lat,lon,a,stepMeters);
    
    lat = min(max(lat,cfg.latMin),cfg.latMax);
    lon = min(max(lon,cfg.lonMin),cfg.lonMax);
    
    elapsedSec = elapsedSec + cfg.dtSec;
    
    newDist = haversine_km(lat,lon,dstLat,dstLon);
    
    if newDist < 0.3
        success = true;
        latPath = latPath(1:step);
        lonPath = lonPath(1:step);
        break;
    end
    
    if elapsedSec >= CITsec
        break;
    end
end

% ================= RESULTS =================
fprintf("\nOrgan: %s\n",organ);
fprintf("Steps Taken: %d\n",step);
fprintf("Final Distance: %.3f km\n",newDist);
fprintf("Remaining CIT (minutes): %.2f\n",(CITsec-elapsedSec)/60);
fprintf("SUCCESS: %d\n\n",success);

% ================= PLOT =================
figure('Name','DRL-NAV Random Hospital Trajectory');
plot(ENV0.Relays.Longitude,ENV0.Relays.Latitude,'k.'); hold on;
plot(ENV0.Hospitals.Longitude,ENV0.Hospitals.Latitude,'ro','LineWidth',1.2);
plot(lonPath,latPath,'b-','LineWidth',2);

plot(ENV0.Hospitals.Longitude(src),ENV0.Hospitals.Latitude(src),'go','MarkerSize',10,'LineWidth',2);
plot(ENV0.Hospitals.Longitude(dst),ENV0.Hospitals.Latitude(dst),'mo','MarkerSize',10,'LineWidth',2);

xlabel('Longitude');
ylabel('Latitude');
title('DRL-NAV UAV Trajectory (Trained PPO)');
legend('Relays','Hospitals','UAV Path','Source','Destination','Location','best');
grid on;

end

% ==========================================================
% =================== HELPER FUNCTIONS =====================
% ==========================================================

function a = extract_action(act)

if iscell(act)
    act = act{1};
end

if isa(act,'dlarray')
    act = extractdata(act);
end

if iscategorical(act)
    act = double(act);
end

a = double(act);

if numel(a) > 1
    a = a(1);
end

a = round(a);
a = max(1,min(9,a));

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

function [lat2, lon2] = apply_action(lat, lon, a, stepMeters)

dLat = stepMeters / 111320;
dLon = stepMeters / (111320 * cosd(lat));

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

function [tempC, u, v, windSpeed] = weather_at(ENV0, lat, lon, windMult)

tempC = 25; u = 0; v = 0;

if isfield(ENV0,"Ftemp")
    try, tempC = ENV0.Ftemp(lon,lat); catch, end
end

if isfield(ENV0,"Fu") && isfield(ENV0,"Fv")
    try
        u = ENV0.Fu(lon,lat);
        v = ENV0.Fv(lon,lat);
    catch
    end
end

u = windMult*u;
v = windMult*v;
windSpeed = hypot(u,v);

end

function [organName, citSec] = drl_nav_sample_organ_CIT()

r = rand;

if r < 0.34
    organName = "Kidney";
    citSec = 10*60*60;
elseif r < 0.67
    organName = "Liver";
    citSec = 8*60*60;
else
    organName = "Heart";
    citSec = 4*60*60;
end

end

function [xN,yN] = norm_xy(lat,lon,cfg)

yN = (lat - cfg.latMin)/(cfg.latMax - cfg.latMin);
xN = (lon - cfg.lonMin)/(cfg.lonMax - cfg.lonMin);

yN = min(max(yN,0),1);
xN = min(max(xN,0),1);

end