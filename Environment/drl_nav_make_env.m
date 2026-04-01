function env = drl_nav_make_env(timeBudgetSec)
% DRL-NAV PPO RL Function Environment for organ delivery UAV navigation
% Includes: multi-sink comm, relay failures, weather (wind+temp), energy, dynamic CIT,
% curriculum, random src/dst
%
% IMPORTANT:
% - No hardcoded coastLon. We derive safe operational bounds from your actual Hospital+Relay coords.
% - We discourage "shore/sea side" by penalizing proximity to EAST boundary (lonMax edge).
%
% External dependencies assumed to exist in your project:
%   environment_engine_cached()
%   net_params_option1()
%   build_relay_backbone(ENVactive, P) -> struct with .G graph
%   uav_to_relays(ENVactive, P, lat, lon) -> table with SNR_dB, Delay_s

if nargin < 1
    timeBudgetSec = 45*60;
end

ENV0 = environment_engine_cached();
P    = net_params_option1();

cfg.airportLat = 12.9965918;
cfg.airportLon = 80.1708076;

cfg.stepMeters_nom = 200;
cfg.uavSpeedMS     = 15;
cfg.dtSec          = cfg.stepMeters_nom / cfg.uavSpeedMS;
cfg.maxSteps       = floor(timeBudgetSec / cfg.dtSec);

cfg.SNRmin_dB      = 8;
cfg.Kdisconnect    = 2;
cfg.maxFailRelays  = 3;

% ================= SAFE OPERATIONAL REGION =================
allLat = [ENV0.Hospitals.Latitude; ENV0.Relays.Latitude];
allLon = [ENV0.Hospitals.Longitude; ENV0.Relays.Longitude];

cfg.boundMarginDeg = 0.01;  % ~1km margin
cfg.latMin = min(allLat) - cfg.boundMarginDeg;
cfg.latMax = max(allLat) + cfg.boundMarginDeg;
cfg.lonMin = min(allLon) - cfg.boundMarginDeg;
cfg.lonMax = max(allLon) + cfg.boundMarginDeg;

% "Shore-avoidance": assume sea is typically to the EAST beyond your network.
% Penalize if UAV approaches east boundary.
cfg.eastBufferKm = 1.5; % start penalty within ~1.5km of lonMax edge

cfg.delayCap = 2.0;

cfg.windCap = 20;     % m/s
cfg.tempMin = -10;    % C
cfg.tempMax = 50;     % C

cfg.energyMax = 1.0;

% obs: [lat lon dstLat dstLon distN snrN e2eDstN e2eCtrlN e2eSrcN handoff disconN windN tempN energyN citN stageN]
obsInfo = rlNumericSpec([16 1], "LowerLimit",-inf, "UpperLimit",inf);
obsInfo.Name = "obs";

actInfo = rlFiniteSetSpec(1:9);
actInfo.Name = "act";

env = rlFunctionEnv(obsInfo, actInfo, ...
    @(a,s) drl_nav_step(a,s,ENV0,P,cfg), ...
    @()    drl_nav_reset(ENV0,P,cfg));

end

% ======================= STEP =======================
function [nextObs, reward, isDone, logged] = drl_nav_step(action, logged, ENV0, P, cfg)

isDone = false;

% action -> scalar [1..9]
a = drl_nav_toScalarAction(action);

% active env after failures
ENVactive = drl_nav_getActiveENV(ENV0, logged.aliveMask);

lat = logged.lat;
lon = logged.lon;

dstLat = ENV0.Hospitals.Latitude(logged.dstHospital);
dstLon = ENV0.Hospitals.Longitude(logged.dstHospital);

prevDist = drl_nav_haversine_km(lat, lon, dstLat, dstLon);

% ===== WEATHER =====
[tempC, u, v, windSpeed] = drl_nav_weather_at(ENV0, lat, lon, logged.windMult);
headingDeg = drl_nav_action_to_heading(a);
alongWind  = drl_nav_wind_along_heading(u, v, headingDeg);

% wind-aware speed
effSpeed = cfg.uavSpeedMS + alongWind;
effSpeed = max(6, min(25, effSpeed));
stepMeters = effSpeed * cfg.dtSec;

% move
[lat2, lon2] = drl_nav_applyActionMeters(lat, lon, a, stepMeters);

% clamp safe operational bounds
lat2 = min(max(lat2, cfg.latMin), cfg.latMax);
lon2 = min(max(lon2, cfg.lonMin), cfg.lonMax);

% step counters
logged.step = logged.step + 1;
logged.elapsedSec = logged.elapsedSec + cfg.dtSec;

% ===== relay failures =====
failNow = (logged.step == logged.failSteps);
if any(failNow)
    ids = logged.failRelays(failNow);
    ids = ids(:)'; % row
    for k = 1:numel(ids)
        if ids(k) >= 1 && ids(k) <= numel(logged.aliveMask)
            logged.aliveMask(ids(k)) = false;
        end
    end
    ENVactive = drl_nav_getActiveENV(ENV0, logged.aliveMask);
    logged.Net = build_relay_backbone(ENVactive, P);
end

Net = logged.Net;

% ===== UAV -> relay access =====
U = uav_to_relays(ENVactive, P, lat2, lon2);

nActive = height(ENVactive.Relays);
if nActive == 0
    nextObs = drl_nav_terminalObs(lat2, lon2, dstLat, dstLon);
    reward  = -5;
    isDone  = true;
    return;
end

% score prefers high SNR + low access delay
score = U.SNR_dB - 10*min(U.Delay_s, cfg.delayCap);
[~, accessRelay] = max(score);

bestSNR         = U.SNR_dB(accessRelay);
bestAccessDelay = U.Delay_s(accessRelay);

% ===== multi-sink gateways =====
rSrc  = drl_nav_nearest_relay_local(ENVactive, ENV0.Hospitals.Latitude(logged.srcHospital), ENV0.Hospitals.Longitude(logged.srcHospital));
rDst  = drl_nav_nearest_relay_local(ENVactive, dstLat, dstLon);
rCtrl = drl_nav_nearest_relay_local(ENVactive, cfg.airportLat, cfg.airportLon);

dSrc  = distances(Net.G, rSrc);
dDst  = distances(Net.G, rDst);
dCtrl = distances(Net.G, rCtrl);

e2eDst  = bestAccessDelay + dDst(accessRelay);
e2eSrc  = bestAccessDelay + dSrc(accessRelay);
e2eCtrl = bestAccessDelay + dCtrl(accessRelay);

routeOk = isfinite(e2eDst) && isfinite(e2eSrc) && isfinite(e2eCtrl);

% handoff
handoff = 0;
if ~isempty(logged.prevRelay) && logged.prevRelay ~= accessRelay
    handoff = 1;
end

% progress
newDist  = drl_nav_haversine_km(lat2, lon2, dstLat, dstLon);
progress = prevDist - newDist;

% ===== boundary “shore side” penalty (east edge) =====
eastDistKm = drl_nav_lon_dist_km(lat2, cfg.lonMax - lon2); % distance to east boundary
eastN = min(max((cfg.eastBufferKm - eastDistKm) / cfg.eastBufferKm, 0), 1); % 1 if very close to east edge

% link health
badLink = (~routeOk) || (bestSNR < cfg.SNRmin_dB);

if badLink
    logged.disconnectCount = logged.disconnectCount + 1;
else
    logged.disconnectCount = 0;
end

% ===== energy =====
headwind = max(0, -alongWind);
windFactor = min(headwind/cfg.windCap, 0.6);

moveCost = (stepMeters / cfg.stepMeters_nom) * (1 + 0.25*windFactor);
commCost = 0.04 * min(drl_nav_safe_cap(e2eDst, cfg.delayCap), cfg.delayCap) / cfg.delayCap;

logged.energy = logged.energy - logged.energyDrain*(moveCost + commCost);

% ===== normalize obs =====
distN    = min(newDist/20, 1);
snrN     = min(max(bestSNR/40, 0), 1);

e2eDstN  = min(drl_nav_safe_cap(e2eDst, cfg.delayCap), cfg.delayCap)/cfg.delayCap;
e2eSrcN  = min(drl_nav_safe_cap(e2eSrc, cfg.delayCap), cfg.delayCap)/cfg.delayCap;
e2eCtrlN = min(drl_nav_safe_cap(e2eCtrl,cfg.delayCap), cfg.delayCap)/cfg.delayCap;

disconN  = min(logged.disconnectCount / cfg.Kdisconnect, 1);

windN = min(max(windSpeed / cfg.windCap, 0), 1);
tempN = min(max((tempC - cfg.tempMin) / (cfg.tempMax - cfg.tempMin), 0), 1);

energyN = min(max(logged.energy / cfg.energyMax, 0), 1);

citLeft = max(0, logged.CITsec - logged.elapsedSec);
citN    = min(citLeft / logged.CITsec, 1);

stageN  = (logged.stage-1)/2;

nextObs = [lat2; lon2; dstLat; dstLon; distN; snrN; e2eDstN; e2eCtrlN; e2eSrcN; ...
           handoff; disconN; windN; tempN; energyN; citN; stageN];

% ================= DRL-NAV REWARD =================
rewardGoalProgress   = 220 * progress;
rewardDistanceShape  = -1.5 * newDist;
rewardCommQuality    = -1.2*e2eDstN -0.25*e2eCtrlN -0.25*e2eSrcN;
rewardRelaySwitch    = -0.35 * handoff;
rewardLinkPenalty    = badLink * (-20);
rewardEnergyPenalty  = -0.06 * (1 - energyN);
rewardUrgencyPenalty = -0.06 * (1 - citN);
rewardAliveBonus     = 0.6;
rewardTimePenalty    = -0.04;

% discourage going near east edge ("shore side")
rewardEastPenalty    = -2.0 * eastN;  % BEFORE scaling

reward = rewardGoalProgress + rewardDistanceShape + rewardCommQuality + rewardRelaySwitch + ...
         rewardLinkPenalty + rewardEnergyPenalty + rewardUrgencyPenalty + rewardAliveBonus + ...
         rewardTimePenalty + rewardEastPenalty;

reward = reward * 0.02;

% ================= TERMINALS =================
if newDist < logged.successRadiusKm
    reward = reward + 80 + 100*(1 - newDist/logged.successRadiusKm);
    isDone = true;
end

if logged.disconnectCount >= cfg.Kdisconnect
    reward = reward - 4;
    isDone = true;
end

if logged.elapsedSec >= logged.CITsec
    reward = reward - 4;
    isDone = true;
end

if logged.energy <= 0
    reward = reward - 4;
    isDone = true;
end

if logged.step >= cfg.maxSteps
    reward = reward - 3;
    isDone = true;
end

logged.lat = lat2;
logged.lon = lon2;
logged.prevRelay = accessRelay;

end

% ======================= RESET =======================
function [obs, logged] = drl_nav_reset(ENV0, P, cfg)

persistent episodeCount
if isempty(episodeCount), episodeCount = 0; end
episodeCount = episodeCount + 1;

% curriculum
if episodeCount <= 200
    logged.stage = 1;
elseif episodeCount <= 500
    logged.stage = 2;
else
    logged.stage = 3;
end

switch logged.stage
    case 1
        logged.windMult = 0.25;
        failMax = 1;
        citScale = 1.40;
        logged.energyDrain = 0.0022;
        logged.successRadiusKm = 0.50;
    case 2
        logged.windMult = 0.60;
        failMax = 2;
        citScale = 1.15;
        logged.energyDrain = 0.0027;
        logged.successRadiusKm = 0.30;
    otherwise
        logged.windMult = 1.00;
        failMax = cfg.maxFailRelays;
        citScale = 1.00;
        logged.energyDrain = 0.0032;
        logged.successRadiusKm = 0.25;
end

% random src/dst
Hn = height(ENV0.Hospitals);
logged.srcHospital = randi(Hn);
logged.dstHospital = randi(Hn);
while logged.dstHospital == logged.srcHospital
    logged.dstHospital = randi(Hn);
end

% dynamic CIT
[logged.organ, baseCIT] = drl_nav_sample_organ_CIT();
logged.CITsec = baseCIT * citScale;

% init counters
logged.step = 0;
logged.elapsedSec = 0;

% init failures
nR = height(ENV0.Relays);
logged.aliveMask = true(nR,1);

kFail = randi([1 failMax]);
logged.failRelays = randperm(nR, kFail);

minS = max(10, floor(0.25*cfg.maxSteps));
maxS = max(minS+1, floor(0.80*cfg.maxSteps));
logged.failSteps = randi([minS maxS], 1, kFail);

logged.disconnectCount = 0;
logged.prevRelay = [];

logged.energy = cfg.energyMax;

% start at source hospital
lat0 = ENV0.Hospitals.Latitude(logged.srcHospital);
lon0 = ENV0.Hospitals.Longitude(logged.srcHospital);

% clamp in bounds
lat0 = min(max(lat0, cfg.latMin), cfg.latMax);
lon0 = min(max(lon0, cfg.lonMin), cfg.lonMax);

logged.lat = lat0;
logged.lon = lon0;

% build initial graph
ENVactive = drl_nav_getActiveENV(ENV0, logged.aliveMask);
logged.Net = build_relay_backbone(ENVactive, P);

dstLat = ENV0.Hospitals.Latitude(logged.dstHospital);
dstLon = ENV0.Hospitals.Longitude(logged.dstHospital);

distN = min(drl_nav_haversine_km(lat0,lon0,dstLat,dstLon)/20, 1);

[tempC, ~, ~, windSpeed] = drl_nav_weather_at(ENV0, lat0, lon0, logged.windMult);
windN = min(max(windSpeed / cfg.windCap, 0), 1);
tempN = min(max((tempC - cfg.tempMin) / (cfg.tempMax - cfg.tempMin), 0), 1);

energyN = 1;
citN    = 1;
stageN  = (logged.stage-1)/2;

obs = [lat0; lon0; dstLat; dstLon; distN; ...
       0; 0; 0; 0; ...
       0; 0; ...
       windN; tempN; energyN; citN; stageN];

end

% ======================= HELPERS =======================
function ENV = drl_nav_getActiveENV(ENV0, aliveMask)
ENV = ENV0;
ENV.Relays = ENV0.Relays(aliveMask,:);
end

function [lat2, lon2] = drl_nav_applyActionMeters(lat, lon, a, stepMeters)
dLat = stepMeters / 111320;
dLon = stepMeters / (111320 * cosd(lat));

switch a
    case 1, lat2 = lat + dLat; lon2 = lon;            % N
    case 2, lat2 = lat - dLat; lon2 = lon;            % S
    case 3, lat2 = lat;        lon2 = lon + dLon;     % E
    case 4, lat2 = lat;        lon2 = lon - dLon;     % W
    case 5, lat2 = lat + dLat; lon2 = lon + dLon;     % NE
    case 6, lat2 = lat + dLat; lon2 = lon - dLon;     % NW
    case 7, lat2 = lat - dLat; lon2 = lon + dLon;     % SE
    case 8, lat2 = lat - dLat; lon2 = lon - dLon;     % SW
    otherwise, lat2 = lat;     lon2 = lon;            % Hover
end
end

function d_km = drl_nav_haversine_km(lat1, lon1, lat2, lon2)
R = 6371;
phi1 = deg2rad(lat1);
phi2 = deg2rad(lat2);
dphi = deg2rad(lat2-lat1);
dlmb = deg2rad(lon2-lon1);
a = sin(dphi/2).^2 + cos(phi1).*cos(phi2).*sin(dlmb/2).^2;
c = 2*atan2(sqrt(a), sqrt(1-a));
d_km = R*c;
end

function km = drl_nav_lon_dist_km(lat, dLonDeg)
km = abs(dLonDeg) * 111.32 * cosd(lat);
end

function hdg = drl_nav_action_to_heading(a)
switch a
    case 1, hdg = 0;
    case 2, hdg = 180;
    case 3, hdg = 90;
    case 4, hdg = 270;
    case 5, hdg = 45;
    case 6, hdg = 315;
    case 7, hdg = 135;
    case 8, hdg = 225;
    otherwise, hdg = 0;
end
end

function along = drl_nav_wind_along_heading(u, v, headingDeg)
he = sind(headingDeg);
hn = cosd(headingDeg);
along = u*he + v*hn;
end

function [tempC, u, v, windSpeed] = drl_nav_weather_at(ENV0, lat, lon, windMult)
tempC = 25; u = 0; v = 0;

if isfield(ENV0, "Ftemp")
    try, tempC = ENV0.Ftemp(lon, lat); catch, tempC = 25; end
end

if isfield(ENV0, "Fu") && isfield(ENV0, "Fv")
    try
        u = ENV0.Fu(lon, lat);
        v = ENV0.Fv(lon, lat);
    catch
        u = 0; v = 0;
    end
end

u = windMult*u;
v = windMult*v;

windSpeed = hypot(u,v);

if ~isfinite(tempC), tempC = 25; end
if ~isfinite(u), u = 0; end
if ~isfinite(v), v = 0; end
if ~isfinite(windSpeed), windSpeed = hypot(u,v); end
end

function idx = drl_nav_nearest_relay_local(ENVactive, lat, lon)
R = ENVactive.Relays;
d = drl_nav_haversine_km(lat, lon, R.Latitude, R.Longitude);
[~, idx] = min(d);
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

function a = drl_nav_toScalarAction(action)
try
    if iscell(action), action = action{1}; end
    if isa(action,"dlarray"), action = extractdata(action); end
    if iscategorical(action), action = double(action); end
    if isstring(action) || ischar(action), action = str2double(action); end
    if isstruct(action) && isfield(action,"act"), action = action.act; end
    action = double(action);
    if numel(action) > 1, action = action(1); end
catch
    action = 9;
end
a = round(action);
if ~isfinite(a), a = 9; end
a = max(1, min(9, a));
end

function obs = drl_nav_terminalObs(lat, lon, dstLat, dstLon)
d = min(drl_nav_haversine_km(lat,lon,dstLat,dstLon)/20, 1);
obs = [lat; lon; dstLat; dstLon; d; 0; 1; 1; 1; 0; 1; 0; 0; 0; 0; 1];
end

function x = drl_nav_safe_cap(x, cap)
if ~isfinite(x), x = cap; end
end