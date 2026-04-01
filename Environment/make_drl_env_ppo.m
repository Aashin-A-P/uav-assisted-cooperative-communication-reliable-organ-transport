function env = make_drl_env_ppo(timeBudgetSec)
% PPO-ready RL Function Environment for organ delivery UAV navigation
% Includes: multi-sink comm, relay failures, weather (wind+temp), energy, dynamic CIT, curriculum, random src/dst

if nargin < 1
    timeBudgetSec = 45*60;
end

ENV0 = environment_engine_cached();     % must contain Hospitals, Relays, and optionally: Ftemp, Fu, Fv
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

cfg.latMin = min(ENV0.Relays.Latitude)  - 0.02;
cfg.latMax = max(ENV0.Relays.Latitude)  + 0.02;
cfg.lonMin = min(ENV0.Relays.Longitude) - 0.02;
cfg.lonMax = max(ENV0.Relays.Longitude) + 0.02;

cfg.coastLon = 80.23;
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
    @(a,s) stepFcn(a,s,ENV0,P,cfg), ...
    @() resetFcn(ENV0,P,cfg));

end

% ======================= STEP =======================
function [nextObs, reward, isDone, logged] = stepFcn(action, logged, ENV0, P, cfg)

isDone = false;

% Force action to plain scalar integer in [1..9]
a = toScalarAction(action);

% active env (keep full indices in aliveMask, but ENVactive is subset)
ENVactive = getActiveENV(ENV0, logged.aliveMask);

lat = logged.lat;
lon = logged.lon;

dstLat = ENV0.Hospitals.Latitude(logged.dstHospital);
dstLon = ENV0.Hospitals.Longitude(logged.dstHospital);

prevDist = haversine_km(lat, lon, dstLat, dstLon);

% ===== WEATHER at current location =====
[tempC, u, v, windSpeed] = weather_at(ENV0, lat, lon, logged.windMult);
headingDeg = action_to_heading(a);
alongWind  = wind_along_heading(u, v, headingDeg);

% wind-aware effective speed
effSpeed = cfg.uavSpeedMS + alongWind;
effSpeed = max(6, min(25, effSpeed));

stepMeters = effSpeed * cfg.dtSec;

% Move UAV
[lat2, lon2] = applyActionMeters(lat, lon, a, stepMeters);

% clamp bounds
lat2 = min(max(lat2, cfg.latMin), cfg.latMax);
lon2 = min(max(lon2, cfg.lonMin), cfg.lonMax);

sea = (lon2 > cfg.coastLon);

% step counters
logged.step = logged.step + 1;
logged.elapsedSec = logged.elapsedSec + cfg.dtSec;

% ===== relay failures at configured steps =====
failNow = (logged.step == logged.failSteps);
if any(failNow)
    ids = logged.failRelays(failNow);
    ids = ids(:)'; % row
    for k = 1:numel(ids)
        if ids(k) >= 1 && ids(k) <= numel(logged.aliveMask)
            logged.aliveMask(ids(k)) = false;
        end
    end
    ENVactive = getActiveENV(ENV0, logged.aliveMask);
    logged.Net = build_relay_backbone(ENVactive, P);
end

Net = logged.Net;

% ===== UAV -> relay access link =====
U = uav_to_relays(ENVactive, P, lat2, lon2);

% If no relays left (edge case)
nActive = height(ENVactive.Relays);
if nActive == 0
    nextObs = terminalObs(lat2, lon2, dstLat, dstLon, cfg);
    reward  = -500;
    isDone  = true;
    return;
end

% score prefers high SNR + low access delay
score = U.SNR_dB - 10*min(U.Delay_s, cfg.delayCap);
[~, accessRelay] = max(score);

bestSNR        = U.SNR_dB(accessRelay);
bestAccessDelay= U.Delay_s(accessRelay);

% ===== multi-sink gateways (nearest in ACTIVE set) =====
rSrc  = nearest_relay_local(ENVactive, ENV0.Hospitals.Latitude(logged.srcHospital), ENV0.Hospitals.Longitude(logged.srcHospital));
rDst  = nearest_relay_local(ENVactive, dstLat, dstLon);
rCtrl = nearest_relay_local(ENVactive, cfg.airportLat, cfg.airportLon);

% ===== shortest-path delays on active graph =====
% distances returns Inf for unreachable
dSrc  = distances(Net.G, rSrc);
dDst  = distances(Net.G, rDst);
dCtrl = distances(Net.G, rCtrl);

e2eDst  = bestAccessDelay + dDst(accessRelay);
e2eSrc  = bestAccessDelay + dSrc(accessRelay);
e2eCtrl = bestAccessDelay + dCtrl(accessRelay);

routeOk = isfinite(e2eDst) && isfinite(e2eSrc) && isfinite(e2eCtrl);

% handoff detection (in active index space)
handoff = 0;
if ~isempty(logged.prevRelay) && logged.prevRelay ~= accessRelay
    handoff = 1;
end

% progress
newDist  = haversine_km(lat2, lon2, dstLat, dstLon);
progress = prevDist - newDist;

% link health
badLink = (~routeOk) || (bestSNR < cfg.SNRmin_dB) || sea;

if badLink
    logged.disconnectCount = logged.disconnectCount + 1;
else
    logged.disconnectCount = 0;
end

% ===== ENERGY model (stable) =====
headwind = max(0, -alongWind);

% cap wind penalty so battery doesn't instantly die in high wind
windFactor = min(headwind/cfg.windCap, 0.6);

moveCost = (stepMeters / cfg.stepMeters_nom) * (1 + 0.25*windFactor);
commCost = 0.04 * min(e2eDst, cfg.delayCap)/cfg.delayCap; % small

logged.energy = logged.energy - logged.energyDrain*(moveCost + commCost);

% ===== normalize obs =====
distN    = min(newDist/20, 1);
snrN     = min(max(bestSNR/40, 0), 1);
e2eDstN  = min(e2eDst, cfg.delayCap)/cfg.delayCap;
e2eSrcN  = min(e2eSrc, cfg.delayCap)/cfg.delayCap;
e2eCtrlN = min(e2eCtrl,cfg.delayCap)/cfg.delayCap;

disconN  = min(logged.disconnectCount / cfg.Kdisconnect, 1);

windN = min(max(windSpeed / cfg.windCap, 0), 1);
tempN = min(max((tempC - cfg.tempMin) / (cfg.tempMax - cfg.tempMin), 0), 1);

energyN = min(max(logged.energy / cfg.energyMax, 0), 1);

citLeft = max(0, logged.CITsec - logged.elapsedSec);
citN    = min(citLeft / logged.CITsec, 1);

stageN  = (logged.stage-1)/2;

nextObs = [lat2; lon2; dstLat; dstLon; distN; snrN; e2eDstN; e2eCtrlN; e2eSrcN; handoff; disconN; windN; tempN; energyN; citN; stageN];

% ===== reward shaping =====
rProgress = 40*progress;                  % encourage moving toward goal
rTime     = -0.10;                        % time penalty each step
rComm     = -2.2*e2eDstN -0.4*e2eCtrlN -0.4*e2eSrcN;
rHandoff  = -0.6*handoff;
rBad      = badLink * (-80);
rEnergy   = -0.15*(1-energyN);            % keep battery high
rUrgency  = -0.10*(1-citN);               % care more as CIT reduces

reward = rProgress + rTime + rComm + rHandoff + rBad + rEnergy + rUrgency;

% ===== terminals =====
if newDist < 0.12
    reward = reward + 650;
    isDone = true;
end

if logged.disconnectCount >= cfg.Kdisconnect
    reward = reward - 350;
    isDone = true;
end

if logged.elapsedSec >= logged.CITsec
    reward = reward - 350;
    isDone = true;
end

if logged.energy <= 0
    reward = reward - 350;
    isDone = true;
end

if logged.step >= cfg.maxSteps
    reward = reward - 250;
    isDone = true;
end

logged.lat = lat2;
logged.lon = lon2;
logged.prevRelay = accessRelay;

end

% ======================= RESET =======================
function [obs, logged] = resetFcn(ENV0, P, cfg)

persistent episodeCount
if isempty(episodeCount), episodeCount = 0; end
episodeCount = episodeCount + 1;

% curriculum stages
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
    case 2
        logged.windMult = 0.60;
        failMax = 2;
        citScale = 1.15;
        logged.energyDrain = 0.0027;
    otherwise
        logged.windMult = 1.00;
        failMax = cfg.maxFailRelays;
        citScale = 1.00;
        logged.energyDrain = 0.0032;
end

% random src/dst
Hn = height(ENV0.Hospitals);
logged.srcHospital = randi(Hn);
logged.dstHospital = randi(Hn);
while logged.dstHospital == logged.srcHospital
    logged.dstHospital = randi(Hn);
end

% dynamic CIT (organ-wise)
[logged.organ, baseCIT] = sample_organ_CIT();
logged.CITsec = baseCIT * citScale;

% init state
logged.step = 0;
logged.elapsedSec = 0;

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

logged.lat = lat0;
logged.lon = lon0;

ENVactive = getActiveENV(ENV0, logged.aliveMask);
logged.Net = build_relay_backbone(ENVactive, P);

dstLat = ENV0.Hospitals.Latitude(logged.dstHospital);
dstLon = ENV0.Hospitals.Longitude(logged.dstHospital);

distN = min(haversine_km(lat0,lon0,dstLat,dstLon)/20, 1);

[tempC, ~, ~, windSpeed] = weather_at(ENV0, lat0, lon0, logged.windMult);
windN = min(max(windSpeed / cfg.windCap, 0), 1);
tempN = min(max((tempC - cfg.tempMin) / (cfg.tempMax - cfg.tempMin), 0), 1);

energyN = 1;
citN    = 1;
stageN  = (logged.stage-1)/2;

% initialize comm terms as 0 (agent will learn)
obs = [lat0; lon0; dstLat; dstLon; distN; 0; 0; 0; 0; 0; 0; windN; tempN; energyN; citN; stageN];

end

% ======================= HELPERS =======================
function ENV = getActiveENV(ENV0, aliveMask)
ENV = ENV0;
ENV.Relays = ENV0.Relays(aliveMask,:);
end

function [lat2, lon2] = applyActionMeters(lat, lon, a, stepMeters)
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

function hdg = action_to_heading(a)
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

function along = wind_along_heading(u, v, headingDeg)
he = sind(headingDeg);
hn = cosd(headingDeg);
along = u*he + v*hn;
end

function [tempC, u, v, windSpeed] = weather_at(ENV0, lat, lon, windMult)
tempC = 25; u = 0; v = 0;

if isfield(ENV0, "Ftemp")
    try
        tempC = ENV0.Ftemp(lon, lat);
    catch
        tempC = 25;
    end
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

function idx = nearest_relay_local(ENVactive, lat, lon)
R = ENVactive.Relays;
d = haversine_km(lat, lon, R.Latitude, R.Longitude);
[~, idx] = min(d);
end

function [organName, citSec] = sample_organ_CIT()
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

function a = toScalarAction(action)
% Converts action from various MATLAB RL formats to plain scalar integer.
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

function obs = terminalObs(lat, lon, dstLat, dstLon, cfg)
d = min(haversine_km(lat,lon,dstLat,dstLon)/20, 1);
obs = [lat; lon; dstLat; dstLon; d; 0; 1; 1; 1; 0; 1; 0; 0; 0; 0; 1];
end