function env = make_drl_env(timeBudgetSec)

if nargin < 1
    timeBudgetSec = 45*60;
end

ENV0 = environment_engine_cached();
P    = net_params_option1();

% ================== CONFIG ==================
cfg.airportLat   = 12.9965918;
cfg.airportLon   = 80.1708076;

cfg.stepMeters   = 200;
cfg.uavSpeedMS   = 15;
cfg.dtSec        = cfg.stepMeters / cfg.uavSpeedMS;
cfg.maxStepsCap  = floor(timeBudgetSec / cfg.dtSec);

cfg.SNRmin_dB    = 8;
cfg.Kdisconnect  = 2;

cfg.latMin = min(ENV0.Relays.Latitude)  - 0.02;
cfg.latMax = max(ENV0.Relays.Latitude)  + 0.02;
cfg.lonMin = min(ENV0.Relays.Longitude) - 0.02;
cfg.lonMax = max(ENV0.Relays.Longitude) + 0.02;

cfg.coastLon = 80.23;
cfg.delayCap = 2.0;

% Weather normalization
cfg.windCap = 20;   % m/s
cfg.tempMin = -10;
cfg.tempMax = 50;

% Energy model (normalized later)
cfg.energy0 = 1.0;                 % normalized battery (1.0 = full)
cfg.eMovePerMeter = 1.0e-4;        % base drain per meter
cfg.eHeadwindScale = 8.0e-3;       % extra drain per (m/s headwind) per step
cfg.eHandoff = 1.5e-3;             % extra drain per handoff

% Dynamic CIT (seconds) baseline ranges per organ (demo-friendly)
cfg.CIT.Heart  = [18*60, 30*60];    % 18–30 min
cfg.CIT.Liver  = [25*60, 45*60];    % 25–45 min
cfg.CIT.Kidney = [35*60, 60*60];    % 35–60 min

% Observation: 16 x 1
% [lat lon dstLat dstLon distN snrN e2eDstN e2eCtrlN e2eSrcN relayN handoff disconN windN tempN timeN energyN]
obsInfo = rlNumericSpec([16 1]);
obsInfo.Name = "obs";
actInfo = rlFiniteSetSpec(1:9);
actInfo.Name = "move";

env = rlFunctionEnv(obsInfo, actInfo, ...
    @(a,s) stepFcn(a,s,ENV0,P,cfg), ...
    @() resetFcn(ENV0,P,cfg));

end

% ======================= STEP =======================
function [nextObs, reward, isDone, logged] = stepFcn(action, logged, ENV0, P, cfg)

isDone = false;

ENV = getActiveENV(ENV0, logged.aliveMask);

lat = logged.lat;
lon = logged.lon;

dstLat = ENV.Hospitals.Latitude(logged.dstHospital);
dstLon = ENV.Hospitals.Longitude(logged.dstHospital);

prevDist = haversine_km(lat, lon, dstLat, dstLon);

% ===== WEATHER at current location =====
[tempC, u, v, windSpeed] = weather_at(ENV0, lat, lon);

% Apply curriculum wind multiplier
u = u * logged.windMult;
v = v * logged.windMult;
windSpeed = hypot(u,v);

headingDeg = action_to_heading(action);
alongWind  = wind_along_heading(u, v, headingDeg);

% Wind-aware movement
effSpeed = cfg.uavSpeedMS + alongWind;
effSpeed = max(5, min(25, effSpeed));
stepMeters = effSpeed * cfg.dtSec;

% Move UAV
[lat2, lon2] = applyActionMeters(lat, lon, action, stepMeters);

% clamp bounds
lat2 = min(max(lat2, cfg.latMin), cfg.latMax);
lon2 = min(max(lon2, cfg.lonMin), cfg.lonMax);

sea = (lon2 > cfg.coastLon);

% Step counter / time
logged.step = logged.step + 1;
logged.elapsedSec = logged.elapsedSec + cfg.dtSec;

% ===== Relay failure trigger =====
failNow = (logged.step == logged.failSteps);
if any(failNow)
    ids = logged.failRelays(failNow);
    for k = 1:numel(ids)
        logged.aliveMask(ids(k)) = false;
    end
    ENV = getActiveENV(ENV0, logged.aliveMask);
    logged.Net = build_relay_backbone(ENV, P);
end

Net = logged.Net;

% ===== Access link UAV->Relay =====
U = uav_to_relays(ENV, P, lat2, lon2);
score = U.SNR_dB - 10*min(U.Delay_s, cfg.delayCap);
[~, accessRelay] = max(score);

bestSNR = U.SNR_dB(accessRelay);
bestAccessDelay = U.Delay_s(accessRelay);

% ===== Gateways (multi-sink) =====
rSrc  = nearest_relay_local(ENV, ...
        ENV.Hospitals.Latitude(logged.srcHospital), ...
        ENV.Hospitals.Longitude(logged.srcHospital));

rDst  = nearest_relay_local(ENV, dstLat, dstLon);
rCtrl = nearest_relay_local(ENV, cfg.airportLat, cfg.airportLon);

dSrc  = distances(Net.G, rSrc);
dDst  = distances(Net.G, rDst);
dCtrl = distances(Net.G, rCtrl);

e2eDst  = bestAccessDelay + dDst(accessRelay);
e2eSrc  = bestAccessDelay + dSrc(accessRelay);
e2eCtrl = bestAccessDelay + dCtrl(accessRelay);

routeOk = isfinite(e2eDst) && isfinite(e2eSrc) && isfinite(e2eCtrl);

% Handoff
handoff = 0;
if ~isempty(logged.prevRelay) && logged.prevRelay ~= accessRelay
    handoff = 1;
end

% ===== Progress =====
newDist = haversine_km(lat2, lon2, dstLat, dstLon);
progress = prevDist - newDist;

badLink = (~routeOk) || (bestSNR < logged.SNRmin_dB) || sea;

if badLink
    logged.disconnectCount = logged.disconnectCount + 1;
else
    logged.disconnectCount = 0;
end

% ================= Energy Model =================
headwind = max(0, -alongWind);

eMove = cfg.eMovePerMeter * stepMeters;
eWind = cfg.eHeadwindScale * (headwind);
eHand = cfg.eHandoff * handoff;

logged.energy = logged.energy - (eMove + eWind + eHand);

% ================= Dynamic CIT =================
remainingCIT = logged.CITsec - logged.elapsedSec;

% ================== Normalized obs ==================
distN    = newDist / 20;
snrN     = bestSNR / 40;

e2eDstN  = min(e2eDst,  cfg.delayCap) / cfg.delayCap;
e2eSrcN  = min(e2eSrc,  cfg.delayCap) / cfg.delayCap;
e2eCtrlN = min(e2eCtrl, cfg.delayCap) / cfg.delayCap;

relayN   = accessRelay / max(1, height(ENV.Relays));
disconN  = logged.disconnectCount / logged.Kdisconnect;

windN = min(max(windSpeed / cfg.windCap, 0), 1);
tempN = min(max((tempC - cfg.tempMin) / (cfg.tempMax - cfg.tempMin), 0), 1);

timeN   = min(max(logged.elapsedSec / logged.CITsec, 0), 1);
energyN = min(max(logged.energy / cfg.energy0, 0), 1);

nextObs = [lat2; lon2; dstLat; dstLon; ...
           distN; snrN; e2eDstN; e2eCtrlN; e2eSrcN; ...
           relayN; handoff; disconN; windN; tempN; timeN; energyN];

% ================= Reward =================
rProgress = 30 * progress;
rStepTime = -0.05;

rComm = -2*e2eDstN -0.3*e2eCtrlN -0.3*e2eSrcN;
rHandoff = -0.5 * handoff;
rBad = badLink * (-80);

% Time pressure (Dynamic CIT)
% mild penalty early, strong penalty after 80% of CIT
lateZone = max(0, timeN - 0.80);
rTimePressure = -10 * lateZone^2 * 10;

% Energy pressure (avoid wasting battery)
rEnergy = -5 * max(0, 0.25 - energyN);

% Weather penalties (wind+temp only)
rWind = -0.2 * (headwind / cfg.windCap);
rTemp = -0.05 * (abs(tempC - 25) / 25);

reward = rProgress + rStepTime + rComm + rHandoff + rBad + rTimePressure + rEnergy + rWind + rTemp;

% ===== Terminal success =====
if newDist < 0.12
    reward = reward + 300;
    isDone = true;
end

% ===== Terminal failures =====
if logged.disconnectCount >= logged.Kdisconnect
    reward = reward - 300;
    isDone = true;
end

if remainingCIT <= 0
    reward = reward - 800;
    isDone = true;
end

if logged.energy <= 0
    reward = reward - 800;
    isDone = true;
end

if logged.step >= logged.maxSteps
    reward = reward - 200;
    isDone = true;
end

% Update state
logged.lat = lat2;
logged.lon = lon2;
logged.prevRelay = accessRelay;

end

% ======================= RESET =======================
function [obs, logged] = resetFcn(ENV0, P, cfg)

persistent epCount
if isempty(epCount)
    epCount = 0;
end
epCount = epCount + 1;

logged.episode = epCount;

% ========= Curriculum stages =========
% Stage1: ep 1..150, Stage2: 151..300, Stage3: 301+
if epCount <= 150
    stage = 1;
elseif epCount <= 300
    stage = 2;
else
    stage = 3;
end
logged.stage = stage;

% Stage parameters
switch stage
    case 1
        logged.windMult = 0.6;      % mild wind
        logged.maxFailRelays = 1;   % few failures
        logged.Kdisconnect = 3;     % forgiving
        logged.SNRmin_dB = 6;
        citScale = 1.15;            % looser CIT
    case 2
        logged.windMult = 1.0;      % normal wind
        logged.maxFailRelays = 2;
        logged.Kdisconnect = 2;
        logged.SNRmin_dB = 8;
        citScale = 1.00;
    otherwise
        logged.windMult = 1.4;      % strong wind
        logged.maxFailRelays = 3;
        logged.Kdisconnect = 2;
        logged.SNRmin_dB = 9;
        citScale = 0.85;            % tighter CIT
end

logged.step = 0;
logged.elapsedSec = 0;

% Energy start
logged.energy = cfg.energy0;

% Randomize hospital pair
nH = height(ENV0.Hospitals);
logged.srcHospital = randi(nH);
logged.dstHospital = randi(nH);
while logged.dstHospital == logged.srcHospital
    logged.dstHospital = randi(nH);
end

% Sample organ type (affects CIT)
orgs = ["Heart","Liver","Kidney"];
org = orgs(randi(3));
logged.organ = org;

range = cfg.CIT.(org);
logged.CITsec = citScale * (range(1) + (range(2)-range(1))*rand);

% Active relays
nRel = height(ENV0.Relays);
logged.aliveMask = true(nRel,1);

% Relay failures
kFail = randi([0 logged.maxFailRelays]);
if kFail == 0
    logged.failRelays = [];
    logged.failSteps  = [];
else
    logged.failRelays = randperm(nRel, kFail);

    minS = max(5, floor(0.2*cfg.maxStepsCap));
    maxS = max(minS+1, floor(0.8*cfg.maxStepsCap));
    logged.failSteps  = randi([minS maxS], 1, kFail);
end

logged.disconnectCount = 0;
logged.prevRelay = [];

% Start state at source
lat0 = ENV0.Hospitals.Latitude(logged.srcHospital);
lon0 = ENV0.Hospitals.Longitude(logged.srcHospital);

logged.lat = lat0;
logged.lon = lon0;

% Build network
ENV = getActiveENV(ENV0, logged.aliveMask);
logged.Net = build_relay_backbone(ENV, P);

% Destination
dstLat = ENV.Hospitals.Latitude(logged.dstHospital);
dstLon = ENV.Hospitals.Longitude(logged.dstHospital);

% Initial access relay
U = uav_to_relays(ENV, P, lat0, lon0);
[~, accessRelay] = max(U.SNR_dB);

distN = haversine_km(lat0, lon0, dstLat, dstLon) / 20;

% Weather at start
[tempC, u, v, windSpeed] = weather_at(ENV0, lat0, lon0);
u = u * logged.windMult; v = v * logged.windMult;
windSpeed = hypot(u,v);

windN = min(max(windSpeed / cfg.windCap, 0), 1);
tempN = min(max((tempC - cfg.tempMin) / (cfg.tempMax - cfg.tempMin), 0), 1);

timeN = 0;
energyN = 1;

logged.maxSteps = min(cfg.maxStepsCap, floor(logged.CITsec / cfg.dtSec)); % CIT-driven steps

obs = [lat0; lon0; dstLat; dstLon; ...
       distN; 0; 0; 0; 0; ...
       accessRelay / max(1,nRel); ...
       0; 0; ...
       windN; tempN; timeN; energyN];

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

function [tempC, u, v, windSpeed] = weather_at(ENV0, lat, lon)

tempC = 25; u = 0; v = 0; windSpeed = 0;

if isfield(ENV0,"Ftemp")
    try
        tempC = ENV0.Ftemp(lon, lat);
    catch
        tempC = 25;
    end
end

if isfield(ENV0,"Fu") && isfield(ENV0,"Fv")
    try
        u = ENV0.Fu(lon, lat);
        v = ENV0.Fv(lon, lat);
        windSpeed = hypot(u,v);
    catch
        u = 0; v = 0; windSpeed = 0;
    end
end

if ~isfinite(tempC), tempC = 25; end
if ~isfinite(u), u = 0; end
if ~isfinite(v), v = 0; end
if ~isfinite(windSpeed), windSpeed = hypot(u,v); end

end

function idx = nearest_relay_local(ENV, lat, lon)
n = height(ENV.Relays);
d = zeros(n,1);
for i = 1:n
    d(i) = haversine_km(lat, lon, ENV.Relays.Latitude(i), ENV.Relays.Longitude(i));
end
[~, idx] = min(d);
end