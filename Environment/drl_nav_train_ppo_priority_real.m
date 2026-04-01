function drl_nav_train_ppo_priority_real()
% Priority-aware DRL-NAV PPO (REAL environment only)
% Requires your existing project functions:
%   environment_engine_cached()
%   net_params_option1()
%   build_relay_backbone(ENVactive,P) -> Net struct with .G (graph)
%   uav_to_relays(ENVactive,P,lat,lon) -> table with SNR_dB, Delay_s

% =========================
% CONFIG
% =========================
cfg = struct();

cfg.timeBudgetSec = 45*60;

% motion
cfg.stepMeters_nom = 200;
cfg.uavSpeedMS     = 15;
cfg.dtSec          = cfg.stepMeters_nom / cfg.uavSpeedMS;
cfg.maxSteps       = 350;

% comm
cfg.SNRmin_dB = 8;
cfg.delayCap  = 2.0;

% hard constraint tolerances (curriculum)
cfg.Kdisconnect_stage1 = 6;
cfg.Kdisconnect_stage2 = 4;
cfg.Kdisconnect_stage3 = 2;

% weather
cfg.windCap = 20;        % m/s
cfg.tempMin = -10;       % C
cfg.tempMax = 50;        % C

% energy
cfg.energyMax = 1.0;

% curriculum episodes
cfg.stage1Episodes = 300;   % feasibility
cfg.stage2Episodes = 900;   % realism
cfg.totalEpisodes  = 2000;  % full

% fixed control station (airport)
cfg.airportLat = 12.9965918;
cfg.airportLon = 80.1708076;

% =========================
% CREATE ENV
% =========================
env = drl_nav_make_env_priority_real(cfg);

obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);

numObs = obsInfo.Dimension(1);
numAct = numel(actInfo.Elements);

% =========================
% ACTOR / CRITIC
% =========================
actorNet = [
    featureInputLayer(numObs,'Normalization','none','Name','obs')
    fullyConnectedLayer(256,'Name','afc1')
    reluLayer('Name','arelu1')
    fullyConnectedLayer(256,'Name','afc2')
    reluLayer('Name','arelu2')
    fullyConnectedLayer(numAct,'Name','afc3')
    softmaxLayer('Name','asm')
];
actor = rlDiscreteCategoricalActor(actorNet, obsInfo, actInfo, ObservationInputNames="obs");

criticNet = [
    featureInputLayer(numObs,'Normalization','none','Name','obs')
    fullyConnectedLayer(256,'Name','cfc1')
    reluLayer('Name','crelu1')
    fullyConnectedLayer(256,'Name','cfc2')
    reluLayer('Name','crelu2')
    fullyConnectedLayer(1,'Name','cfc3')
];
critic = rlValueFunction(criticNet, obsInfo, ObservationInputNames="obs");

% =========================
% PPO OPTIONS
% =========================
agentOpts = rlPPOAgentOptions;
agentOpts.ExperienceHorizon = 1024;
agentOpts.ClipFactor        = 0.2;
agentOpts.MiniBatchSize     = 256;
agentOpts.NumEpoch          = 4;
agentOpts.GAEFactor         = 0.95;

agent = rlPPOAgent(actor, critic, agentOpts);

trainOpts = rlTrainingOptions;
trainOpts.MaxStepsPerEpisode         = cfg.maxSteps;
trainOpts.ScoreAveragingWindowLength = 25;
trainOpts.Plots                      = "training-progress";
trainOpts.Verbose                    = true;

% =========================
% PHASE 1: Feasibility
% =========================
agent.AgentOptions.EntropyLossWeight = 0.05;
agent.AgentOptions.DiscountFactor    = 0.995;

trainOpts.MaxEpisodes = cfg.stage1Episodes;
stats1 = train(agent, env, trainOpts);
save("drl_nav_priority_phase1_real.mat","agent","stats1","cfg");

% =========================
% PHASE 2: Realism
% =========================
agent.AgentOptions.EntropyLossWeight = 0.015;
agent.AgentOptions.DiscountFactor    = 0.998;

trainOpts.MaxEpisodes = cfg.stage2Episodes - cfg.stage1Episodes;
stats2 = train(agent, env, trainOpts);
save("drl_nav_priority_phase2_real.mat","agent","stats1","stats2","cfg");

% =========================
% PHASE 3: Full Objective
% =========================
agent.AgentOptions.EntropyLossWeight = 0.005;
agent.AgentOptions.DiscountFactor    = 0.999;

trainOpts.MaxEpisodes = cfg.totalEpisodes - cfg.stage2Episodes;
stats3 = train(agent, env, trainOpts);
save("drl_nav_priority_final_real.mat","agent","stats1","stats2","stats3","cfg");

% =========================
% EVAL + PLOT
% =========================
evaluate_priority_policy_real(agent, env, 20);
plot_one_rollout_real(agent, env);

end

% =====================================================================
% ENV FACTORY (REAL)
% =====================================================================
function env = drl_nav_make_env_priority_real(cfg)

ENV0 = environment_engine_cached();
P    = net_params_option1();

% bounds from your actual coords
allLat = [ENV0.Hospitals.Latitude; ENV0.Relays.Latitude];
allLon = [ENV0.Hospitals.Longitude; ENV0.Relays.Longitude];

cfg.boundMarginDeg = 0.01;
cfg.latMin = min(allLat) - cfg.boundMarginDeg;
cfg.latMax = max(allLat) + cfg.boundMarginDeg;
cfg.lonMin = min(allLon) - cfg.boundMarginDeg;
cfg.lonMax = max(allLon) + cfg.boundMarginDeg;

actInfo = rlFiniteSetSpec(1:9);
actInfo.Name = "act";

% Observation (16):
% [xN yN dstxN dstyN distN snrN e2eDstN e2eCtrlN e2eSrcN disconN windSpeedN windAlongN tempN energyN citN stageN]
obsInfo = rlNumericSpec([16 1], "LowerLimit",-inf, "UpperLimit",inf);
obsInfo.Name = "obs";

env = rlFunctionEnv(obsInfo, actInfo, ...
    @(a,s) drl_nav_step_priority_real(a,s,ENV0,P,cfg), ...
    @()    drl_nav_reset_priority_real(ENV0,P,cfg));

end

% =====================================================================
% RESET (REAL)
% =====================================================================
function [obs, logged] = drl_nav_reset_priority_real(ENV0, P, cfg)

persistent episodeCount
if isempty(episodeCount), episodeCount = 0; end
episodeCount = episodeCount + 1;

% curriculum stage
if episodeCount <= cfg.stage1Episodes
    logged.stage = 1;
elseif episodeCount <= cfg.stage2Episodes
    logged.stage = 2;
else
    logged.stage = 3;
end

switch logged.stage
    case 1
        logged.windMult = 0.0;
        logged.failMax  = 0;
        logged.citScale = 1.40;
        logged.energyDrain = 0.0020;
        logged.successRadiusKm = 0.50;
        logged.Kdisconnect = cfg.Kdisconnect_stage1;
    case 2
        logged.windMult = 0.6;
        logged.failMax  = 2;
        logged.citScale = 1.15;
        logged.energyDrain = 0.0027;
        logged.successRadiusKm = 0.30;
        logged.Kdisconnect = cfg.Kdisconnect_stage2;
    otherwise
        logged.windMult = 1.0;
        logged.failMax  = 3;
        logged.citScale = 1.00;
        logged.energyDrain = 0.0032;
        logged.successRadiusKm = 0.25;
        logged.Kdisconnect = cfg.Kdisconnect_stage3;
end

% random src/dst
Hn = height(ENV0.Hospitals);
logged.srcHospital = randi(Hn);
logged.dstHospital = randi(Hn);
while logged.dstHospital == logged.srcHospital
    logged.dstHospital = randi(Hn);
end

% store for evaluation
logged.ENV0 = ENV0;

% dynamic CIT
[logged.organ, baseCIT] = drl_nav_sample_organ_CIT();
logged.CITsec = baseCIT * logged.citScale;

% counters
logged.step = 0;
logged.elapsedSec = 0;
logged.disconnectCount = 0;

% failures
nR = height(ENV0.Relays);
logged.aliveMask = true(nR,1);

if logged.failMax > 0
    kFail = randi([1 logged.failMax]);
    logged.failRelays = randperm(nR, kFail);
    minS = max(10, floor(0.25*cfg.maxSteps));
    maxS = max(minS+1, floor(0.80*cfg.maxSteps));
    logged.failSteps = randi([minS maxS], 1, kFail);
else
    logged.failRelays = [];
    logged.failSteps  = [];
end

% energy
logged.energy = cfg.energyMax;

% init position = source hospital
lat0 = ENV0.Hospitals.Latitude(logged.srcHospital);
lon0 = ENV0.Hospitals.Longitude(logged.srcHospital);

lat0 = min(max(lat0, cfg.latMin), cfg.latMax);
lon0 = min(max(lon0, cfg.lonMin), cfg.lonMax);

logged.lat = lat0;
logged.lon = lon0;

% build backbone on active relays
ENVactive = drl_nav_getActiveENV(ENV0, logged.aliveMask);
logged.Net = build_relay_backbone(ENVactive, P);

% destination coords
dstLat = ENV0.Hospitals.Latitude(logged.dstHospital);
dstLon = ENV0.Hospitals.Longitude(logged.dstHospital);
logged.dstLat = dstLat;
logged.dstLon = dstLon;

distKm = drl_nav_haversine_km(lat0,lon0,dstLat,dstLon);
distN  = min(distKm/20, 1);

% weather at start
[tempC, u, v, windSpeed] = drl_nav_weather_at(ENV0, lat0, lon0, logged.windMult);
windSpeedN = clamp01(windSpeed/cfg.windCap);
windAlongN = clamp11(drl_nav_wind_along_heading(u,v,0)/cfg.windCap); % heading=0 init
tempN = clamp01((tempC - cfg.tempMin)/(cfg.tempMax - cfg.tempMin));

energyN = 1;
citN    = 1;
stageN  = (logged.stage-1)/2;

[xN,yN] = norm_xy(lat0,lon0,cfg);
[dstxN,dstyN] = norm_xy(dstLat,dstLon,cfg);

snrN=0; e2eDstN=0; e2eCtrlN=0; e2eSrcN=0; disconN=0;

obs = [xN; yN; dstxN; dstyN; distN; snrN; e2eDstN; e2eCtrlN; e2eSrcN; ...
       disconN; windSpeedN; windAlongN; tempN; energyN; citN; stageN];

end

% =====================================================================
% STEP (REAL)
% =====================================================================
function [nextObs, reward, isDone, logged] = drl_nav_step_priority_real(action, logged, ENV0, P, cfg)

isDone = false;

a = drl_nav_toScalarAction(action);

ENVactive = drl_nav_getActiveENV(ENV0, logged.aliveMask);

lat = logged.lat;
lon = logged.lon;

dstLat = logged.dstLat;
dstLon = logged.dstLon;

prevDist = drl_nav_haversine_km(lat, lon, dstLat, dstLon);

% weather
[tempC, u, v, windSpeed] = drl_nav_weather_at(ENV0, lat, lon, logged.windMult);
headingDeg = drl_nav_action_to_heading(a);
alongWind  = drl_nav_wind_along_heading(u, v, headingDeg);

% move (wind affects speed)
effSpeed = cfg.uavSpeedMS + alongWind;
effSpeed = max(6, min(25, effSpeed));
stepMeters = effSpeed * cfg.dtSec;

[lat2, lon2] = drl_nav_applyActionMeters(lat, lon, a, stepMeters);

lat2 = min(max(lat2, cfg.latMin), cfg.latMax);
lon2 = min(max(lon2, cfg.lonMin), cfg.lonMax);

logged.step = logged.step + 1;
logged.elapsedSec = logged.elapsedSec + cfg.dtSec;

% relay failures
if ~isempty(logged.failSteps)
    failNow = (logged.step == logged.failSteps);
    if any(failNow)
        ids = logged.failRelays(failNow);
        ids = ids(:)';
        for k = 1:numel(ids)
            if ids(k) >= 1 && ids(k) <= numel(logged.aliveMask)
                logged.aliveMask(ids(k)) = false;
            end
        end
        ENVactive = drl_nav_getActiveENV(ENV0, logged.aliveMask);
        logged.Net = build_relay_backbone(ENVactive, P);
    end
end

Net = logged.Net;

% UAV -> relays access
U = uav_to_relays(ENVactive, P, lat2, lon2);
nActive = height(ENVactive.Relays);
if nActive == 0
    reward = -80;
    isDone = true;
    nextObs = zeros(16,1);
    return;
end

score = U.SNR_dB - 10*min(U.Delay_s, cfg.delayCap);
[~, accessRelay] = max(score);

bestSNR         = U.SNR_dB(accessRelay);
bestAccessDelay = U.Delay_s(accessRelay);

% multi-sink gateways
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
badLink = (~routeOk) || (bestSNR < cfg.SNRmin_dB);

if badLink
    logged.disconnectCount = logged.disconnectCount + 1;
else
    logged.disconnectCount = 0;
end

% distance + progress
newDist    = drl_nav_haversine_km(lat2, lon2, dstLat, dstLon);
progressKm = prevDist - newDist;

% energy model
headwind = max(0, -alongWind);
windFactor = min(headwind/cfg.windCap, 0.6);

moveCost = (stepMeters / cfg.stepMeters_nom) * (1 + 0.25*windFactor);
commCost = 0.04 * min(drl_nav_safe_cap(e2eDst, cfg.delayCap), cfg.delayCap) / cfg.delayCap;

logged.energy = logged.energy - logged.energyDrain*(moveCost + commCost);

% normalized terms
distN   = min(newDist/20, 1);
snrN    = clamp01(bestSNR/40);

e2eDstN  = min(drl_nav_safe_cap(e2eDst, cfg.delayCap), cfg.delayCap)/cfg.delayCap;
e2eSrcN  = min(drl_nav_safe_cap(e2eSrc, cfg.delayCap), cfg.delayCap)/cfg.delayCap;
e2eCtrlN = min(drl_nav_safe_cap(e2eCtrl,cfg.delayCap), cfg.delayCap)/cfg.delayCap;

disconN = clamp01(logged.disconnectCount / max(1, logged.Kdisconnect));

windSpeedN = clamp01(windSpeed/cfg.windCap);
windAlongN = clamp11(alongWind/cfg.windCap);
tempN = clamp01((tempC - cfg.tempMin)/(cfg.tempMax - cfg.tempMin));

energyN = clamp01(logged.energy / cfg.energyMax);

citLeft = max(0, logged.CITsec - logged.elapsedSec);
citN    = clamp01(citLeft / logged.CITsec);

stageN  = (logged.stage-1)/2;

[xN,yN] = norm_xy(lat2,lon2,cfg);
[dstxN,dstyN] = norm_xy(dstLat,dstLon,cfg);

nextObs = [xN; yN; dstxN; dstyN; distN; snrN; e2eDstN; e2eCtrlN; e2eSrcN; ...
           disconN; windSpeedN; windAlongN; tempN; energyN; citN; stageN];

% =========================
% PRIORITY-AWARE REWARD
% =========================
R_success = 80;
R_fail    = 80;

% terminals (constraints FIRST)
if newDist < logged.successRadiusKm
    reward = R_success;
    isDone = true;
elseif logged.disconnectCount >= logged.Kdisconnect
    reward = -R_fail;
    isDone = true;
elseif logged.elapsedSec >= logged.CITsec
    reward = -R_fail;
    isDone = true;
elseif logged.energy <= 0
    reward = -R_fail;
    isDone = true;
elseif logged.step >= cfg.maxSteps
    reward = -60;
    isDone = true;
else
    % Stage 1: feasibility reward (CIT + progress + avoid bad link + don’t waste energy)
    wP = 10;     % progress km
    wT = 0.02;   % time
    wC = 1.0;    % bad link
    wU = 0.5;    % CIT urgency
    wE = 0.2;    % energy

    reward = wP*progressKm ...
           - wT ...
           - wC*(badLink) ...
           - wU*(1 - citN)^2 ...
           - wE*(1 - energyN);

    % Stage 2+: add reliability margin optimization
    if logged.stage >= 2
        wD = 0.5; % delay penalty
        wS = 0.2; % snr bonus
        reward = reward - wD*e2eDstN + wS*snrN;
    end

    % Stage 3: add weather + temp penalties (organ damage risk)
    if logged.stage == 3
        wW = 0.15;
        wTemp = 0.2;
        tauTemp = 0.7;
        reward = reward - wW*windSpeedN - wTemp*max(0, tempN - tauTemp);
    end
end

logged.lat = lat2;
logged.lon = lon2;

end

% =====================================================================
% EVALUATION + PLOT
% =====================================================================
function evaluate_priority_policy_real(agent, env, nEpisodes)

succ = 0;
stepsArr = zeros(nEpisodes,1);
finalDistArr = zeros(nEpisodes,1);

for ep = 1:nEpisodes
    [obs, logged] = feval(getResetFcn(env)); %#ok<ASGLU>
    done = false;

    while ~done && logged.step < 10000
        act = getAction(agent, obs);
        [obs, ~, done, logged] = feval(getStepFcn(env), act, logged);
    end

    ENV0 = logged.ENV0;
    d = drl_nav_haversine_km(logged.lat, logged.lon, ...
        ENV0.Hospitals.Latitude(logged.dstHospital), ENV0.Hospitals.Longitude(logged.dstHospital));

    stepsArr(ep) = logged.step;
    finalDistArr(ep) = d;

    if d < logged.successRadiusKm
        succ = succ + 1;
    end
end

fprintf("\n============================\n");
fprintf("EVAL over %d episodes\n", nEpisodes);
fprintf("SUCCESS RATE: %.2f %%\n", 100*succ/nEpisodes);
fprintf("Average Steps: %.2f\n", mean(stepsArr));
fprintf("Average Final Distance: %.3f km\n", mean(finalDistArr));
fprintf("============================\n\n");

end

function plot_one_rollout_real(agent, env)

[obs, logged] = feval(getResetFcn(env));

maxSteps = 350;
latPath = zeros(maxSteps,1);
lonPath = zeros(maxSteps,1);

for t = 1:maxSteps
    latPath(t) = logged.lat;
    lonPath(t) = logged.lon;

    act = getAction(agent, obs);
    [obs, ~, done, logged] = feval(getStepFcn(env), act, logged);
    if done
        latPath = latPath(1:t);
        lonPath = lonPath(1:t);
        break;
    end
end

ENV0 = logged.ENV0;

figure('Name','Priority-aware DRL-NAV Trajectory (REAL)');
plot(ENV0.Relays.Longitude, ENV0.Relays.Latitude, 'k.'); hold on;
plot(ENV0.Hospitals.Longitude, ENV0.Hospitals.Latitude, 'ro', 'LineWidth', 1.5);
plot(lonPath, latPath, 'b-', 'LineWidth', 2);

xlabel('Longitude'); ylabel('Latitude');
title('Priority-aware DRL-NAV UAV Trajectory (PPO, REAL)');
legend('Relays','Hospitals','UAV Path','Location','best');
grid on;

end

% =====================================================================
% HELPERS (self-contained; do not conflict with your existing ones)
% =====================================================================
function ENV = drl_nav_getActiveENV(ENV0, aliveMask)
ENV = ENV0;
ENV.Relays = ENV0.Relays(aliveMask,:);
end

function [xN,yN] = norm_xy(lat, lon, cfg)
yN = (lat - cfg.latMin) / max(1e-9, (cfg.latMax - cfg.latMin));
xN = (lon - cfg.lonMin) / max(1e-9, (cfg.lonMax - cfg.lonMin));
yN = clamp01(yN);
xN = clamp01(xN);
end

function y = clamp01(x)
y = min(max(x,0),1);
end

function y = clamp11(x)
y = min(max(x,-1),1);
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

function x = drl_nav_safe_cap(x, cap)
if ~isfinite(x), x = cap; end
end