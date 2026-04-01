function drl_nav_train_ppo_continuous_heading_real()
% Continuous-action PPO for DRL-NAV (REAL env only)
% Action: headingNorm in [-1,1] mapped to headingRad in [-pi,pi]
%
% Requires your existing project functions:
%   environment_engine_cached()
%   net_params_option1()
%   build_relay_backbone(ENVactive, P) -> struct Net with .G graph (weighted)
%   uav_to_relays(ENVactive, P, lat, lon) -> table with SNR_dB, Delay_s
clc;

%% ================= CONFIG =================
cfg.stepMeters_nom = 200;
cfg.uavSpeedMS = 15;
cfg.dtSec = cfg.stepMeters_nom / cfg.uavSpeedMS;
cfg.maxSteps = 350;

ENV0 = environment_engine_cached();
P    = net_params_option1();

allLat = [ENV0.Hospitals.Latitude; ENV0.Relays.Latitude];
allLon = [ENV0.Hospitals.Longitude; ENV0.Relays.Longitude];

cfg.latMin = min(allLat) - 0.01;
cfg.latMax = max(allLat) + 0.01;
cfg.lonMin = min(allLon) - 0.01;
cfg.lonMax = max(allLon) + 0.01;

%% ================= RL SPECS =================
obsInfo = rlNumericSpec([8 1],LowerLimit=-inf,UpperLimit=inf);
obsInfo.Name = "obs";

actInfo = rlNumericSpec([1 1],LowerLimit=-1,UpperLimit=1);
actInfo.Name = "heading";

env = rlFunctionEnv(obsInfo,actInfo,...
    @(a,s) stepFcn(a,s,ENV0,cfg),...
    @() resetFcn(ENV0,cfg));

%% ================= ACTOR (MEAN + STD HEADS) =================
numObs = 8;
numAct = 1;

lg = layerGraph();

lg = addLayers(lg,featureInputLayer(numObs,'Normalization','none','Name','obs'));
lg = addLayers(lg,fullyConnectedLayer(256,'Name','fc1'));
lg = addLayers(lg,reluLayer('Name','relu1'));
lg = addLayers(lg,fullyConnectedLayer(256,'Name','fc2'));
lg = addLayers(lg,reluLayer('Name','relu2'));

meanPath = [
    fullyConnectedLayer(numAct,'Name','mean_fc')
    tanhLayer('Name','mean_tanh')
];

stdPath = [
    fullyConnectedLayer(numAct,'Name','std_fc')
    softplusLayer('Name','std_softplus')
];

lg = addLayers(lg,meanPath);
lg = addLayers(lg,stdPath);

lg = connectLayers(lg,'obs','fc1');
lg = connectLayers(lg,'fc1','relu1');
lg = connectLayers(lg,'relu1','fc2');
lg = connectLayers(lg,'fc2','relu2');
lg = connectLayers(lg,'relu2','mean_fc');
lg = connectLayers(lg,'relu2','std_fc');

actor = rlContinuousGaussianActor(lg,obsInfo,actInfo,...
    ObservationInputNames="obs",...
    ActionMeanOutputNames="mean_tanh",...
    ActionStandardDeviationOutputNames="std_softplus");

%% ================= CRITIC =================
criticNet = [
    featureInputLayer(numObs,'Normalization','none')
    fullyConnectedLayer(256)
    reluLayer
    fullyConnectedLayer(256)
    reluLayer
    fullyConnectedLayer(1)
];

critic = rlValueFunction(criticNet,obsInfo);

%% ================= PPO =================
agentOpts = rlPPOAgentOptions;
agentOpts.ExperienceHorizon = 1024;
agentOpts.ClipFactor = 0.2;
agentOpts.MiniBatchSize = 256;
agentOpts.NumEpoch = 4;
agentOpts.GAEFactor = 0.95;

agent = rlPPOAgent(actor,critic,agentOpts);

trainOpts = rlTrainingOptions;
trainOpts.MaxStepsPerEpisode = cfg.maxSteps;
trainOpts.ScoreAveragingWindowLength = 20;
trainOpts.Plots = "training-progress";
trainOpts.MaxEpisodes = 1500;
trainOpts.Verbose = true;

train(agent,env,trainOpts);

save("continuous_research_agent.mat","agent","cfg");

disp("Training Complete");

evaluate(agent,ENV0,cfg,20);
plotTrajectory(agent,ENV0,cfg);

end

%% ================= STEP =================
function [obs,reward,isDone,logged] = stepFcn(action,logged,ENV0,cfg)

heading = extractAction(action);
headingRad = heading*pi;

lat = logged.lat;
lon = logged.lon;

dstLat = logged.dstLat;
dstLon = logged.dstLon;

prevDist = hav(lat,lon,dstLat,dstLon);

dx = cfg.stepMeters_nom*sin(headingRad);
dy = cfg.stepMeters_nom*cos(headingRad);

lat2 = lat + dy/111320;
lon2 = lon + dx/(111320*cosd(lat));

newDist = hav(lat2,lon2,dstLat,dstLon);
progress = prevDist - newDist;

targetAngle = atan2d(dstLon-lon,dstLat-lat);
headingDeg = rad2deg(headingRad);
angleDiff = abs(wrapTo180(targetAngle - headingDeg));

directionReward = cosd(angleDiff);
smoothPenalty = abs(heading - logged.prevHeading);

reward = 30*progress + 4*directionReward - 0.4*smoothPenalty - 0.01;

logged.prevHeading = heading;
logged.lat = lat2;
logged.lon = lon2;
logged.step = logged.step + 1;

isDone = false;
if newDist < 0.3
    reward = reward + 60;
    isDone = true;
end
if logged.step >= cfg.maxSteps
    isDone = true;
end

obs = [
    lat2; lon2;
    dstLat; dstLon;
    newDist;
    progress;
    heading;
    logged.step/cfg.maxSteps
];

end

%% ================= RESET =================
function [obs,logged] = resetFcn(ENV0,cfg)

Hn = height(ENV0.Hospitals);

logged.srcHospital = randi(Hn);
logged.dstHospital = randi(Hn);
while logged.dstHospital == logged.srcHospital
    logged.dstHospital = randi(Hn);
end

logged.lat = ENV0.Hospitals.Latitude(logged.srcHospital);
logged.lon = ENV0.Hospitals.Longitude(logged.srcHospital);

logged.dstLat = ENV0.Hospitals.Latitude(logged.dstHospital);
logged.dstLon = ENV0.Hospitals.Longitude(logged.dstHospital);

logged.prevHeading = 0;
logged.step = 0;

obs = [
    logged.lat; logged.lon;
    logged.dstLat; logged.dstLon;
    hav(logged.lat,logged.lon,logged.dstLat,logged.dstLon);
    0;
    0;
    0
];

end

%% ================= EVALUATE =================
function evaluate(agent,ENV0,cfg,n)

succ=0;

for i=1:n
    [obs,logged]=resetFcn(ENV0,cfg);
    done=false;
    while ~done
        act=getAction(agent,obs);
        a=extractAction(act);
        [obs,~,done,logged]=stepFcn(a,logged,ENV0,cfg);
    end
    if hav(logged.lat,logged.lon,logged.dstLat,logged.dstLon)<0.3
        succ=succ+1;
    end
end

fprintf("Success Rate: %.2f%%\n",100*succ/n);

end

%% ================= PLOT =================
function plotTrajectory(agent,ENV0,cfg)

[obs,logged]=resetFcn(ENV0,cfg);

latPath=[];
lonPath=[];

done=false;
while ~done
    latPath(end+1)=logged.lat;
    lonPath(end+1)=logged.lon;
    act=getAction(agent,obs);
    a=extractAction(act);
    [obs,~,done,logged]=stepFcn(a,logged,ENV0,cfg);
end

figure;
plot(ENV0.Relays.Longitude,ENV0.Relays.Latitude,'k.'); hold on;
plot(ENV0.Hospitals.Longitude,ENV0.Hospitals.Latitude,'ro');
plot(lonPath,latPath,'b-','LineWidth',2);
grid on;
title("Continuous PPO UAV Trajectory");

end

%% ================= HELPERS =================
function a=extractAction(act)
if iscell(act), act=act{1}; end
if isa(act,'dlarray'), act=extractdata(act); end
a=double(act);
a=max(-1,min(1,a));
end

function d=hav(lat1,lon1,lat2,lon2)
R=6371;
phi1=deg2rad(lat1); phi2=deg2rad(lat2);
dphi=deg2rad(lat2-lat1);
dlmb=deg2rad(lon2-lon1);
a=sin(dphi/2).^2+cos(phi1).*cos(phi2).*sin(dlmb/2).^2;
c=2*atan2(sqrt(a),sqrt(1-a));
d=R*c;
end