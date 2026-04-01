function [results, summaryTable] = run_baseline_comparison(agentFile, numRuns, outputDir, randomSeed)
%RUN_BASELINE_COMPARISON Compare DRL policy against baseline policies.
%
% Usage:
%   run_baseline_comparison("drl_nav_final.mat", 50)
%   [results, summaryTable] = run_baseline_comparison("drl_nav_final.mat", 100, "my_outputs", 7)

if nargin < 1 || strlength(string(agentFile)) == 0
    error("Please provide agent file path, e.g. run_baseline_comparison(""drl_nav_final.mat"", 50)");
end
if nargin < 2 || isempty(numRuns)
    numRuns = 50;
end
if nargin < 3 || strlength(string(outputDir)) == 0
    outputDir = fullfile(fileparts(mfilename("fullpath")), "comparison_outputs");
end
if nargin < 4 || isempty(randomSeed)
    randomSeed = 42;
end

rng(randomSeed);
mkdir(outputDir);

suiteDir = fileparts(mfilename("fullpath"));
coreDir = fileparts(suiteDir);
addpath(coreDir);

fprintf("Loading environment and agent...\n");
ENV0 = environment_engine_cached();
P = get_comm_params();
S = load(agentFile, "agent");
agent = S.agent;

cfg = struct();
cfg.stepMeters = 200;
cfg.uavSpeedMS = 25;
cfg.dtSec = cfg.stepMeters / cfg.uavSpeedMS;
cfg.maxSteps = 350;
cfg.successRadiusKm = 0.30;
cfg.energyTotal = 100000;
cfg.energyPerMeter = 1.25;
cfg.citTotalSec = 15000;

models = ["DRL", "Straight", "NearestRelay", "SNRGreedy"];
metricNames = ["success", "timeSec", "distanceKm", "batteryPct", "avgSINR", "handoffs"];

results = struct();
for m = 1:numel(models)
    for f = 1:numel(metricNames)
        results.(models(m)).(metricNames(f)) = zeros(numRuns, 1);
    end
end

srcDstPairs = sample_src_dst_pairs(height(ENV0.Hospitals), numRuns);
save(fullfile(outputDir, "src_dst_pairs.mat"), "srcDstPairs");

for i = 1:numRuns
    src = srcDstPairs(i, 1);
    dst = srcDstPairs(i, 2);
    fprintf("Run %d/%d | src=%d dst=%d\n", i, numRuns, src, dst);

    for m = 1:numel(models)
        out = simulate_one_model(models(m), src, dst, agent, ENV0, P, cfg);
        results.(models(m)).success(i) = out.success;
        results.(models(m)).timeSec(i) = out.timeSec;
        results.(models(m)).distanceKm(i) = out.distanceKm;
        results.(models(m)).batteryPct(i) = out.batteryPct;
        results.(models(m)).avgSINR(i) = out.avgSINR;
        results.(models(m)).handoffs(i) = out.handoffs;
    end
end

summaryTable = build_summary_table(results, models);
writetable(summaryTable, fullfile(outputDir, "summary_table.csv"));
save(fullfile(outputDir, "comparison_results.mat"), "results", "summaryTable", "models", "cfg");

plot_baseline_results(results, models, summaryTable, outputDir);
fprintf("Comparison complete. Outputs saved in: %s\n", outputDir);

end

function out = simulate_one_model(model, src, dst, agent, ENV0, P, cfg)
lat = ENV0.Hospitals.Latitude(src);
lon = ENV0.Hospitals.Longitude(src);
dstLat = ENV0.Hospitals.Latitude(dst);
dstLon = ENV0.Hospitals.Longitude(dst);

obsDim = agent.ObservationInfo.Dimension(1);

totalDistKm = 0;
totalSINR = 0;
countSINR = 0;
handoffs = 0;
success = 0;
timeElapsedSec = 0;
energyRemaining = cfg.energyTotal;
prevRelayIdx = -1;

for step = 1:cfg.maxSteps
    distKm = haversine_km(lat, lon, dstLat, dstLon);
    if distKm < cfg.successRadiusKm
        success = 1;
        break;
    end

    action = choose_action(model, lat, lon, dstLat, dstLon, agent, obsDim, ENV0, P, cfg);
    [lat2, lon2] = apply_discrete_action(lat, lon, action, cfg.stepMeters);

    stepDistKm = haversine_km(lat, lon, lat2, lon2);
    totalDistKm = totalDistKm + stepDistKm;
    energyRemaining = energyRemaining - stepDistKm * 1000 * cfg.energyPerMeter;
    timeElapsedSec = (totalDistKm * 1000) / cfg.uavSpeedMS;

    relayTable = uav_to_relays(ENV0, P, lat2, lon2);
    if ~isempty(relayTable)
        [bestSINR, bestIdx] = max(relayTable.SNR_dB);
        totalSINR = totalSINR + bestSINR;
        countSINR = countSINR + 1;

        if prevRelayIdx ~= -1 && bestIdx ~= prevRelayIdx
            handoffs = handoffs + 1;
        end
        prevRelayIdx = bestIdx;
    end

    lat = lat2;
    lon = lon2;

    if energyRemaining <= 0 || timeElapsedSec > cfg.citTotalSec
        break;
    end
end

out.success = success;
out.timeSec = timeElapsedSec;
out.distanceKm = totalDistKm;
out.batteryPct = (energyRemaining / cfg.energyTotal) * 100;
out.avgSINR = totalSINR / max(countSINR, 1);
out.handoffs = handoffs;
end

function action = choose_action(model, lat, lon, dstLat, dstLon, agent, obsDim, ENV0, P, cfg)
switch string(model)
    case "DRL"
        obs = zeros(obsDim, 1);
        distKm = haversine_km(lat, lon, dstLat, dstLon);
        obs(1:min(5, obsDim)) = [0; 0; 0; 0; min(distKm / 20, 1)];
        act = getAction(agent, obs);
        action = normalize_discrete_action(act);

    case "Straight"
        action = action_towards_target(lat, lon, dstLat, dstLon, cfg.stepMeters);

    case "NearestRelay"
        d2Relay = (ENV0.Relays.Latitude - lat).^2 + (ENV0.Relays.Longitude - lon).^2;
        [~, relayIdx] = min(d2Relay);
        relayLat = ENV0.Relays.Latitude(relayIdx);
        relayLon = ENV0.Relays.Longitude(relayIdx);

        if haversine_km(lat, lon, dstLat, dstLon) < 1.0
            targetLat = dstLat;
            targetLon = dstLon;
        else
            targetLat = relayLat;
            targetLon = relayLon;
        end
        action = action_towards_target(lat, lon, targetLat, targetLon, cfg.stepMeters);

    case "SNRGreedy"
        % One-step lookahead over discrete actions: balance progress and SNR.
        wDist = 1.00;
        wSnr = 0.04;
        bestScore = -inf;
        bestAction = 1;

        for a = 1:4
            [candLat, candLon] = apply_discrete_action(lat, lon, a, cfg.stepMeters);
            dKm = haversine_km(candLat, candLon, dstLat, dstLon);
            relayTable = uav_to_relays(ENV0, P, candLat, candLon);
            if isempty(relayTable)
                bestSnr = -100;
            else
                bestSnr = max(relayTable.SNR_dB);
            end
            score = -wDist * dKm + wSnr * bestSnr;
            if score > bestScore
                bestScore = score;
                bestAction = a;
            end
        end
        action = bestAction;

    otherwise
        error("Unknown model: %s", string(model));
end
end

function action = normalize_discrete_action(act)
% Handle common action return types from RL toolbox across versions.
if iscell(act)
    act = act{1};
end

if isa(act, "dlarray")
    val = double(extractdata(act));
else
    val = double(act);
end

if isempty(val)
    action = 1;
else
    action = round(val(1));
end

action = min(4, max(1, action));
end

function action = action_towards_target(lat, lon, tgtLat, tgtLon, stepMeters)
[northLat, northLon] = apply_discrete_action(lat, lon, 1, stepMeters);
[southLat, southLon] = apply_discrete_action(lat, lon, 2, stepMeters);
[eastLat, eastLon] = apply_discrete_action(lat, lon, 3, stepMeters);
[westLat, westLon] = apply_discrete_action(lat, lon, 4, stepMeters);

dNorth = haversine_km(northLat, northLon, tgtLat, tgtLon);
dSouth = haversine_km(southLat, southLon, tgtLat, tgtLon);
dEast = haversine_km(eastLat, eastLon, tgtLat, tgtLon);
dWest = haversine_km(westLat, westLon, tgtLat, tgtLon);

[~, action] = min([dNorth, dSouth, dEast, dWest]);
end

function pairs = sample_src_dst_pairs(numHospitals, numRuns)
pairs = zeros(numRuns, 2);
for i = 1:numRuns
    src = randi(numHospitals);
    dst = randi(numHospitals);
    while dst == src
        dst = randi(numHospitals);
    end
    pairs(i, :) = [src, dst];
end
end

function T = build_summary_table(results, models)
n = numel(models);

Model = strings(n, 1);
SuccessRatePct = zeros(n, 1);
TimeMeanSec = zeros(n, 1);
DistanceMeanKm = zeros(n, 1);
BatteryMeanPct = zeros(n, 1);
SINRMeanDb = zeros(n, 1);
HandoffsMean = zeros(n, 1);

for m = 1:n
    model = models(m);
    Model(m) = model;
    SuccessRatePct(m) = 100 * mean(results.(model).success);
    TimeMeanSec(m) = mean(results.(model).timeSec);
    DistanceMeanKm(m) = mean(results.(model).distanceKm);
    BatteryMeanPct(m) = mean(results.(model).batteryPct);
    SINRMeanDb(m) = mean(results.(model).avgSINR);
    HandoffsMean(m) = mean(results.(model).handoffs);
end

T = table(Model, SuccessRatePct, TimeMeanSec, DistanceMeanKm, BatteryMeanPct, SINRMeanDb, HandoffsMean);
end
