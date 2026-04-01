function result = react(agentFile, mission)
% REACT Parameterized DRL mission runner for React/Node integration.
%
% Usage:
%   mission = struct( ...
%       "src", 9, ...
%       "dst", 15, ...
%       "organ", "Heart", ...
%       "CIT_total", 15000, ...
%       "batteryPercent", 100, ...
%       "uavSpeedMS", 25, ...
%       "webEnable", true, ...
%       "webBase", "http://localhost:8000", ...
%       "enableVisual", true);
%   result = react("trainedAgent.mat", mission);
%
% WEBWRITE PATCH POINTS are marked below:
%   1) mission_start
%   2) frame_stream (inside main movement loop)
%   3) mission_end

clc;
close all;

if nargin < 2 || isempty(mission)
    mission = struct();
end

load(agentFile, "agent");

ENV0 = environment_engine_cached();
P    = get_comm_params();

%% ================= INPUTS =================
src            = get_field(mission, "src", 9);
dst            = get_field(mission, "dst", 15);
organ          = string(get_field(mission, "organ", "Unknown"));
CIT_total      = get_field(mission, "CIT_total", 15000);
batteryStart   = get_field(mission, "batteryPercent", 100);
uavSpeedMS     = get_field(mission, "uavSpeedMS", 25);
enableVisual   = logical(get_field(mission, "enableVisual", true));
enableFailure  = logical(get_field(mission, "enableFailure", true));

% Optional web streaming configuration
webEnable      = logical(get_field(mission, "webEnable", false));
webBase        = string(get_field(mission, "webBase", "http://localhost:8000"));
webTimeoutSec  = get_field(mission, "webTimeoutSec", 1);
webOpts        = weboptions("MediaType", "application/json", "Timeout", webTimeoutSec);

%% ================= CONFIG =================
cfg.stepMeters_nom = get_field(mission, "stepMeters_nom", 200);
cfg.uavSpeedMS     = uavSpeedMS;
cfg.dtSec          = cfg.stepMeters_nom / cfg.uavSpeedMS;
cfg.maxSteps       = get_field(mission, "maxSteps", 350);

SINR_min = get_field(mission, "SINR_min", 5);

allLat = [ENV0.Hospitals.Latitude; ENV0.Relays.Latitude];
allLon = [ENV0.Hospitals.Longitude; ENV0.Relays.Longitude];

cfg.latMin = min(allLat) - 0.01;
cfg.latMax = max(allLat) + 0.01;
cfg.lonMin = min(allLon) - 0.01;
cfg.lonMax = max(allLon) + 0.01;

%% ================= BATTERY =================
energy_total = get_field(mission, "energy_total", 100000);
energy_per_meter = get_field(mission, "energy_per_meter", 1.25);

batteryStart = max(min(batteryStart, 100), 0);
energy_remaining = energy_total * (batteryStart / 100);
battery_remaining = batteryStart;

%% ================= RELAY FAILURE =================
failureRelays = get_field(mission, "failureRelays", [6, 7, 9]);
failureTimes  = get_field(mission, "failureTimes", [2600, 1400, 1500]);
failedRelays  = false(height(ENV0.Relays), 1);

%% ================= SRC DST =================
lat = ENV0.Hospitals.Latitude(src);
lon = ENV0.Hospitals.Longitude(src);

dstLat = ENV0.Hospitals.Latitude(dst);
dstLon = ENV0.Hospitals.Longitude(dst);

obsDim = agent.ObservationInfo.Dimension(1);

fprintf("Source: %d | Destination: %d | Organ: %s\n", src, dst, organ);

%% WEBWRITE PATCH POINT 1: mission_start
startPayload = struct( ...
    "event", "mission_start", ...
    "src", src, ...
    "dst", dst, ...
    "organ", char(organ), ...
    "citSec", CIT_total, ...
    "batteryPercent", batteryStart, ...
    "uavSpeedMS", cfg.uavSpeedMS);
safe_webwrite(webEnable, webBase + "/mission/start", startPayload, webOpts);

%% ================= DRL PATH =================
latPath = zeros(cfg.maxSteps, 1);
lonPath = zeros(cfg.maxSteps, 1);
stoppedEarly = false;

for step = 1:cfg.maxSteps

    latPath(step) = lat;
    lonPath(step) = lon;

    obs = zeros(obsDim, 1);

    [xN, yN] = norm_xy(lat, lon, cfg);
    [dstxN, dstyN] = norm_xy(dstLat, dstLon, cfg);
    distKm = haversine_km(lat, lon, dstLat, dstLon);
    distN  = min(distKm / 20, 1);

    obs(1:5) = [xN; yN; dstxN; dstyN; distN];

    act = getAction(agent, obs);
    a   = extract_action(act);

    stepMeters = cfg.uavSpeedMS * cfg.dtSec;
    [lat, lon] = apply_discrete_action(lat, lon, a, stepMeters);

    if haversine_km(lat, lon, dstLat, dstLon) < 0.3
        latPath = latPath(1:step);
        lonPath = lonPath(1:step);
        stoppedEarly = true;
        break;
    end
end

if ~stoppedEarly
    latPath = latPath(latPath ~= 0);
    lonPath = lonPath(lonPath ~= 0);
end

%% ================= SMOOTH =================
validIdx = latPath ~= 0;
latPath = latPath(validIdx);
lonPath = lonPath(validIdx);

nCtrl = min(10, length(latPath));
idx   = round(linspace(1, length(latPath), nCtrl));

latCtrl = latPath(idx);
lonCtrl = lonPath(idx);

latCtrl(end + 1) = dstLat;
lonCtrl(end + 1) = dstLon;

t  = 1:length(latCtrl);
tt = linspace(1, length(latCtrl), 150);

latSmooth = interp1(t, latCtrl, tt, "pchip");
lonSmooth = interp1(t, lonCtrl, tt, "pchip");

%% ================= VISUAL =================
if enableVisual
    figure;
    gx = geoaxes;
    hold(gx, "on");
    geobasemap(gx, "satellite");
    disableDefaultInteractivity(gx);

    geoscatter(gx, ENV0.Relays.Latitude, ENV0.Relays.Longitude, 20, "k", "filled");
    geoscatter(gx, ENV0.Hospitals.Latitude, ENV0.Hospitals.Longitude, 40, "r", "filled");

    geoscatter(gx, latCtrl(1), lonCtrl(1), 150, "g", "filled");
    geoscatter(gx, dstLat, dstLon, 150, "m", "filled");

    uavPlot  = geoscatter(gx, latSmooth(1), lonSmooth(1), 100, "b", "filled");
    pathPlot = geoplot(gx, latSmooth(1), lonSmooth(1), "b", "LineWidth", 3);

    relayColors = repmat([0 0 0], height(ENV0.Relays), 1);
    relayPlot = geoscatter(gx, ...
        ENV0.Relays.Latitude, ...
        ENV0.Relays.Longitude, ...
        30, relayColors, "filled");
else
    gx = [];
    uavPlot = [];
    pathPlot = [];
    relayPlot = [];
    relayColors = repmat([0 0 0], height(ENV0.Relays), 1);
end

prevRelayIdx = -1;
initialConnected = false;
noLinkCounter = 0;

%% ================= DIST =================
totalDist_km = 0;
prevLat = latSmooth(1);
prevLon = lonSmooth(1);

latHist = latSmooth(1);
lonHist = lonSmooth(1);

finalReason = "Simulation Complete";
mode = "NO LINK";
snr = -100;

%% ================= MOVE =================
for i = 2:length(latSmooth)

    lat = latSmooth(i);
    lon = lonSmooth(i);

    timeElapsed = (i / length(latSmooth)) * (length(latPath) * cfg.dtSec);
    CIT_remaining = max(CIT_total - timeElapsed, 0);

    if enableFailure
        for k = 1:length(failureRelays)
            if timeElapsed >= failureTimes(k) && ~failedRelays(failureRelays(k))
                failedRelays(failureRelays(k)) = true;
                fprintf("RELAY %d FAILED at %.2f sec\n", failureRelays(k), timeElapsed);
            end
        end
    end

    stepDist_km = haversine_km(prevLat, prevLon, lat, lon);
    totalDist_km = totalDist_km + stepDist_km;

    energy_used = stepDist_km * 1000 * energy_per_meter;
    energy_remaining = max(energy_remaining - energy_used, 0);
    battery_remaining = (energy_remaining / energy_total) * 100;

    prevLat = lat;
    prevLon = lon;

    relayTable = uav_to_relays(ENV0, P, lat, lon);

    for r = find(failedRelays)'
        if r <= height(relayTable)
            relayTable.SNR_dB(r) = -100;
            relayTable.Delay_s(r) = inf;
        end
    end

    validIdx = (relayTable.SNR_dB > SINR_min) & (relayTable.Delay_s < 5);

    if any(validIdx)
        validRelays = relayTable(validIdx, :);
        [~, bestLocalIdx] = max(validRelays.SNR_dB);
        originalIdx = find(validIdx);
        bestRelayIdx = originalIdx(bestLocalIdx);

        snr   = relayTable.SNR_dB(bestRelayIdx);
        delay = relayTable.Delay_s(bestRelayIdx);
        mode  = adaptive_AF_DF_switch(snr, delay, 3);
    else
        bestRelayIdx = -1;
        mode = "NO LINK";
        snr = -100;
    end

    if (bestRelayIdx == -1 || snr < SINR_min) && initialConnected
        noLinkCounter = noLinkCounter + 1;
        fprintf("LINK ISSUE | SINR: %.2f | Count: %d\n", snr, noLinkCounter);

        if noLinkCounter > 10
            finalReason = "Network Failure";
            print_landing(lat, lon, finalReason);
            break;
        end
    else
        noLinkCounter = 0;
    end

    if bestRelayIdx ~= prevRelayIdx && bestRelayIdx ~= -1
        if ~initialConnected
            fprintf("Initial Connection -> Relay %d | Mode: %s\n", bestRelayIdx, string(mode));
            initialConnected = true;
        else
            fprintf("Relay Handoff: %d -> %d | Mode: %s\n", prevRelayIdx, bestRelayIdx, string(mode));
        end
        prevRelayIdx = bestRelayIdx;
    end

    relayColors = repmat([0 0 0], height(ENV0.Relays), 1);

    for r = find(failedRelays)'
        relayColors(r, :) = [1 0.5 0];
    end

    if bestRelayIdx > 0
        relayColors(bestRelayIdx, :) = [0 1 0];
    end

    if enableVisual
        set(relayPlot, "CData", relayColors);
    end

    latHist = [latHist lat]; %#ok<AGROW>
    lonHist = [lonHist lon]; %#ok<AGROW>

    if enableVisual
        set(uavPlot, "LatitudeData", lat, "LongitudeData", lon);
        set(pathPlot, "LatitudeData", latHist, "LongitudeData", lonHist);

        title(gx, sprintf([ ...
            "Relay: %d | Mode: %s | SINR: %.1f dB\n" + ...
            "Time: %.1f sec | CIT: %.1f sec\n" + ...
            "Dist: %.2f km | Battery: %.1f %%"], ...
            prevRelayIdx, string(mode), snr, timeElapsed, CIT_remaining, ...
            totalDist_km, battery_remaining));

        drawnow limitrate;
        pause(0.05);
    end

    %% WEBWRITE PATCH POINT 2: frame_stream
    framePayload = struct( ...
        "event", "frame", ...
        "timeElapsed", timeElapsed, ...
        "citRemaining", CIT_remaining, ...
        "lat", lat, ...
        "lon", lon, ...
        "src", src, ...
        "dst", dst, ...
        "organ", char(organ), ...
        "relay", bestRelayIdx, ...
        "mode", char(string(mode)), ...
        "sinr", snr, ...
        "batteryPercent", battery_remaining, ...
        "distanceKm", totalDist_km, ...
        "networkIssueCount", noLinkCounter);
    safe_webwrite(webEnable, webBase + "/frame", framePayload, webOpts);

    if CIT_remaining <= 0
        finalReason = "CIT Expired";
        print_landing(lat, lon, finalReason);
        break;
    end

    if battery_remaining <= 5
        finalReason = "Battery Critical";
        print_landing(lat, lon, finalReason);
        break;
    end
end

fprintf("\nTOTAL DISTANCE: %.2f km\n", totalDist_km);
fprintf("BATTERY LEFT: %.1f %%\n", battery_remaining);
disp(finalReason);

%% WEBWRITE PATCH POINT 3: mission_end
endPayload = struct( ...
    "event", "mission_end", ...
    "src", src, ...
    "dst", dst, ...
    "organ", char(organ), ...
    "finalReason", char(finalReason), ...
    "totalDistanceKm", totalDist_km, ...
    "batteryLeftPercent", battery_remaining, ...
    "finalLat", lat, ...
    "finalLon", lon);
safe_webwrite(webEnable, webBase + "/mission/end", endPayload, webOpts);

result = struct( ...
    "finalReason", finalReason, ...
    "totalDistanceKm", totalDist_km, ...
    "batteryLeftPercent", battery_remaining, ...
    "finalLat", lat, ...
    "finalLon", lon);

end

function value = get_field(s, name, defaultValue)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    value = s.(name);
else
    value = defaultValue;
end
end

function safe_webwrite(isEnabled, endpoint, payload, opts)
if ~isEnabled
    return;
end
try
    webwrite(char(endpoint), payload, opts);
catch
    % Keep simulation running even if the web endpoint is unavailable.
end
end

function print_landing(lat, lon, reason)
fprintf("\nDRONE LANDED\n");
fprintf("Location: (%.6f, %.6f)\n", lat, lon);
fprintf("Reason: %s\n\n", reason);
end
