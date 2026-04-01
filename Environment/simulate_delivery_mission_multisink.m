clc; clear; close all;

%% ---------------- Load Environment ----------------
ENV = environment_engine_cached();
P   = net_params_option1();
Net = build_relay_backbone(ENV, P);

%% ---------------- Mission Definition ----------------
srcHospital = 1;
dstHospital = 15;

% Airport control center coordinates
CTRL.Latitude  = 12.9965918;
CTRL.Longitude = 80.1708076;

% Gateways (nearest relay for each sink)
srcG  = hospital_to_nearest_relay(ENV, srcHospital);
dstG  = hospital_to_nearest_relay(ENV, dstHospital);
ctrlG = point_to_nearest_relay(ENV, CTRL.Latitude, CTRL.Longitude);

fprintf("\nGateways:\n");
fprintf("  Source Hospital H%d -> Relay %d\n", srcHospital, srcG);
fprintf("  Dest Hospital   H%d -> Relay %d\n", dstHospital, dstG);
fprintf("  Airport Control -> Relay %d\n\n", ctrlG);

%% ---------------- UAV Trajectory ----------------
T = 60;

srcLat = ENV.Hospitals.Latitude(srcHospital);
srcLon = ENV.Hospitals.Longitude(srcHospital);
dstLat = ENV.Hospitals.Latitude(dstHospital);
dstLon = ENV.Hospitals.Longitude(dstHospital);

latPath = linspace(srcLat, dstLat, T);
lonPath = linspace(srcLon, dstLon, T);

%% ---------------- Storage ----------------
bestRelay = zeros(T,1);
bestSNR   = zeros(T,1);
handoffs  = 0;
prevRelayIdx = [];

paths.dst  = cell(T,1);
paths.src  = cell(T,1);
paths.ctrl = cell(T,1);

%% ---------------- Simulation Loop ----------------
for t = 1:T

    uavLat = latPath(t);
    uavLon = lonPath(t);

    % UAV -> Relay links
    U = uav_to_relays(ENV, P, uavLat, uavLon);

    % Access relay selection (SNR based)
    out = best_access_relay_by_snr(U, prevRelayIdx);
    r = out.bestRelayIdx;

    bestRelay(t) = r;
    bestSNR(t)   = out.bestSNR_dB;

    if t > 1 && bestRelay(t) ~= bestRelay(t-1)
        handoffs = handoffs + 1;
    end

    % Multi-hop backbone routing
    paths.dst{t}  = shortestpath(Net.G, r, dstG);
    paths.src{t}  = shortestpath(Net.G, r, srcG);
    paths.ctrl{t} = shortestpath(Net.G, r, ctrlG);

    prevRelayIdx = r;

    fprintf("Step %d/%d | UAV(%.5f, %.5f) | AccessRelay=%d | SNR=%.2f dB | Hops(dst/src/ctrl)=%d/%d/%d\n", ...
        t, T, uavLat, uavLon, r, bestSNR(t), ...
        max(0,numel(paths.dst{t})-1), ...
        max(0,numel(paths.src{t})-1), ...
        max(0,numel(paths.ctrl{t})-1));
end

fprintf("\nMission done. Total handoffs = %d\n", handoffs);

%% ---------------- Plots ----------------
plot_handoff_timeline(bestRelay, bestSNR);

stepToPlot = T; % show routing at final step
plot_multihop_paths_on_env(ENV, latPath, lonPath, bestRelay, paths, CTRL, stepToPlot);