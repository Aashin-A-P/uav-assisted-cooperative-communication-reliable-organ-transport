clc; clear; close all;

% ----------------- Load environment + params -----------------
ENV = environment_engine_cached();       % shows map
P   = net_params_option1();              % Option 1 settings
Net = build_relay_backbone(ENV, P);      % relay-relay mesh

% ----------------- Choose mission -----------------
srcHospital = 1;
dstHospital = 15;

% UAV starts at source hospital and moves to destination hospital
srcLat = ENV.Hospitals.Latitude(srcHospital);
srcLon = ENV.Hospitals.Longitude(srcHospital);

dstLat = ENV.Hospitals.Latitude(dstHospital);
dstLon = ENV.Hospitals.Longitude(dstHospital);

% Destination gateway relay (nearest relay to destination hospital)
destRelayIdx = hospital_to_nearest_relay(ENV, dstHospital);

% ----------------- Trajectory settings -----------------
T = 50;                     % number of steps (increase for smoother)
latPath = linspace(srcLat, dstLat, T);
lonPath = linspace(srcLon, dstLon, T);

bestRelay = zeros(T,1);
bestCost  = zeros(T,1);
handoffs  = 0;
prevRelayIdx = [];

% ----------------- Simulate -----------------
for t = 1:T
    uavLat = latPath(t);
    uavLon = lonPath(t);

    U = uav_to_relays(ENV, P, uavLat, uavLon);
    out = best_relay_attach_baseline(Net, U, destRelayIdx, prevRelayIdx);

    bestRelay(t) = out.bestRelayIdx;
    bestCost(t)  = out.bestCost_s;

    if t > 1 && bestRelay(t) ~= bestRelay(t-1)
        handoffs = handoffs + 1;
    end

    prevRelayIdx = bestRelay(t);

    fprintf("Step %d/%d | UAV(%.5f, %.5f) | Best Relay=%d | Cost=%.4f s\n", ...
        t, T, uavLat, uavLon, bestRelay(t), bestCost(t));
end

fprintf("\nMission done. Total handoffs = %d\n", handoffs);

% ----------------- Plot handoff timeline -----------------
plot_handoff_timeline(bestRelay, bestCost);

% ----------------- Optional: show trajectory on map -----------------
plot_uav_trajectory_on_env(ENV, latPath, lonPath, bestRelay);