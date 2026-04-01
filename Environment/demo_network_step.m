ENV = environment_engine_cached();   % your environment + map
P = net_params_option1();

Net = build_relay_backbone(ENV, P);

% Example: UAV position
uavLat = ENV.Hospitals.Latitude(1);
uavLon = ENV.Hospitals.Longitude(1);

U = uav_to_relays(ENV, P, uavLat, uavLon);

sourceHospital = 1;
destHospital   = 15;

destRelayIdx = hospital_to_nearest_relay(ENV, destHospital);

% Choose best relay to attach (single-relay baseline)
prevRelayIdx = [];
out = best_relay_attach_baseline(Net, U, destRelayIdx, prevRelayIdx);

fprintf("Best relay to attach: Relay Index %d, estimated cost %.3f s\n", ...
    out.bestRelayIdx, out.bestCost_s);