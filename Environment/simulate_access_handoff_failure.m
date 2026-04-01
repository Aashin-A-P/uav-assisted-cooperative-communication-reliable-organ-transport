function simulate_access_handoff_failure()

clc; close all;

ENV = environment_engine_cached();
P   = net_params_option1();

srcHospital = 1;
dstHospital = 15;

airportLat = 12.9965918;
airportLon = 80.1708076;

% Build backbone
Net = build_relay_backbone(ENV, P);

% Build cost map + trajectory (fixed)
CostMap = build_multi_sink_cost_map( ...
    ENV, Net, P, ...
    srcHospital, dstHospital, ...
    airportLat, airportLon);

RawPath = compute_astar_path(CostMap, ENV, srcHospital, dstHospital);
Path    = smooth_path(RawPath);

% Choose relay to fail
relayToFail = 7;
failureStep = round(length(Path.lat)/2);

fprintf("Relay %d will fail at step %d\n", relayToFail, failureStep);

% Plot
figure('Color','w','Position',[100 80 1400 750]);
gx = geoaxes;
geobasemap(gx,"satellite");
hold(gx,"on");

geoscatter(gx, ENV.Relays.Latitude, ...
               ENV.Relays.Longitude, ...
               50,'b','filled');

geoscatter(gx, ENV.Hospitals.Latitude, ...
               ENV.Hospitals.Longitude, ...
               60,'w','filled');

geoscatter(gx, ENV.Hospitals.Latitude(srcHospital), ...
               ENV.Hospitals.Longitude(srcHospital), ...
               120,'g','filled');

geoscatter(gx, ENV.Hospitals.Latitude(dstHospital), ...
               ENV.Hospitals.Longitude(dstHospital), ...
               120,'r','filled');

uavMarker = geoscatter(gx, Path.lat(1), Path.lon(1), ...
                       120,'y','filled');

drawnow;

currentRelay = [];

for k = 1:length(Path.lat)
    
    lat = Path.lat(k);
    lon = Path.lon(k);
    
    % UAV scans nearby relays
    U = uav_to_relays(ENV, P, lat, lon);
    
    % Choose strongest SNR relay
    [~, bestRelay] = max(U.SNR_dB);
    
    currentRelay = bestRelay;
    
    % Trigger failure
    if k == failureStep
        
        fprintf("Relay %d FAILED\n", relayToFail);
        
        if relayToFail <= height(ENV.Relays)
            ENV.Relays(relayToFail,:) = [];
        end
        
        Net = build_relay_backbone(ENV, P);
    end
    
    % Update UAV marker
    set(uavMarker,'LatitudeData',lat, ...
                  'LongitudeData',lon);
    
    drawnow;
    pause(0.05);
end

fprintf("Mission Completed with Dynamic Relay Handoff\n");

end