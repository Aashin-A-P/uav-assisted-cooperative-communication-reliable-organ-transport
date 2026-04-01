function simulate_relay_failure_midflight()

clc; close all;

% -------------------------------------------------
% 1) Load Environment
% -------------------------------------------------
ENV = environment_engine_cached();
P   = net_params_option1();

srcHospital = 1;
dstHospital = 15;

airportLat = 12.9965918;
airportLon = 80.1708076;

% -------------------------------------------------
% 2) Initial Backbone + Cost
% -------------------------------------------------
Net = build_relay_backbone(ENV, P);

CostMap = build_multi_sink_cost_map( ...
    ENV, Net, P, ...
    srcHospital, dstHospital, ...
    airportLat, airportLon);

RawPath = compute_astar_path(CostMap, ENV, srcHospital, dstHospital);
Path    = smooth_path(RawPath);

% -------------------------------------------------
% 3) Choose Relay to Fail
% -------------------------------------------------
relayToFail = 7;   % change if needed
failureStep = round(length(Path.lat)/2);

fprintf("\nRelay %d will fail at step %d\n\n", relayToFail, failureStep);

% -------------------------------------------------
% 4) Plot Setup
% -------------------------------------------------
figure('Color','w','Position',[100 80 1400 750]);

gx = geoaxes;
geobasemap(gx,"satellite");
hold(gx,"on");

title(gx,"Relay Failure Mid-Flight Simulation");

% Plot relays
relayPlot = geoscatter(gx, ENV.Relays.Latitude, ...
                           ENV.Relays.Longitude, ...
                           50,'b','filled');

% Plot hospitals
geoscatter(gx, ENV.Hospitals.Latitude, ...
               ENV.Hospitals.Longitude, ...
               60,'w','filled');

% Mark source and destination
geoscatter(gx, ENV.Hospitals.Latitude(srcHospital), ...
               ENV.Hospitals.Longitude(srcHospital), ...
               120,'g','filled');

geoscatter(gx, ENV.Hospitals.Latitude(dstHospital), ...
               ENV.Hospitals.Longitude(dstHospital), ...
               120,'r','filled');

% UAV marker
uavMarker = geoscatter(gx, Path.lat(1), Path.lon(1), ...
                           100,'y','filled');

drawnow;

% -------------------------------------------------
% 5) Simulate Flight
% -------------------------------------------------
for k = 1:length(Path.lat)
    
    % Move UAV
    set(uavMarker,'LatitudeData',Path.lat(k), ...
                  'LongitudeData',Path.lon(k));
    
    drawnow;
    pause(0.05);
    
    % -------- Relay Failure Trigger --------
    if k == failureStep
        
        fprintf(">>> Relay %d FAILED!\n", relayToFail);
        
        % Remove relay from ENV
        ENV.Relays(relayToFail,:) = [];
        
        % Rebuild backbone
        Net = build_relay_backbone(ENV, P);
        
        % Recompute cost
        CostMap = build_multi_sink_cost_map( ...
            ENV, Net, P, ...
            srcHospital, dstHospital, ...
            airportLat, airportLon);
        
        % Replan from current UAV position
        currentLat = Path.lat(k);
        currentLon = Path.lon(k);
        
        % Temporarily overwrite source hospital position
        ENV.Hospitals.Latitude(srcHospital)  = currentLat;
        ENV.Hospitals.Longitude(srcHospital) = currentLon;
        
        RawPath = compute_astar_path(CostMap, ENV, srcHospital, dstHospital);
        Path    = smooth_path(RawPath);
        
        % Reset loop index
        k = 1;
    end
end

fprintf("\nMission Completed with Relay Failure Recovery\n");

end