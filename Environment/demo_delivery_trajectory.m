function demo_delivery_trajectory()

clc; close all;

% -------------------------------------------------
% 1) Load Environment + Network
% -------------------------------------------------

ENV = environment_engine_cached();
P   = net_params_option1();
Net = build_relay_backbone(ENV, P);

% -------------------------------------------------
% 2) Define Mission
% -------------------------------------------------

srcHospital = 1;    % change if needed
dstHospital = 15;   % change if needed

airportLat = 12.9965918;
airportLon = 80.1708076;

% -------------------------------------------------
% 3) Build Cost Map (Comm + Goal Attraction)
% -------------------------------------------------

CostMap = build_multi_sink_cost_map( ...
    ENV, Net, P, ...
    srcHospital, dstHospital, ...
    airportLat, airportLon);

% -------------------------------------------------
% 4) Compute Smooth UAV Trajectory
% -------------------------------------------------

RawPath = compute_astar_path(CostMap, ENV, srcHospital, dstHospital);
Path    = smooth_path(RawPath);

% -------------------------------------------------
% 5) Plot on Real Satellite Map
% -------------------------------------------------

figure('Color','w','Position',[100 80 1400 750]);

gx = geoaxes;
geobasemap(gx,"satellite");
hold(gx,"on");

title(gx,"Delivery UAV Trajectory (Communication-Aware)");

geoscatter(gx, ENV.Relays.Latitude, ...
               ENV.Relays.Longitude, ...
               40,'b','filled');

geoscatter(gx, ENV.Hospitals.Latitude, ...
               ENV.Hospitals.Longitude, ...
               60,'w','filled');

geoscatter(gx, ENV.Hospitals.Latitude(srcHospital), ...
               ENV.Hospitals.Longitude(srcHospital), ...
               120,'g','filled');

geoscatter(gx, ENV.Hospitals.Latitude(dstHospital), ...
               ENV.Hospitals.Longitude(dstHospital), ...
               120,'r','filled');

geoplot(gx, Path.lat, Path.lon, ...
        'w-','LineWidth',3);

legend(gx,"Relays","Hospitals","Source","Destination","UAV Path", ...
       "Location","northoutside");

end