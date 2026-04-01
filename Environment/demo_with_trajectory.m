ENV = environment_engine_cached();
P   = net_params_option1();
Net = build_relay_backbone(ENV, P);

airportLat = 12.9965918;
airportLon = 80.1708076;

srcHospital = 1;
dstHospital = 15;

CostMap = build_multi_sink_cost_map(ENV, Net, P, ...
                                     srcHospital, dstHospital, ...
                                     airportLat, airportLon);

Path = compute_smooth_trajectory(CostMap, ENV, ...
                                  srcHospital, dstHospital);

figure;
imagesc(CostMap.lonv, CostMap.latv, CostMap.Cost);
set(gca,'YDir','normal');
colorbar;
hold on;

plot(Path.lon, Path.lat, 'w-', 'LineWidth', 3);

title('Communication-Aware Smooth UAV Trajectory');
xlabel('Longitude');
ylabel('Latitude');