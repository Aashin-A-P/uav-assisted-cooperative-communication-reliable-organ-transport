clc; clear; close all;

ENV = environment_engine_cached();
P   = net_params_option1();
Net = build_relay_backbone(ENV, P);

srcHospital = 1;
dstHospital = 15;

CostMap = build_cost_map_multisink(ENV, Net, P, srcHospital, dstHospital);

plot_cost_map(CostMap, ENV);

Path = compute_smooth_trajectory(CostMap, ENV, srcHospital, dstHospital);

hold on;
plot(Path.lon, Path.lat, 'w-', 'LineWidth',3);