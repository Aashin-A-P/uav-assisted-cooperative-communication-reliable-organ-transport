function plot_cost_map(CostMap, ENV)

figure('Color','w','Position',[100 80 1200 800]);

imagesc(CostMap.lonv, CostMap.latv, CostMap.Cost);
set(gca,'YDir','normal');
colorbar;
colormap(turbo);

xlabel('Longitude');
ylabel('Latitude');
title('Multi-Sink Communication Cost Map');

hold on;

% Overlay relays
scatter(ENV.Relays.Longitude, ENV.Relays.Latitude, 30, 'k', 'filled');

% Overlay hospitals
scatter(ENV.Hospitals.Longitude, ENV.Hospitals.Latitude, 60, 'w', 'filled');

end