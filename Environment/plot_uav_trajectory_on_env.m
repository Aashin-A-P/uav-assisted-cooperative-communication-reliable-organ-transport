function plot_uav_trajectory_on_env(ENV, latPath, lonPath, bestRelay)

hosp = ENV.Hospitals;
rel  = ENV.Relays;

figure('Color','w','Position',[100 80 1400 750]);
gx = geoaxes;
geobasemap(gx,"satellite");
hold(gx,"on");

title(gx,"UAV Trajectory + Relay Handoff Points");

% Hospitals
geoscatter(gx, hosp.Latitude, hosp.Longitude, 70, 'y', 'filled');

% Relays
geoscatter(gx, rel.Latitude, rel.Longitude, 40, 'b', 'filled');

% UAV path
geoplot(gx, latPath, lonPath, 'w-', 'LineWidth', 2);

% Mark handoff points
handoffIdx = [1; find(diff(bestRelay)~=0)+1; numel(bestRelay)];
geoscatter(gx, latPath(handoffIdx), lonPath(handoffIdx), 60, 'r', 'filled');

% Labels (small)
for i = 1:height(hosp)
    text(gx, hosp.Latitude(i)+0.0015, hosp.Longitude(i)+0.0015, ...
        "H"+string(i), 'Color','w','FontSize',9,'FontWeight','bold');
end
for i = 1:height(rel)
    text(gx, rel.Latitude(i)+0.001, rel.Longitude(i)+0.001, ...
        "R"+string(i), 'Color','w','FontSize',8,'FontWeight','bold');
end

legend(gx,"Hospitals","Relay UAVs","UAV Path","Handoff Points","Location","northoutside");
grid(gx,"on");

end