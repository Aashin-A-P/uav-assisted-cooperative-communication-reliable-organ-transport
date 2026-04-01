function plot_multihop_paths_on_env(ENV, latPath, lonPath, bestRelaySeq, paths, CTRL, stepToPlot)

hosp = ENV.Hospitals;
rel  = ENV.Relays;

figure('Color','w','Position',[100 80 1500 780]);
gx = geoaxes;
geobasemap(gx,"satellite");
hold(gx,"on");

title(gx,"UAV Trajectory + Access Relay + Multi-hop Routing");

% Hospitals
geoscatter(gx, hosp.Latitude, hosp.Longitude, 70, 'y', 'filled');

% Relays
geoscatter(gx, rel.Latitude, rel.Longitude, 40, 'b', 'filled');

% Airport
geoscatter(gx, CTRL.Latitude, CTRL.Longitude, 90, 'g', 'filled');

% UAV trajectory
geoplot(gx, latPath, lonPath, 'w-', 'LineWidth', 2);

% Handoff points
handoffChanges = find(diff(bestRelaySeq) ~= 0) + 1;
handoffIdx = unique([1; handoffChanges]);
geoscatter(gx, latPath(handoffIdx), lonPath(handoffIdx), 80, 'r', 'filled');

% Default: show final step routing
if nargin < 7 || isempty(stepToPlot)
    stepToPlot = numel(bestRelaySeq);
end

plot_one_path(gx, rel, paths.dst{stepToPlot});
plot_one_path(gx, rel, paths.src{stepToPlot});
plot_one_path(gx, rel, paths.ctrl{stepToPlot});

legend(gx, ...
    "Hospitals","Relays","Airport","UAV Path","Handoff Points", ...
    "Location","northoutside");

grid(gx,"on");

end


function plot_one_path(gx, rel, nodeList)
if isempty(nodeList) || numel(nodeList) < 2
    return;
end
lat = rel.Latitude(nodeList);
lon = rel.Longitude(nodeList);
geoplot(gx, lat, lon, '-', 'LineWidth', 2);
end