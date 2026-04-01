clc; clear; close all;

%% ================== INPUT FILES ==================
scriptDir = fileparts(mfilename('fullpath'));
hospitalFile = fullfile(scriptDir, "hospital.csv");
relayFile    = fullfile(scriptDir, "relay.csv");

if ~isfile(hospitalFile)
    error("Missing file: %s", hospitalFile);
end
if ~isfile(relayFile)
    error("Missing file: %s", relayFile);
end

Th = readtable(hospitalFile, "TextType", "string");
Tr = readtable(relayFile, "TextType", "string");

requiredHospCols = ["Name","Latitude","Longitude"];
requiredRelayCols = ["RelayID","Latitude","Longitude"];

if ~all(ismember(requiredHospCols, string(Th.Properties.VariableNames)))
    error("hospital.csv must contain columns: Name, Latitude, Longitude");
end
if ~all(ismember(requiredRelayCols, string(Tr.Properties.VariableNames)))
    error("relay.csv must contain columns: RelayID, Latitude, Longitude");
end

hospLat = Th.Latitude(:);
hospLon = Th.Longitude(:);
relayLat = Tr.Latitude(:);
relayLon = Tr.Longitude(:);
relayId = Tr.RelayID;

%% ================== SETTINGS ==================
numWeatherPoints = 60;
gridStep_km = 2.0;  % candidate resolution for selecting spread-out API points

%% ================== LAT/LON <-> METERS ==================
R = 6371000;
allLat = [hospLat; relayLat];
allLon = [hospLon; relayLon];
lat0 = mean(allLat) * pi/180;
lon0 = mean(allLon) * pi/180;

latlon2m = @(la,lo) deal( ...
    R*((lo*pi/180)-lon0).*cos(lat0), ...
    R*((la*pi/180)-lat0));

m2latlon = @(x,y) deal( ...
    (y/R + lat0)*180/pi, ...
    (x./(R*cos(lat0)) + lon0)*180/pi);

[xAll,yAll] = latlon2m(allLat, allLon);
NodesXY = [xAll yAll];

%% ================== CANDIDATE REGION ==================
idxHull = convhull(NodesXY(:,1), NodesXY(:,2));
landPoly = polyshape(NodesXY(idxHull,1), NodesXY(idxHull,2));

gs = gridStep_km * 1000;
xmin = min(NodesXY(:,1)) - 5000;
xmax = max(NodesXY(:,1)) + 5000;
ymin = min(NodesXY(:,2)) - 5000;
ymax = max(NodesXY(:,2)) + 5000;

[xg, yg] = meshgrid(xmin:gs:xmax, ymin:gs:ymax);
Cand = [xg(:) yg(:)];
inside = isinterior(landPoly, Cand(:,1), Cand(:,2));
CandXY = Cand(inside,:);

if size(CandXY,1) < numWeatherPoints
    error("Only %d candidate points available. Decrease numWeatherPoints or gridStep_km.", size(CandXY,1));
end

%% ================== SELECT 60 SPREAD-OUT POINTS ==================
WeatherXY = farthest_point_sampling(CandXY, numWeatherPoints, mean(NodesXY,1));
[weatherLat, weatherLon] = m2latlon(WeatherXY(:,1), WeatherXY(:,2));

%% ================== SAVE WEATHER POINTS ==================
outCsv = fullfile(scriptDir, "weather_points_60.csv");
Tw = table((1:numWeatherPoints)', weatherLat(:), weatherLon(:), WeatherXY(:,1), WeatherXY(:,2), ...
    'VariableNames', ["PointID","Latitude","Longitude","X_m","Y_m"]);
writetable(Tw, outCsv);
fprintf("Saved 60 weather API points to: %s\n", outCsv);

%% ================== MAP PLOT ==================
fig = figure('Color','w','Position',[80 80 1400 760]);
gx = geoaxes(fig);
geobasemap(gx, "satellite");
hold(gx, 'on');

title(gx, "Weather API 60 Points + Hospitals + Relay UAVs");

% Hospitals
geoscatter(gx, hospLat, hospLon, 65, 'y', 'filled');

% Relays
geoscatter(gx, relayLat, relayLon, 35, 'b', 'filled');

% Weather API points
geoscatter(gx, weatherLat, weatherLon, 22, [1 0.3 0.1], 'filled');

% Optional labels (kept compact)
for i = 1:numel(hospLat)
    text(gx, hospLat(i)+0.0015, hospLon(i)+0.0015, ...
        "H"+string(i), 'Color','w', 'FontSize',8, 'FontWeight','bold');
end
for k = 1:numel(relayLat)
    text(gx, relayLat(k)+0.0009, relayLon(k)+0.0009, ...
        "R"+string(relayId(k)), 'Color','w', 'FontSize',7, 'FontWeight','bold');
end

legend(gx, "Hospitals", "Relay UAVs", "Weather API Points (60)", ...
    "Location", "northoutside");

disp("Map generated: hospitals + relay UAVs + 60 weather API points.");

%% ================== LOCAL FUNCTION ==================
function P = farthest_point_sampling(CandXY, K, seedXY)
    P = zeros(K,2);
    P(1,:) = seedXY;

    d = vecnorm(CandXY - P(1,:), 2, 2);
    for i = 2:K
        [~, idx] = max(d);
        P(i,:) = CandXY(idx,:);
        d = min(d, vecnorm(CandXY - P(i,:), 2, 2));
    end
end
