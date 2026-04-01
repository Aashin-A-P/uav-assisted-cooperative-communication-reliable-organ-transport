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

hospLat   = Th.Latitude;
hospLon   = Th.Longitude;
relayId   = Tr.RelayID;
relayLat  = Tr.Latitude;
relayLon  = Tr.Longitude;

%% ================== COVERAGE CONFIG ==================
coverageRadius_km = 7;   % change here if needed

% Local map conversion centered on all points
R = 6371000;
lat0 = mean([hospLat; relayLat]) * pi/180;
lon0 = mean([hospLon; relayLon]) * pi/180;

latlon2m = @(la,lo) deal( ...
    R*((lo*pi/180)-lon0).*cos(lat0), ...
    R*((la*pi/180)-lat0));

m2latlon = @(x,y) deal( ...
    (y/R + lat0)*180/pi, ...
    (x./(R*cos(lat0)) + lon0)*180/pi);

[relayX, relayY] = latlon2m(relayLat, relayLon);
coverageRadius_m = coverageRadius_km * 1000;
theta = linspace(0,2*pi,250);

%% ================== MAP PLOT ==================
fig = figure('Color','w','Position',[80 80 1400 760]);
gx = geoaxes(fig);
geobasemap(gx,"satellite");
hold(gx,'on');

title(gx,"Hospitals, Relay UAVs, and Coverage (" + coverageRadius_km + " km)");

% Coverage circles
for k = 1:numel(relayLat)
    xc = relayX(k) + coverageRadius_m*cos(theta);
    yc = relayY(k) + coverageRadius_m*sin(theta);
    [clat, clon] = m2latlon(xc, yc);
    pgon = geopolyshape(clat, clon);
    geoplot(gx, pgon, ...
        'FaceColor',[0.4 0.8 1], ...
        'FaceAlpha',0.22, ...
        'EdgeColor','none');
end

% Hospitals
geoscatter(gx, hospLat, hospLon, 65, 'y', 'filled');

% Relays
geoscatter(gx, relayLat, relayLon, 35, 'b', 'filled');

% Labels
for i = 1:numel(hospLat)
    text(gx, hospLat(i)+0.0015, hospLon(i)+0.0015, ...
        "H"+string(i), 'Color','w', 'FontSize',9, 'FontWeight','bold');
end

for k = 1:numel(relayLat)
    text(gx, relayLat(k)+0.0009, relayLon(k)+0.0009, ...
        "R"+string(relayId(k)), 'Color','w', 'FontSize',8, 'FontWeight','bold');
end

legend(gx, "Coverage Area", "Hospitals", "Relay UAVs", "Location","northoutside");

disp("Map generated: hospitals + relays + coverage circles from CSV input.");
