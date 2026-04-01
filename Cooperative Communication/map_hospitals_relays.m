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

hospNames = string(Th.Name);
hospLat   = Th.Latitude;
hospLon   = Th.Longitude;

relayId  = Tr.RelayID;
relayLat = Tr.Latitude;
relayLon = Tr.Longitude;

%% ================== MAP PLOT ==================
fig = figure('Color','w','Position',[80 80 1400 760]);
gx = geoaxes(fig);
geobasemap(gx,"satellite");
hold(gx,'on');

title(gx,"Hospitals and Relay UAV Points");

% Hospitals
geoscatter(gx, hospLat, hospLon, 65, 'y', 'filled');

% Relays
geoscatter(gx, relayLat, relayLon, 35, 'b', 'filled');

% Hospital labels
for i = 1:numel(hospLat)
    text(gx, hospLat(i)+0.0015, hospLon(i)+0.0015, ...
        "H"+string(i), 'Color','w', 'FontSize',9, 'FontWeight','bold');
end

% Relay labels
for k = 1:numel(relayLat)
    text(gx, relayLat(k)+0.0009, relayLon(k)+0.0009, ...
        "R"+string(relayId(k)), 'Color','w', 'FontSize',8, 'FontWeight','bold');
end

legend(gx, "Hospitals", "Relay UAVs", "Location","northoutside");

disp("Map generated: hospitals + relays (no coverage circles).");
