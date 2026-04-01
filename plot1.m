clc; clear; close all;

%% ================== HOSPITAL SET ==================
H = {
    "MIOT Hospitals",13.0212213,80.1839951
    "Government Royapettah Hospital",13.0554981,80.2648512
    "Rajiv Gandhi Govt General Hospital",13.0815311,80.2776431
    "Sri Ramachandra Medical Centre",13.0392523,80.1434904
    "Frontline Hospital",13.1022933,80.1903163
    "Velammal Medical College",13.0792928,80.1144087
    "Dr Kamakshi Memorial",12.9518467,80.2094059
    "Gleneagles Health City",12.8980580,80.2062485
    "Chettinad Hospital",12.7968561,80.2184882
    "Sree Balaji Medical College",12.9554166,80.1378104
    "SRM Medical College",12.8210435,80.0481410
    "Hindu Mission Hospital",12.9238515,80.1141072
    "Saveetha Medical College",13.0264032,80.0139753
    "Tagore Medical College",12.8603845,80.1361455
    "Karpagam Hospital",13.2006399,79.8908348
};

names = string(H(:,1));
lat = cell2mat(H(:,2));
lon = cell2mat(H(:,3));
N = numel(lat);

%% ================== AIRPORT ==================
airportLat = 12.9965918;
airportLon = 80.1708076;

nodeNames = [names; "A1-Airport"];
nodeLat   = [lat; airportLat];
nodeLon   = [lon; airportLon];

%% ================== LAT/LON → METERS ==================
R = 6371000;
lat0 = mean(nodeLat)*pi/180;
lon0 = mean(nodeLon)*pi/180;

latlon2m = @(la,lo) deal( ...
    R*((lo*pi/180)-lon0).*cos(lat0), ...
    R*((la*pi/180)-lat0));

m2latlon = @(x,y) deal( ...
    (y/R + lat0)*180/pi, ...
    (x./(R*cos(lat0)) + lon0)*180/pi);

[xN,yN] = latlon2m(nodeLat,nodeLon);
NodesXY = [xN(:) yN(:)];

%% ================== LAND-ONLY REGION ==================
idxHull = convhull(NodesXY(:,1),NodesXY(:,2));
landPoly = polyshape(NodesXY(idxHull,1),NodesXY(idxHull,2));

%% ================== CANDIDATE GRID ==================
gridStep_km = 2.0;
gs = gridStep_km * 1000;

xmin = min(NodesXY(:,1)) - 5000;
xmax = max(NodesXY(:,1)) + 5000;
ymin = min(NodesXY(:,2)) - 5000;
ymax = max(NodesXY(:,2)) + 5000;

[xg,yg] = meshgrid(xmin:gs:xmax, ymin:gs:ymax);
Cand = [xg(:) yg(:)];

inside = isinterior(landPoly, Cand(:,1), Cand(:,2));
CandXY = Cand(inside,:);

%% ================== PLACE 17 RELAY UAVs ==================
K = 17;
RelayXY = greedy_kcenter(CandXY, K, mean(NodesXY,1));

[relayLat, relayLon] = m2latlon(RelayXY(:,1), RelayXY(:,2));

%% ================== PLOT SATELLITE MAP ==================
outDir = "relay_17_map";
if ~exist(outDir,"dir"), mkdir(outDir); end

fig = figure('Color','w','Position',[80 80 1300 720]);
gx = geoaxes(fig);
geobasemap(gx,"satellite");
hold(gx,'on');

title(gx,"17 Relay UAV Placement (Land-only, Blue Dots)");

% Hospitals (larger)
geoscatter(gx, lat, lon, 65, 'y', 'filled');

% Airport
geoscatter(gx, airportLat, airportLon, 130, 'r', '^', 'filled');

% Relay UAVs (smaller blue dots)
relayMarkerSize = 35;
geoscatter(gx, relayLat, relayLon, relayMarkerSize, 'b', 'filled');

% Hospital labels
for i = 1:N
    text(gx, lat(i)+0.0015, lon(i)+0.0015, ...
        "H"+string(i), 'Color','w','FontWeight','bold','FontSize',9);
end

% Airport label
text(gx, airportLat+0.0015, airportLon+0.0015, ...
    "A1",'Color','c','FontWeight','bold','FontSize',10);

% Relay labels R1–R17
for k = 1:K
    text(gx, relayLat(k)+0.0009, relayLon(k)+0.0009, ...
        "R"+string(k),'Color','w','FontWeight','bold','FontSize',8);
end

legend(gx,"Hospitals","Airport","Relay UAVs", ...
    "Location","northoutside");

saveas(fig, fullfile(outDir,"Relay17_BlueDots_R1toR17.png"));
savefig(fig, fullfile(outDir,"Relay17_BlueDots_R1toR17.fig"));

%% ================== SAVE RELAY COORDINATES ==================
T = table((1:K)', relayLat(:), relayLon(:), ...
    'VariableNames',["RelayID","Latitude","Longitude"]);
writetable(T, fullfile(outDir,"Relay17_Coordinates.csv"));

disp("✅ Relay UAV map + coordinates saved successfully.");

%% ================== LOCAL FUNCTION ==================
function RelayXY = greedy_kcenter(CandXY, K, seedXY)
    RelayXY = zeros(K,2);
    RelayXY(1,:) = seedXY;

    distToSet = sqrt(sum((CandXY - RelayXY(1,:)).^2,2));

    for i = 2:K
        [~, idx] = max(distToSet);
        RelayXY(i,:) = CandXY(idx,:);
        newDist = sqrt(sum((CandXY - RelayXY(i,:)).^2,2));
        distToSet = min(distToSet, newDist);
    end
end