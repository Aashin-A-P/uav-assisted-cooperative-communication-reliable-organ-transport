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

%% ================== LAT/LON → METERS ==================
R = 6371000;
lat0 = mean([lat; airportLat])*pi/180;
lon0 = mean([lon; airportLon])*pi/180;

latlon2m = @(la,lo) deal( ...
    R*((lo*pi/180)-lon0).*cos(lat0), ...
    R*((la*pi/180)-lat0));

m2latlon = @(x,y) deal( ...
    (y/R + lat0)*180/pi, ...
    (x./(R*cos(lat0)) + lon0)*180/pi);

[xH,yH] = latlon2m(lat,lon);
[xA,yA] = latlon2m(airportLat,airportLon);

NodesXY = [xH yH; xA yA];

%% ================== LAND REGION ==================
idxHull = convhull(NodesXY(:,1),NodesXY(:,2));
landPoly = polyshape(NodesXY(idxHull,1),NodesXY(idxHull,2));

%% ================== CANDIDATE GRID ==================
gs = 2000;
xmin = min(NodesXY(:,1))-5000;
xmax = max(NodesXY(:,1))+5000;
ymin = min(NodesXY(:,2))-5000;
ymax = max(NodesXY(:,2))+5000;

[xg,yg] = meshgrid(xmin:gs:xmax, ymin:gs:ymax);
Cand = [xg(:) yg(:)];
inside = isinterior(landPoly, Cand(:,1), Cand(:,2));
CandXY = Cand(inside,:);

%% ================== PLACE RELAY UAVs ==================
K = 17;
outDir = "relay_coverage_final";
if ~exist(outDir,"dir"), mkdir(outDir); end

relayStoreMat = fullfile(outDir, "Relay17_Positions.mat");
relayStoreCsv = fullfile(outDir, "Relay17_Positions.csv");

if exist(relayStoreMat,"file")
    S = load(relayStoreMat, "RelayXY", "relayLat", "relayLon", "K");
    hasValidK = isfield(S,"K") && S.K == K;
    hasValidShape = isfield(S,"RelayXY") && size(S.RelayXY,1) == K;
    if hasValidK && hasValidShape
        RelayXY = S.RelayXY;
        relayLat = S.relayLat;
        relayLon = S.relayLon;
        fprintf("Loaded saved relay positions from: %s\n", relayStoreMat);
    else
        RelayXY = greedy_kcenter(CandXY, K, mean(NodesXY,1));
        [relayLat, relayLon] = m2latlon(RelayXY(:,1), RelayXY(:,2));
        save(relayStoreMat, "RelayXY", "relayLat", "relayLon", "K");
        Trelay = table((1:K)', relayLat(:), relayLon(:), RelayXY(:,1), RelayXY(:,2), ...
            'VariableNames',["RelayID","Latitude","Longitude","X_m","Y_m"]);
        writetable(Trelay, relayStoreCsv);
        fprintf("Relay cache mismatch. Regenerated and saved: %s\n", relayStoreMat);
    end
else
    RelayXY = greedy_kcenter(CandXY, K, mean(NodesXY,1));
    [relayLat, relayLon] = m2latlon(RelayXY(:,1), RelayXY(:,2));

    save(relayStoreMat, "RelayXY", "relayLat", "relayLon", "K");
    Trelay = table((1:K)', relayLat(:), relayLon(:), RelayXY(:,1), RelayXY(:,2), ...
        'VariableNames',["RelayID","Latitude","Longitude","X_m","Y_m"]);
    writetable(Trelay, relayStoreCsv);
    fprintf("Saved relay positions to: %s\n", relayStoreMat);
end

%% ================== COVERAGE PARAMETERS ==================
coverageRadius_km = 7;           % 7 km (your choice)
coverageRadius_m  = coverageRadius_km * 1000;
theta = linspace(0,2*pi,250);

%% ================== PLOT COVERAGE MAP ==================
fig = figure('Color','w','Position',[80 80 1400 760]);
gx = geoaxes(fig);
geobasemap(gx,"satellite");
hold(gx,'on');

title(gx, ...
    "Relay UAV Network Coverage (Translucent Blue – 7 km Radius)");

% -------- Draw Coverage Areas --------
for k = 1:K
    xc = RelayXY(k,1) + coverageRadius_m*cos(theta);
    yc = RelayXY(k,2) + coverageRadius_m*sin(theta);

    [clat, clon] = m2latlon(xc, yc);

    pgon = geopolyshape(clat, clon);

    geoplot(gx, pgon, ...
        'FaceColor',[0.4 0.8 1], ...
        'FaceAlpha',0.22, ...
        'EdgeColor','none');
end

% -------- Relay UAV Points --------
geoscatter(gx, relayLat, relayLon, 35, 'b', 'filled');

% -------- Hospitals --------
geoscatter(gx, lat, lon, 65, 'y', 'filled');

% -------- Airport --------
geoscatter(gx, airportLat, airportLon, 130, 'r', '^', 'filled');

% -------- Labels --------
for k = 1:K
    text(gx, relayLat(k)+0.0009, relayLon(k)+0.0009, ...
        "R"+string(k), 'Color','w','FontWeight','bold','FontSize',8);
end

for i = 1:N
    text(gx, lat(i)+0.0015, lon(i)+0.0015, ...
        "H"+string(i), 'Color','w','FontSize',9);
end

text(gx, airportLat+0.0015, airportLon+0.0015, ...
    "A1",'Color','c','FontWeight','bold','FontSize',10);

legend(gx, ...
    "Coverage Area", ...
    "Relay UAV", ...
    "Hospitals", ...
    "Airport", ...
    "Location","northoutside");

%% ================== SAVE ==================
saveas(fig, fullfile(outDir,"Relay17_Coverage_7km.png"));
savefig(fig, fullfile(outDir,"Relay17_Coverage_7km.fig"));

disp("✅ Translucent relay coverage map generated successfully.");

%% ================== FUNCTION ==================
function RelayXY = greedy_kcenter(CandXY, K, seedXY)
    RelayXY = zeros(K,2);
    RelayXY(1,:) = seedXY;
    distToSet = vecnorm(CandXY - seedXY,2,2);
    for i = 2:K
        [~, idx] = max(distToSet);
        RelayXY(i,:) = CandXY(idx,:);
        newDist = vecnorm(CandXY - RelayXY(i,:),2,2);
        distToSet = min(distToSet, newDist);
    end
end
