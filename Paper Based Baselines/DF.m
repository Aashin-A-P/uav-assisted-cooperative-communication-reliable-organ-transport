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

lat = cell2mat(H(:,2));
lon = cell2mat(H(:,3));
N = numel(lat);

%% ================== AIRPORT (optional) ==================
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

%% ================== LAND REGION ==================
NodesXY = [xH yH; xA yA];
idxHull = convhull(NodesXY(:,1),NodesXY(:,2));
landPoly = polyshape(NodesXY(idxHull,1),NodesXY(idxHull,2));

%% ================== CANDIDATE GRID ==================
gs = 2000;
[xg,yg] = meshgrid( ...
    min(NodesXY(:,1))-5000:gs:max(NodesXY(:,1))+5000, ...
    min(NodesXY(:,2))-5000:gs:max(NodesXY(:,2))+5000);

Cand = [xg(:) yg(:)];
inside = isinterior(landPoly, Cand(:,1), Cand(:,2));
CandXY = Cand(inside,:);

%% ================== PLACE RELAY UAVs ==================
K = 17;
RelayXY = greedy_kcenter(CandXY, K, mean(NodesXY,1));
[relayLat, relayLon] = m2latlon(RelayXY(:,1), RelayXY(:,2));

%% ================== PARAMETERS ==================
accessRange = 7000;      % 7 km (DUAV ↔ Relay only)
step_m = 500;
switchMargin_m = 250;

alpha_access = 3;
alpha_backhaul = 2.5;

Pt_access = 10^(10/10);
Pt_backhaul = 10^(15/10);

gamma_th = 1;            % outage threshold (0 dB)

%% ================== MISSION ==================
srcHosp = 1;
dstHosp = 15;

srcXY = [xH(srcHosp) yH(srcHosp)];
dstXY = [xH(dstHosp) yH(dstHosp)];

%% ================== DELIVERY UAV TRAJECTORY ==================
distTot = norm(dstXY - srcXY);
numSteps = ceil(distTot/step_m)+1;

trajX = linspace(srcXY(1), dstXY(1), numSteps);
trajY = linspace(srcXY(2), dstXY(2), numSteps);
traj = [trajX(:) trajY(:)];

[trajLat, trajLon] = m2latlon(traj(:,1), traj(:,2));

%% ================== DYNAMIC RELAY ASSOCIATION ==================
connectedRelay = nan(numSteps,1);
currentRelay = nan;

for t = 1:numSteps
    p = traj(t,:);
    dists = vecnorm(RelayXY - p,2,2);
    [dmin, idxMin] = min(dists);

    if isnan(currentRelay)
        if dmin <= accessRange
            currentRelay = idxMin;
        end
    else
        dCur = dists(currentRelay);
        if dCur > accessRange && dmin <= accessRange
            currentRelay = idxMin;
        elseif idxMin ~= currentRelay && dmin + switchMargin_m < dCur
            currentRelay = idxMin;
        end
    end
    connectedRelay(t) = currentRelay;
end

relaySeq = connectedRelay(~isnan(connectedRelay));
relaySeq = relaySeq([true; diff(relaySeq)~=0]);

disp("Relay hopping sequence:");
disp(relaySeq.');

%% ================== BACKHAUL PATH ==================
pathBackhaul = relaySeq;

hopXY = [srcXY; RelayXY(pathBackhaul,:); dstXY];
[hopLat, hopLon] = m2latlon(hopXY(:,1), hopXY(:,2));
numHops = size(hopXY,1)-1;

%% ================== DF END-TO-END SNR ==================
gamma_hops = zeros(numHops,1);

for h = 1:numHops
    d = norm(hopXY(h+1,:) - hopXY(h,:));
    fading = exprnd(1);
    if h==1
        gamma_hops(h) = Pt_access * fading * d^(-alpha_access);
    else
        gamma_hops(h) = Pt_backhaul * fading * d^(-alpha_backhaul);
    end
end

gamma_DF = min(gamma_hops);
isOutage = gamma_DF < gamma_th;

disp("DF end-to-end SNR = " + gamma_DF);
disp("Outage? " + string(isOutage));

%% ================== MAP + MP4 ==================
fig = figure('Color','w','Position',[80 60 1500 850]);
gx = geoaxes(fig);
geobasemap(gx,"satellite");
hold(gx,'on');
title(gx,"Trajectory-Based DF Hopping (Map View)");

geoscatter(gx, lat, lon, 80,'y','filled');
geoscatter(gx, relayLat, relayLon, 55,'b','filled');
geoscatter(gx, airportLat, airportLon,140,'r','^','filled');

geoplot(gx, trajLat, trajLon,'k--','LineWidth',1.8);

v = VideoWriter("DF_Trajectory_Backhaul_Map.mp4","MPEG-4");
v.FrameRate = 12;
open(v);

hUAV = geoscatter(gx,trajLat(1),trajLon(1),120,'c','filled');
hLink = geoplot(gx,[trajLat(1) trajLat(1)], ...
                   [trajLon(1) trajLon(1)],'r-','LineWidth',2);

for t = 1:numSteps
    set(hUAV,'LatitudeData',trajLat(t),'LongitudeData',trajLon(t));
    r = connectedRelay(t);
    if ~isnan(r)
        set(hLink,'LatitudeData',[trajLat(t) relayLat(r)], ...
                  'LongitudeData',[trajLon(t) relayLon(r)]);
    end
    drawnow;
    writeVideo(v,getframe(fig));
end

for h = 1:numHops
    geoplot(gx, hopLat(h:h+1), hopLon(h:h+1),'b-','LineWidth',3);
    writeVideo(v,getframe(fig));
end

close(v);
disp("✅ Saved: DF_Trajectory_Backhaul_Map.mp4");

%% ================== FUNCTION ==================
function RelayXY = greedy_kcenter(CandXY, K, seedXY)
    RelayXY = zeros(K,2);
    RelayXY(1,:) = seedXY;
    distToSet = vecnorm(CandXY - seedXY,2,2);
    for i = 2:K
        [~,idx] = max(distToSet);
        RelayXY(i,:) = CandXY(idx,:);
        newDist = vecnorm(CandXY - RelayXY(i,:),2,2);
        distToSet = min(distToSet,newDist);
    end
end