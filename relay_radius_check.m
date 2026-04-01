clc; clear; close all;

outDir = "relay_coverage_latest";
if ~exist(outDir,"dir"), mkdir(outDir); end

%% ================== CONSTANTS ==================
R = 6371000;
BW = 10e6;                  % 10 MHz
Pt_dBm = 30;                % UAV Tx power
Noise_dBm = -174 + 10*log10(BW);

theta = linspace(0,2*pi,250);
radiusRange_km = 1:10;

%% ================== HOSPITAL + AIRPORT ==================
H = {
    "MIOT",13.0212213,80.1839951
    "Royapettah",13.0554981,80.2648512
    "RGGGH",13.0815311,80.2776431
    "SRMC",13.0392523,80.1434904
    "Frontline",13.1022933,80.1903163
    "Velammal",13.0792928,80.1144087
    "Kamakshi",12.9518467,80.2094059
    "Gleneagles",12.8980580,80.2062485
    "Chettinad",12.7968561,80.2184882
    "Balaji",12.9554166,80.1378104
    "SRM",12.8210435,80.0481410
    "Hindu",12.9238515,80.1141072
    "Saveetha",13.0264032,80.0139753
    "Tagore",12.8603845,80.1361455
    "Karpagam",13.2006399,79.8908348
};

lat = cell2mat(H(:,2));
lon = cell2mat(H(:,3));

airportLat = 12.9965918;
airportLon = 80.1708076;

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
landArea = area(landPoly);

%% ================== RELAY UAV PLACEMENT ==================
gs = 2000;
[xg,yg] = meshgrid(min(NodesXY(:,1))-5e3:gs:max(NodesXY(:,1))+5e3, ...
                   min(NodesXY(:,2))-5e3:gs:max(NodesXY(:,2))+5e3);

Cand = [xg(:) yg(:)];
Cand = Cand(isinterior(landPoly,Cand(:,1),Cand(:,2)),:);

K = 17;
RelayXY = greedy_kcenter(Cand,K,mean(NodesXY,1));
[relayLat, relayLon] = m2latlon(RelayXY(:,1),RelayXY(:,2));

%% ================== METRIC STORAGE ==================
coveragePct = zeros(1,10);
uncoveredPct = zeros(1,10);
avgSINR = zeros(1,10);
avgThroughput = zeros(1,10);

%% ================== LOOP OVER RADII ==================
for r = radiusRange_km
    Rm = r*1000;
    coverPoly = polyshape;

    for k = 1:K
        xc = RelayXY(k,1) + Rm*cos(theta);
        yc = RelayXY(k,2) + Rm*sin(theta);
        coverPoly = union(coverPoly, polyshape(xc,yc));
    end

    coveredArea = area(intersect(coverPoly, landPoly));
    coveragePct(r) = coveredArea/landArea*100;
    uncoveredPct(r) = 100 - coveragePct(r);

    %% ---- SINR + Throughput ----
    d = pdist2(NodesXY, RelayXY);
    PL = 32.4 + 20*log10(d/1000) + 20*log10(2.4e9/1e9);
    Pr = Pt_dBm - PL;
    SINR = Pr - Noise_dBm;
    avgSINR(r) = mean(max(SINR,[],'all'));
    avgThroughput(r) = BW*log2(1+10^(avgSINR(r)/10))/1e6;

    %% ---- SAVE COVERAGE MAP ----
    fig = figure('Visible','off');
    gx = geoaxes(fig);
    geobasemap(gx,"satellite"); hold(gx,'on');
    title(gx,"Relay Coverage – "+r+" km");

    for k = 1:K
        xc = RelayXY(k,1) + Rm*cos(theta);
        yc = RelayXY(k,2) + Rm*sin(theta);
        [clat,clon] = m2latlon(xc,yc);
        geoplot(gx, geopolyshape(clat,clon), ...
            'FaceColor',[0.4 0.8 1],'FaceAlpha',0.25,'EdgeColor','none');
    end

    geoscatter(gx, relayLat, relayLon, 30,'b','filled');
    geoscatter(gx, lat, lon, 60,'y','filled');
    geoscatter(gx, airportLat, airportLon,120,'r','^','filled');

    saveas(fig, fullfile(outDir,"Coverage_"+r+"km.png"));
    close(fig)
end

%% ================== FINAL GRAPHS ==================
figure; plot(radiusRange_km,coveragePct,'-o','LineWidth',2);
xlabel("Radius (km)"); ylabel("Coverage (%)"); grid on;
saveas(gcf,fullfile(outDir,"Coverage_vs_Radius.png"));

figure; plot(radiusRange_km,uncoveredPct,'-s','LineWidth',2);
xlabel("Radius (km)"); ylabel("Uncovered (%)"); grid on;
saveas(gcf,fullfile(outDir,"Uncovered_vs_Radius.png"));

figure; plot(radiusRange_km,avgSINR,'-^','LineWidth',2);
xlabel("Radius (km)"); ylabel("Avg SINR (dB)"); grid on;
saveas(gcf,fullfile(outDir,"SINR_vs_Radius.png"));

figure; plot(radiusRange_km,avgThroughput,'-d','LineWidth',2);
xlabel("Radius (km)"); ylabel("Throughput (Mbps)"); grid on;
saveas(gcf,fullfile(outDir,"Throughput_vs_Radius.png"));

disp("✅ All coverage comparison plots saved.");

%% ================== FUNCTION ==================
function RelayXY = greedy_kcenter(CandXY,K,seed)
    RelayXY=zeros(K,2);
    RelayXY(1,:)=seed;
    d=vecnorm(CandXY-seed,2,2);
    for i=2:K
        [~,idx]=max(d);
        RelayXY(i,:)=CandXY(idx,:);
        d=min(d,vecnorm(CandXY-RelayXY(i,:),2,2));
    end
end