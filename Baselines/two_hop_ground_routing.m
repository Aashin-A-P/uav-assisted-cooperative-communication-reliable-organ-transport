clc; clear; close all;

%% ================= LOAD DATA =================
% hospital_data() must return:
% names, lat, lon, airportLat, airportLon, HospitalsXY (meters)
[names, lat, lon, airportLat, airportLon, HospitalsXY] = hospital_data();
N = size(HospitalsXY,1);

%% ================= ADD AIRPORT AS NODE (A1) =================
R = 6371000;
lat0 = mean(lat) * pi/180;
lon0 = mean(lon) * pi/180;

xA = R * ((airportLon*pi/180) - lon0) * cos(lat0);
yA = R * ((airportLat*pi/180) - lat0);
AirportXY = [xA yA];

nodeNames = [names; "A1-Airport"];
nodeLat   = [lat; airportLat];
nodeLon   = [lon; airportLon];
NodesXY   = [HospitalsXY; AirportXY];

M = size(NodesXY,1);    % hospitals + airport

%% ================= COMMUNICATION PARAMETERS =================
Pt_dBm = 20;
noise_dBm = -100;
pathLossExp = 2.5;

%% ================= UAV PARAMETERS (VISUAL ONLY) =================
uavSpeed = 20;      % m/s
dt = 50;            % seconds fast-forward
stepSize = uavSpeed * dt;

%% ================= OUTPUT FOLDER =================
outDir = "twohop_ground_baseline_results";
if ~exist(outDir,"dir"), mkdir(outDir); end

%% ================= STORAGE =================
DistanceMatrix_km = zeros(M,M);
SINR_Matrix_dB    = zeros(M,M);
RelayIndex        = zeros(M,M);   % which hospital chosen as relay (0 = none)

%% ================= METERS → LAT/LON =================
meters2latlon = @(x,y) deal( ...
    (y/R + lat0) * 180/pi, ...
    (x./(R*cos(lat0)) + lon0) * 180/pi );

%% ================= GEO FIGURE =================
fig = figure('Color','w','Position',[100 100 1300 720]);
gx = geoaxes(fig);
geobasemap(gx,"satellite");
hold(gx,'on');

%% ================= VIDEO WRITER =================
videoObj = VideoWriter( ...
    fullfile(outDir,"twohop_ground_with_airport_fastforward.mp4"), ...
    "MPEG-4");
videoObj.FrameRate = 15;
open(videoObj);

%% ================= MAIN LOOP =================
for src = 1:M
    for dst = 1:M

        if src == dst
            DistanceMatrix_km(src,dst) = 0;
            SINR_Matrix_dB(src,dst) = Inf;
            RelayIndex(src,dst) = 0;
            continue;
        end

        srcXY = NodesXY(src,:);
        dstXY = NodesXY(dst,:);

        %% ===== SELECT BEST RELAY HOSPITAL (ONLY FROM H1..HN) =====
        candidates = 1:N;
        candidates(candidates == src) = [];
        candidates(candidates == dst) = [];

        if isempty(candidates)
            % happens when src/dst are hospitals and N small; not your case
            RelayIndex(src,dst) = 0;
            continue;
        end

        bestRelay = candidates(1);
        bestCost  = inf;

        for k = candidates
            cost = norm(srcXY - NodesXY(k,:)) + norm(NodesXY(k,:) - dstXY);
            if cost < bestCost
                bestCost  = cost;
                bestRelay = k;
            end
        end

        RelayIndex(src,dst) = bestRelay;

        relayXY = NodesXY(bestRelay,:);
        relayLat = nodeLat(bestRelay);
        relayLon = nodeLon(bestRelay);

        %% ===== TWO HOPS =====
        % Hop-1: src -> relay
        % Hop-2: relay -> dst
        % End-to-end SINR = min(SINR1, SINR2)

        %% ===== VISUAL FLIGHT: UAV MOVES IN TWO SEGMENTS (JUST TO SHOW ROUTING) =====
        posXY = srcXY;
        pathXY = posXY;

        % Segment 1: src -> relay
        while norm(relayXY - posXY) > stepSize
            dir = (relayXY - posXY) / norm(relayXY - posXY);
            posXY = posXY + stepSize * dir;
            pathXY = [pathXY; posXY];

            d1 = norm(posXY - srcXY);       % src -> UAV pos
            d2 = norm(relayXY - posXY);     % UAV pos -> relay

            PL = 10 * pathLossExp * log10(max(d2,1));
            SINR_seg = Pt_dBm - PL - noise_dBm;

            latPath = zeros(size(pathXY,1),1);
            lonPath = zeros(size(pathXY,1),1);
            for t = 1:size(pathXY,1)
                [latPath(t), lonPath(t)] = meters2latlon(pathXY(t,1), pathXY(t,2));
            end

            cla(gx);
            geobasemap(gx,"satellite");
            hold(gx,'on');

            geoscatter(gx, lat, lon, 45, 'y', 'filled');
            geoscatter(gx, airportLat, airportLon, 110, 'r', '^', 'filled');

            for i = 1:N
                text(gx, lat(i)+0.0015, lon(i)+0.0015, "H"+string(i), ...
                    'Color','w','FontWeight','bold','FontSize',9);
            end
            text(gx, airportLat+0.0015, airportLon+0.0015, "A1", ...
                'Color','w','FontWeight','bold','FontSize',10);

            geoplot(gx, latPath, lonPath, 'c-', 'LineWidth', 2);

            [uLat,uLon] = meters2latlon(posXY(1),posXY(2));
            geoscatter(gx, uLat, uLon, 70, 'm', '^', 'filled');

            geoscatter(gx, nodeLat(src), nodeLon(src), 100, 'g', 'filled');
            geoscatter(gx, relayLat, relayLon, 110, 'w', 'filled');
            geoscatter(gx, nodeLat(dst), nodeLon(dst), 100, 'c', 'filled');

            title(gx, sprintf("Two-Hop Ground | %s → H%d → %s | Hop1 moving | SINR~%.1f dB", ...
                nodeNames(src), bestRelay, nodeNames(dst), SINR_seg));

            drawnow;
            writeVideo(videoObj, getframe(fig));
        end

        % Segment 2: relay -> dst
        posXY = relayXY;
        pathXY = posXY;

        while norm(dstXY - posXY) > stepSize
            dir = (dstXY - posXY) / norm(dstXY - posXY);
            posXY = posXY + stepSize * dir;
            pathXY = [pathXY; posXY];

            d = norm(dstXY - posXY);
            PL = 10 * pathLossExp * log10(max(d,1));
            SINR_seg = Pt_dBm - PL - noise_dBm;

            latPath = zeros(size(pathXY,1),1);
            lonPath = zeros(size(pathXY,1),1);
            for t = 1:size(pathXY,1)
                [latPath(t), lonPath(t)] = meters2latlon(pathXY(t,1), pathXY(t,2));
            end

            cla(gx);
            geobasemap(gx,"satellite");
            hold(gx,'on');

            geoscatter(gx, lat, lon, 45, 'y', 'filled');
            geoscatter(gx, airportLat, airportLon, 110, 'r', '^', 'filled');

            for i = 1:N
                text(gx, lat(i)+0.0015, lon(i)+0.0015, "H"+string(i), ...
                    'Color','w','FontWeight','bold','FontSize',9);
            end
            text(gx, airportLat+0.0015, airportLon+0.0015, "A1", ...
                'Color','w','FontWeight','bold','FontSize',10);

            geoplot(gx, latPath, lonPath, 'c-', 'LineWidth', 2);

            [uLat,uLon] = meters2latlon(posXY(1),posXY(2));
            geoscatter(gx, uLat, uLon, 70, 'm', '^', 'filled');

            geoscatter(gx, nodeLat(src), nodeLon(src), 100, 'g', 'filled');
            geoscatter(gx, relayLat, relayLon, 110, 'w', 'filled');
            geoscatter(gx, nodeLat(dst), nodeLon(dst), 100, 'c', 'filled');

            title(gx, sprintf("Two-Hop Ground | %s → H%d → %s | Hop2 moving | SINR~%.1f dB", ...
                nodeNames(src), bestRelay, nodeNames(dst), SINR_seg));

            drawnow;
            writeVideo(videoObj, getframe(fig));
        end

        %% ===== FINAL METRICS (E2E TWO-HOP) =====
        d1 = norm(relayXY - srcXY);
        d2 = norm(dstXY - relayXY);

        SINR1 = Pt_dBm - 10*pathLossExp*log10(max(d1,1)) - noise_dBm;
        SINR2 = Pt_dBm - 10*pathLossExp*log10(max(d2,1)) - noise_dBm;

        SINR_e2e = min(SINR1, SINR2);

        DistanceMatrix_km(src,dst) = (d1 + d2) / 1000;
        SINR_Matrix_dB(src,dst)    = SINR_e2e;
    end
end

close(videoObj);

%% ================= SAVE RESULTS =================
writetable(array2table(DistanceMatrix_km, ...
    'VariableNames', nodeNames, ...
    'RowNames', nodeNames), ...
    fullfile(outDir,"pairwise_path_distance_matrix_km.csv"), ...
    'WriteRowNames',true);

writetable(array2table(SINR_Matrix_dB, ...
    'VariableNames', nodeNames, ...
    'RowNames', nodeNames), ...
    fullfile(outDir,"pairwise_E2E_SINR_matrix_dB.csv"), ...
    'WriteRowNames',true);

writetable(array2table(RelayIndex, ...
    'VariableNames', nodeNames, ...
    'RowNames', nodeNames), ...
    fullfile(outDir,"chosen_relay_index_matrix.csv"), ...
    'WriteRowNames',true);

save(fullfile(outDir,"twohop_ground_with_airport_summary.mat"), ...
     "DistanceMatrix_km","SINR_Matrix_dB","RelayIndex","nodeNames", ...
     "Pt_dBm","noise_dBm","pathLossExp","uavSpeed","dt");

disp("✅ Two-Hop Ground Baseline (WITH AIRPORT) Completed");
disp("🎥 MP4 saved: twohop_ground_with_airport_fastforward.mp4");
disp("📁 Folder: twohop_ground_baseline_results/");