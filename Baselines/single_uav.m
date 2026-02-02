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

M = size(NodesXY,1);   % hospitals + airport

%% ================= COMMUNICATION PARAMETERS =================
Pt_dBm = 20;
noise_dBm = -100;
pathLossExp = 2.5;

%% ================= UAV PARAMETERS =================
uavSpeed = 20;     % m/s
dt = 50;           % seconds
stepSize = uavSpeed * dt;

%% ================= OUTPUT FOLDER =================
outDir = "single_uav_df_baseline_results";
if ~exist(outDir,"dir"), mkdir(outDir); end

%% ================= STORAGE =================
DistanceMatrix_km = zeros(M,M);
SINR_Matrix_dB    = zeros(M,M);

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
    fullfile(outDir,"single_uav_df_with_airport_fastforward.mp4"), ...
    "MPEG-4");
videoObj.FrameRate = 15;
open(videoObj);

%% ================= MAIN LOOP =================
for src = 1:M
    for dst = 1:M

        if src == dst
            DistanceMatrix_km(src,dst) = 0;
            SINR_Matrix_dB(src,dst) = Inf;
            continue;
        end

        srcXY = NodesXY(src,:);
        dstXY = NodesXY(dst,:);
        posXY = srcXY;
        pathXY = posXY;

        %% ===== UAV FLIGHT (DF RELAY) =====
        while norm(dstXY - posXY) > stepSize

            dir = (dstXY - posXY) / norm(dstXY - posXY);
            posXY = posXY + stepSize * dir;
            pathXY = [pathXY; posXY];

            %% ===== TWO-HOP DISTANCES =====
            d1 = norm(posXY - srcXY);   % SRC → UAV
            d2 = norm(dstXY - posXY);   % UAV → DST

            %% ===== PATH LOSS =====
            PL1 = 10 * pathLossExp * log10(d1);
            PL2 = 10 * pathLossExp * log10(d2);

            %% ===== DF END-TO-END SINR =====
            SINR1 = Pt_dBm - PL1 - noise_dBm;
            SINR2 = Pt_dBm - PL2 - noise_dBm;
            SINR_DF = min(SINR1, SINR2);

            %% ===== PATH → LAT/LON =====
            latPath = zeros(size(pathXY,1),1);
            lonPath = zeros(size(pathXY,1),1);
            for k = 1:size(pathXY,1)
                [latPath(k), lonPath(k)] = ...
                    meters2latlon(pathXY(k,1), pathXY(k,2));
            end

            %% ===== VISUALIZATION =====
            cla(gx);
            geobasemap(gx,"satellite");
            hold(gx,'on');

            % Hospitals
            geoscatter(gx, lat, lon, 45, 'y', 'filled');

            % Airport
            geoscatter(gx, airportLat, airportLon, 110, 'r', '^', 'filled');

            % Labels
            for i = 1:N
                text(gx, lat(i)+0.0015, lon(i)+0.0015, "H"+string(i), ...
                    'Color','w','FontWeight','bold','FontSize',9);
            end
            text(gx, airportLat+0.0015, airportLon+0.0015, "A1", ...
                'Color','w','FontWeight','bold','FontSize',10);

            % UAV path
            geoplot(gx, latPath, lonPath, 'g-', 'LineWidth', 2);

            % UAV position (triangle)
            [uLat,uLon] = meters2latlon(posXY(1),posXY(2));
            geoscatter(gx, uLat, uLon, 70, 'm', '^', 'filled');

            % Source & Destination
            geoscatter(gx, nodeLat(src), nodeLon(src), 100, 'g', 'filled');
            geoscatter(gx, nodeLat(dst), nodeLon(dst), 100, 'c', 'filled');

            title(gx, sprintf( ...
                "Single UAV DF | %s → UAV → %s | SINR = %.1f dB", ...
                nodeNames(src), nodeNames(dst), SINR_DF));

            drawnow;
            writeVideo(videoObj, getframe(fig));
        end

        %% ===== FINAL METRICS =====
        d_total = norm(dstXY - srcXY);
        DistanceMatrix_km(src,dst) = d_total / 1000;

        midXY = (srcXY + dstXY)/2;
        d1 = norm(midXY - srcXY);
        d2 = norm(dstXY - midXY);

        SINR_Matrix_dB(src,dst) = ...
            min(Pt_dBm - 10*pathLossExp*log10(d1) - noise_dBm, ...
                Pt_dBm - 10*pathLossExp*log10(d2) - noise_dBm);
    end
end

close(videoObj);

%% ================= SAVE RESULTS =================
writetable(array2table(DistanceMatrix_km, ...
    'VariableNames', nodeNames, ...
    'RowNames', nodeNames), ...
    fullfile(outDir,"pairwise_distance_matrix_km.csv"), ...
    'WriteRowNames',true);

writetable(array2table(SINR_Matrix_dB, ...
    'VariableNames', nodeNames, ...
    'RowNames', nodeNames), ...
    fullfile(outDir,"pairwise_SINR_matrix_dB.csv"), ...
    'WriteRowNames',true);

save(fullfile(outDir,"single_uav_df_with_airport_summary.mat"), ...
     "DistanceMatrix_km","SINR_Matrix_dB","nodeNames", ...
     "Pt_dBm","noise_dBm","pathLossExp","uavSpeed","dt");

disp("✅ Single UAV DF Baseline (WITH AIRPORT) Completed");
disp("🎥 MP4 saved: single_uav_df_with_airport_fastforward.mp4");
disp("📁 Folder: single_uav_df_baseline_results/");