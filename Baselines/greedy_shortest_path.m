clc; clear; close all;

%% ================= LOAD DATA =================
[names, lat, lon, airportLat, airportLon, HospitalsXY] = hospital_data();

% Append airport as last node
names = [names; "Airport"];
lat   = [lat; airportLat];
lon   = [lon; airportLon];

%% ================= GEO → XY FOR AIRPORT =================
R = 6371000;
lat0 = mean(lat(1:end-1)) * pi/180;
lon0 = mean(lon(1:end-1)) * pi/180;

xA = R * ((airportLon*pi/180) - lon0) * cos(lat0);
yA = R * ((airportLat*pi/180) - lat0);

HospitalsXY = [HospitalsXY; [xA yA]];
N = size(HospitalsXY,1);

%% ================= UAV PARAMETERS =================
uavSpeed = 20;      % m/s  (~72 km/h realistic for medical UAV)
dt = 1;             % time step (s)
stepSize = uavSpeed * dt;

%% ================= OUTPUT FOLDER =================
outDir = "greedy_baseline_results";
if ~exist(outDir,"dir")
    mkdir(outDir);
end

%% ================= STORAGE =================
DistanceMatrix = zeros(N,N);
TimeMatrix     = zeros(N,N);
results = [];

%% ================= GEO FIGURE =================
fig = figure('Color','w','Position',[100 100 1300 720]);
gx = geoaxes(fig);
geobasemap(gx,"satellite");
hold(gx,'on');

%% ================= VIDEO WRITER =================
videoObj = VideoWriter(fullfile(outDir,"greedy_baseline_with_airport.mp4"),"MPEG-4");
videoObj.FrameRate = 10;
open(videoObj);

%% ================= METERS → LAT/LON =================
meters2latlon = @(x,y) deal( ...
    (y/R + lat0) * 180/pi, ...
    (x./(R*cos(lat0)) + lon0) * 180/pi );

%% ================= MAIN LOOP (ALL PAIRS) =================
for src = 1:N
    for dst = 1:N
        if src == dst
            continue;
        end

        pos = HospitalsXY(src,:);
        target = HospitalsXY(dst,:);
        pathXY = pos;
        time = 0;

        %% ===== GREEDY NAVIGATION =====
        while norm(target - pos) > stepSize
            dir = (target - pos) / norm(target - pos);
            pos = pos + stepSize * dir;
            pathXY = [pathXY; pos];
            time = time + dt;
        end

        pathXY = [pathXY; target];
        time = time + norm(target - pos)/uavSpeed;

        %% ===== METRICS =====
        totalDist = sum(vecnorm(diff(pathXY),2,2)); % meters
        DistanceMatrix(src,dst) = totalDist/1000;
        TimeMatrix(src,dst)     = time/60;

        results = [results;
            src dst DistanceMatrix(src,dst) TimeMatrix(src,dst)];

        %% ===== PATH → LAT/LON =====
        latPath = zeros(size(pathXY,1),1);
        lonPath = zeros(size(pathXY,1),1);
        for k = 1:size(pathXY,1)
            [latPath(k), lonPath(k)] = meters2latlon(pathXY(k,1), pathXY(k,2));
        end

        %% ===== LABELS (NO TERNARY – MATLAB SAFE) =====
        if src == N
            srcLabel = "A";
        else
            srcLabel = "H" + string(src);
        end

        if dst == N
            dstLabel = "A";
        else
            dstLabel = "H" + string(dst);
        end

        %% ===== ANIMATION =====
        cla(gx);
        geobasemap(gx,"satellite");

        % Hospitals
        geoscatter(gx, lat(1:end-1), lon(1:end-1), 45, 'y', 'filled');

        % Airport
        geoscatter(gx, lat(end), lon(end), 120, 'b', 'p', 'filled');

        % UAV path
        geoplot(gx, latPath, lonPath, 'r-', 'LineWidth', 2);

        % Source / Destination
        geoscatter(gx, lat(src), lon(src), 90, 'g', 'filled');
        geoscatter(gx, lat(dst), lon(dst), 90, 'r', 'filled');

        title(gx, sprintf("Greedy Baseline | %s → %s | %.2f km | %.1f min", ...
            srcLabel, dstLabel, DistanceMatrix(src,dst), TimeMatrix(src,dst)));

        drawnow;
        writeVideo(videoObj, getframe(fig));
    end
end

close(videoObj);

%% ================= SAVE RESULTS =================
resultsTable = array2table(results, ...
    'VariableNames',{'Source','Destination','Distance_km','Time_min'});

writetable(resultsTable, fullfile(outDir,"greedy_baseline_results.csv"));
writetable(array2table(DistanceMatrix), fullfile(outDir,"pairwise_distance_matrix.csv"));
writetable(array2table(TimeMatrix), fullfile(outDir,"pairwise_time_matrix.csv"));

save(fullfile(outDir,"summary.mat"), ...
     "resultsTable","DistanceMatrix","TimeMatrix","names");

disp("✅ Greedy baseline completed");
disp("📁 Folder created: greedy_baseline_results/");
disp("🎥 MP4 + CSV + MAT files saved successfully");