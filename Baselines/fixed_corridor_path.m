clc; clear; close all;

%% ================= LOAD DATA =================
% hospital_data() must return:
% names, lat, lon, airportLat, airportLon, HospitalsXY
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
uavSpeed = 20;     % m/s  (~72 km/h, payload-capable medical UAV)

%% ================= OUTPUT FOLDER =================
outDir = "fixed_waypoint_baseline_results";
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
videoObj = VideoWriter(fullfile(outDir,"fixed_waypoint_baseline.mp4"),"MPEG-4");
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

        srcPos = HospitalsXY(src,:);
        dstPos = HospitalsXY(dst,:);

        %% ===== FIXED CORRIDOR (NON-OPTIMAL WAYPOINT) =====
        midPoint = (srcPos + dstPos)/2;

        % Artificial detour (forces longer, fixed corridor path)
        offset = [3000, -3000];     % meters
        wp = midPoint + offset;

        waypoints = [srcPos; wp; dstPos];

        %% ===== PATH FOLLOWING =====
        pathXY = [];
        time = 0;

        for i = 1:size(waypoints,1)-1
            p1 = waypoints(i,:);
            p2 = waypoints(i+1,:);

            d = norm(p2 - p1);
            t = d / uavSpeed;

            steps = ceil(t);
            px = linspace(p1(1),p2(1),steps);
            py = linspace(p1(2),p2(2),steps);

            pathXY = [pathXY; [px' py']];
            time = time + t;
        end

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

        %% ===== LABELS =====
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

        % UAV Path
        geoplot(gx, latPath, lonPath, 'm--', 'LineWidth', 2);

        % Waypoint
        [wpLat, wpLon] = meters2latlon(wp(1), wp(2));
        geoscatter(gx, wpLat, wpLon, 90, 'c', 'filled');

        % Source / Destination
        geoscatter(gx, lat(src), lon(src), 90, 'g', 'filled');
        geoscatter(gx, lat(dst), lon(dst), 90, 'r', 'filled');

        title(gx, sprintf("Fixed Waypoint Baseline | %s → %s | %.2f km | %.1f min", ...
            srcLabel, dstLabel, DistanceMatrix(src,dst), TimeMatrix(src,dst)));

        drawnow;
        writeVideo(videoObj, getframe(fig));
    end
end

close(videoObj);

%% ================= SAVE RESULTS =================
resultsTable = array2table(results, ...
    'VariableNames',{'Source','Destination','Distance_km','Time_min'});

writetable(resultsTable, fullfile(outDir,"fixed_waypoint_results.csv"));
writetable(array2table(DistanceMatrix), fullfile(outDir,"pairwise_distance_matrix.csv"));
writetable(array2table(TimeMatrix), fullfile(outDir,"pairwise_time_matrix.csv"));

save(fullfile(outDir,"summary.mat"), ...
     "resultsTable","DistanceMatrix","TimeMatrix","names");

disp("✅ Fixed Waypoint baseline completed");
disp("📁 Folder created: fixed_waypoint_baseline_results/");
disp("🎥 MP4 + CSV + MAT files saved successfully");