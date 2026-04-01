clc; clear; close all;

%% ================== INPUT/OUTPUT ==================
scriptDir = fileparts(mfilename('fullpath'));
inCsv = fullfile(scriptDir, "weather_openweather_latest.csv");
outDir = fullfile(scriptDir, "weather_interpolated_results");
if ~exist(outDir, "dir"), mkdir(outDir); end

if ~isfile(inCsv)
    error("Missing file: %s", inCsv);
end

%% ================== LOAD DATA ==================
T = readtable(inCsv, "TextType", "string");
requiredCols = ["Latitude","Longitude","TempC","PressurehPa","HumidityPct","WindSpeedMS","WindDirDeg","Status"];
if ~all(ismember(requiredCols, string(T.Properties.VariableNames)))
    error("Input CSV must contain: Latitude, Longitude, TempC, PressurehPa, HumidityPct, WindSpeedMS, WindDirDeg, Status");
end

ok = strcmpi(string(T.Status), "OK");
if nnz(ok) < 6
    error("Not enough valid weather points (Status=OK). Found: %d", nnz(ok));
end

lat = T.Latitude(ok);
lon = T.Longitude(ok);
tempC = T.TempC(ok);
press = T.PressurehPa(ok);
humid = T.HumidityPct(ok);
wspd = T.WindSpeedMS(ok);
wdir = T.WindDirDeg(ok);

%% ================== GRID ==================
nGrid = 120;
latv = linspace(min(lat), max(lat), nGrid);
lonv = linspace(min(lon), max(lon), nGrid);
[LON, LAT] = meshgrid(lonv, latv);

%% ================== SCALAR INTERPOLATION ==================
Ftemp = scatteredInterpolant(lon, lat, tempC, "natural", "nearest");
Fprs  = scatteredInterpolant(lon, lat, press, "natural", "nearest");
Fhum  = scatteredInterpolant(lon, lat, humid, "natural", "nearest");

TempGrid = Ftemp(LON, LAT);
PressGrid = Fprs(LON, LAT);
HumGrid = Fhum(LON, LAT);

%% ================== WIND INTERPOLATION (U,V) ==================
theta = deg2rad(wdir);
u = wspd .* cos(theta);
v = wspd .* sin(theta);

Fu = scatteredInterpolant(lon, lat, u, "natural", "nearest");
Fv = scatteredInterpolant(lon, lat, v, "natural", "nearest");

Ugrid = Fu(LON, LAT);
Vgrid = Fv(LON, LAT);

WindSpeedGrid = hypot(Ugrid, Vgrid);
WindDirGridDeg = mod(rad2deg(atan2(Vgrid, Ugrid)), 360);

%% ================== SAVE GRID CSV ==================
gridTable = table( ...
    LAT(:), LON(:), TempGrid(:), PressGrid(:), HumGrid(:), WindSpeedGrid(:), WindDirGridDeg(:), Ugrid(:), Vgrid(:), ...
    'VariableNames', {'Latitude','Longitude','TempC','PressurehPa','HumidityPct','WindSpeedMS','WindDirDeg','U','V'});

outGridCsv = fullfile(outDir, "weather_interpolated_grid.csv");
writetable(gridTable, outGridCsv);

%% ================== PLOTS ==================
% Temperature
fig1 = figure("Color","w","Position",[100 80 1000 760]);
contourf(LON, LAT, TempGrid, 20, "LineColor", "none");
hold on; scatter(lon, lat, 18, "k", "filled");
colorbar; xlabel("Longitude"); ylabel("Latitude");
title("Interpolated Temperature (C)");
axis tight; grid on;
saveas(fig1, fullfile(outDir, "temp_interpolated.png"));

% Humidity
fig2 = figure("Color","w","Position",[130 100 1000 760]);
contourf(LON, LAT, HumGrid, 20, "LineColor", "none");
hold on; scatter(lon, lat, 18, "k", "filled");
colorbar; xlabel("Longitude"); ylabel("Latitude");
title("Interpolated Humidity (%)");
axis tight; grid on;
saveas(fig2, fullfile(outDir, "humidity_interpolated.png"));

% Pressure
fig3 = figure("Color","w","Position",[160 120 1000 760]);
contourf(LON, LAT, PressGrid, 20, "LineColor", "none");
hold on; scatter(lon, lat, 18, "k", "filled");
colorbar; xlabel("Longitude"); ylabel("Latitude");
title("Interpolated Pressure (hPa)");
axis tight; grid on;
saveas(fig3, fullfile(outDir, "pressure_interpolated.png"));

% Wind speed + vectors
fig4 = figure("Color","w","Position",[190 140 1100 780]);
contourf(LON, LAT, WindSpeedGrid, 20, "LineColor", "none");
hold on;
skip = 6;
quiver(LON(1:skip:end,1:skip:end), LAT(1:skip:end,1:skip:end), ...
       Ugrid(1:skip:end,1:skip:end), Vgrid(1:skip:end,1:skip:end), ...
       1.2, "k");
scatter(lon, lat, 14, "w", "filled");
colorbar; xlabel("Longitude"); ylabel("Latitude");
title("Interpolated Wind Speed (m/s) and Direction");
axis tight; grid on;
saveas(fig4, fullfile(outDir, "wind_interpolated.png"));

fprintf("Saved interpolated grid CSV: %s\n", outGridCsv);
fprintf("Saved plots in: %s\n", outDir);
disp("Done: weather fields interpolated from weather_openweather_latest.csv");
