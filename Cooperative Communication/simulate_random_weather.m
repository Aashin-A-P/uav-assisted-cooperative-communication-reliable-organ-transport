clc; clear; close all;

%% ================== CONFIG ==================
scriptDir = fileparts(mfilename('fullpath'));
pointsFile = fullfile(scriptDir, "weather_points_60.csv");

if ~isfile(pointsFile)
    error("Missing file: weather_points_60.csv (run map_weather_60_points.m first)");
end

%% ================== SELECT SCENARIO ==================
% Choose scenario:
% 1 = Normal
% 2 = High Wind
% 3 = Storm
% 4 = Heatwave
scenarioType = 2;  

rng('shuffle'); % different values each run

%% ================== LOAD POINTS ==================
Tp = readtable(pointsFile, "TextType", "string");

n = height(Tp);
ptId = Tp.PointID;
ptLat = Tp.Latitude;
ptLon = Tp.Longitude;

%% ================== GENERATE RANDOM WEATHER ==================

switch scenarioType

    case 1  % Normal
        tempC = 25 + 3*randn(n,1);
        humidityPct = 60 + 10*randn(n,1);
        pressurehPa = 1013 + 5*randn(n,1);
        windSpeedMS = 4 + 1.5*randn(n,1);

    case 2  % High Wind
        tempC = 24 + 2*randn(n,1);
        humidityPct = 55 + 8*randn(n,1);
        pressurehPa = 1008 + 4*randn(n,1);
        windSpeedMS = 12 + 3*randn(n,1);

    case 3  % Storm
        tempC = 22 + 2*randn(n,1);
        humidityPct = 85 + 5*randn(n,1);
        pressurehPa = 995 + 6*randn(n,1);
        windSpeedMS = 18 + 4*randn(n,1);

    case 4  % Heatwave
        tempC = 40 + 3*randn(n,1);
        humidityPct = 35 + 6*randn(n,1);
        pressurehPa = 1005 + 3*randn(n,1);
        windSpeedMS = 5 + 1*randn(n,1);

    otherwise
        error("Invalid scenarioType");
end

% Clamp realistic ranges
humidityPct = max(10, min(100, humidityPct));
pressurehPa = max(950, min(1050, pressurehPa));
windSpeedMS = max(0, windSpeedMS);

windDirDeg = rand(n,1)*360;
windGustMS = windSpeedMS + rand(n,1)*3;

statusCode = repmat("SIMULATED", n, 1);
timestampUTC = repmat(string(datetime("now","TimeZone","UTC")), n, 1);

%% ================== SAVE FILE ==================
Tout = table(ptId, ptLat, ptLon, tempC, pressurehPa, humidityPct, ...
    windSpeedMS, windDirDeg, windGustMS, statusCode, timestampUTC, ...
    'VariableNames', {'PointID','Latitude','Longitude','TempC','PressurehPa','HumidityPct','WindSpeedMS','WindDirDeg','WindGustMS','Status','TimestampUTC'});

outCsv = fullfile(scriptDir, "weather_simulated_random.csv");
writetable(Tout, outCsv);

fprintf("Saved simulated weather: %s\n", outCsv);

%% ================== PLOT MAP ==================
fig = figure("Color","w","Position",[80 80 1450 780]);
gx = geoaxes(fig);
geobasemap(gx,"satellite");
hold(gx,"on");

geoscatter(gx, ptLat, ptLon, 46, tempC, "filled", ...
    "MarkerEdgeColor","k");

cb = colorbar;
cb.Label.String = "Temperature (C)";
colormap(turbo);

title(gx, "Simulated Weather Scenario " + scenarioType);

disp("Done: Simulated random weather generated.");