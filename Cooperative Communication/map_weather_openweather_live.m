clc; clear; close all;

%% ================== CONFIG ==================
scriptDir = fileparts(mfilename('fullpath'));
pointsFile = fullfile(scriptDir, "weather_points_60.csv");
apiKeyFile = fullfile(scriptDir, "openweather_api_key.txt");

if ~isfile(apiKeyFile)
    error("Missing file: %s (put your OpenWeather API key in this file)", apiKeyFile);
end

apiKey = strtrim(string(fileread(apiKeyFile)));
if strlength(apiKey) == 0 || apiKey == "YOUR_OPENWEATHER_API_KEY_HERE"
    error("Invalid API key in %s. Replace placeholder with your real key.", apiKeyFile);
end

if ~isfile(pointsFile)
    error("Missing file: %s (run map_weather_60_points.m first)", pointsFile);
end

units = "metric";                           % "metric" => temp in C
baseUrl = "https://api.openweathermap.org/data/2.5/weather";
rateLimitPerMin = 60;                       % free-tier style limit
minDelaySec = 60/rateLimitPerMin + 0.05;    % keep slight buffer

%% ================== LOAD POINTS ==================
Tp = readtable(pointsFile, "TextType", "string");
requiredCols = ["PointID","Latitude","Longitude"];
if ~all(ismember(requiredCols, string(Tp.Properties.VariableNames)))
    error("weather_points_60.csv must contain: PointID, Latitude, Longitude");
end

n = height(Tp);
ptId = Tp.PointID(:);
ptLat = Tp.Latitude(:);
ptLon = Tp.Longitude(:);

%% ================== FETCH WEATHER ==================
tempC = nan(n,1);
pressurehPa = nan(n,1);
humidityPct = nan(n,1);
windSpeedMS = nan(n,1);
windDirDeg = nan(n,1);
windGustMS = nan(n,1);
statusCode = strings(n,1);

opts = weboptions("Timeout", 20);

fprintf("Fetching OpenWeather for %d points...\n", n);
for i = 1:n
    try
        url = sprintf("%s?lat=%.8f&lon=%.8f&appid=%s&units=%s", ...
            baseUrl, ptLat(i), ptLon(i), char(apiKey), char(units));
        data = webread(url, opts);

        tempC(i) = data.main.temp;
        pressurehPa(i) = data.main.pressure;
        humidityPct(i) = data.main.humidity;
        windSpeedMS(i) = data.wind.speed;

        if isfield(data.wind, "deg")
            windDirDeg(i) = data.wind.deg;
        end

        if isfield(data.wind, "gust")
            windGustMS(i) = data.wind.gust;
        end

        statusCode(i) = "OK";
        fprintf("Point %d/%d: OK (%.1f C)\n", i, n, tempC(i));

    catch ME
        statusCode(i) = "ERROR";
        fprintf("Point %d/%d: ERROR (%s)\n", i, n, ME.message);
    end

    if i < n
        pause(minDelaySec);
    end
end

%% ================== SAVE WEATHER SNAPSHOT ==================
timestampUTC = datetime("now","TimeZone","UTC","Format","yyyy-MM-dd HH:mm:ss");
timestampCol = repmat(string(timestampUTC), n, 1);

% Ensure all columns are n-by-1 (MATLAB version-safe)
ptId = ptId(:);
ptLat = ptLat(:);
ptLon = ptLon(:);
tempC = tempC(:);
pressurehPa = pressurehPa(:);
humidityPct = humidityPct(:);
windSpeedMS = windSpeedMS(:);
windDirDeg = windDirDeg(:);
windGustMS = windGustMS(:);
statusCode = statusCode(:);
timestampCol = timestampCol(:);

Tout = table( ...
    ptId, ptLat, ptLon, tempC, pressurehPa, humidityPct, windSpeedMS, windDirDeg, windGustMS, statusCode, ...
    timestampCol, ...
    'VariableNames', {'PointID','Latitude','Longitude','TempC','PressurehPa','HumidityPct','WindSpeedMS','WindDirDeg','WindGustMS','Status','TimestampUTC'});

outCsv = fullfile(scriptDir, "weather_openweather_latest.csv");
writetable(Tout, outCsv);
fprintf("Saved weather snapshot: %s\n", outCsv);

%% ================== MAP VISUALIZATION ==================
fig = figure("Color","w","Position",[80 80 1450 780]);
gx = geoaxes(fig);
geobasemap(gx, "satellite");
hold(gx, "on");

valid = ~isnan(tempC);

if any(valid)
    geoscatter(gx, ptLat(valid), ptLon(valid), 46, tempC(valid), "filled", ...
        "MarkerEdgeColor", "k", "LineWidth", 0.4);
    cb = colorbar;
    cb.Label.String = "Temperature (C)";
    colormap(turbo);
else
    geoscatter(gx, ptLat, ptLon, 35, [1 0.4 0.1], "filled");
end

% Mark failed points in black X
if any(~valid)
    geoscatter(gx, ptLat(~valid), ptLon(~valid), 32, "k", "x", "LineWidth", 1.2);
end

% Label each point with temp on top
for i = 1:n
    if valid(i)
        labelTxt = sprintf("P%d T%.1fC H%.0f%% P%.0f\nW%.1f D%.0f G%.1f", ...
            ptId(i), tempC(i), humidityPct(i), pressurehPa(i), ...
            windSpeedMS(i), windDirDeg(i), windGustMS(i));
    else
        labelTxt = sprintf("P%d ERR", ptId(i));
    end
    text(gx, ptLat(i)+0.0008, ptLon(i)+0.0008, labelTxt, ...
        "Color","w", "FontWeight","bold", "FontSize",8);
end

title(gx, "OpenWeather Live at 60 Points (" + string(timestampUTC) + " UTC)");

outPng = fullfile(scriptDir, "weather_openweather_live_map.png");
saveas(fig, outPng);
fprintf("Saved map: %s\n", outPng);

disp("Done: OpenWeather fetched, labeled, and plotted on 60 points.");
