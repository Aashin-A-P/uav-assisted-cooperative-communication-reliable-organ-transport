function ENV = build_live_environment()

%% ================== PATH SETUP ==================
scriptDir = fileparts(mfilename('fullpath'));
pointsFile = fullfile(scriptDir, "weather_points_60.csv");
apiKeyFile = fullfile(scriptDir, "openweather_api_key.txt");

if ~isfile(pointsFile)
    error("weather_points_60.csv not found.");
end

if ~isfile(apiKeyFile)
    error("openweather_api_key.txt not found.");
end

apiKey = strtrim(string(fileread(apiKeyFile)));
if strlength(apiKey) == 0
    error("API key file is empty.");
end

Tp = readtable(pointsFile);

%% ================== API CONFIG ==================
baseUrl = "https://api.openweathermap.org/data/2.5/weather";
units = "metric";
opts = weboptions("Timeout",20);

lat = Tp.Latitude;
lon = Tp.Longitude;
n = height(Tp);

tempC = nan(n,1);
press = nan(n,1);
humid = nan(n,1);
wspd  = nan(n,1);
wdir  = nan(n,1);

fprintf("\n=========================================\n");
fprintf("Starting OpenWeather API Collection\n");
fprintf("Total Points: %d\n", n);
fprintf("=========================================\n\n");

%% ================== WEATHER FETCH LOOP ==================
for i = 1:n
    
    fprintf("Collecting data from point %d/%d (%.1f%%)\n", ...
        i, n, (i/n)*100);
    
    try
        url = sprintf("%s?lat=%.8f&lon=%.8f&appid=%s&units=%s", ...
            baseUrl, lat(i), lon(i), char(apiKey), char(units));

        data = webread(url, opts);

        tempC(i) = data.main.temp;
        press(i) = data.main.pressure;
        humid(i) = data.main.humidity;
        wspd(i)  = data.wind.speed;

        if isfield(data.wind,"deg")
            wdir(i) = data.wind.deg;
        else
            wdir(i) = 0;
        end
        
        fprintf("   ✓ Success | Temp: %.1f C | Wind: %.1f m/s\n\n", ...
            tempC(i), wspd(i));

    catch ME
        fprintf("   ✗ Failed | %s\n\n", ME.message);
    end

    pause(1.05); % Respect free-tier API rate limit

end

fprintf("Weather collection completed.\n\n");

%% ================== FILTER VALID DATA ==================
valid = ~isnan(tempC);

if nnz(valid) < 6
    error("Not enough valid weather points collected.");
end

lat   = lat(valid);
lon   = lon(valid);
tempC = tempC(valid);
press = press(valid);
humid = humid(valid);
wspd  = wspd(valid);
wdir  = wdir(valid);

%% ================== WIND VECTOR CONVERSION ==================
theta = deg2rad(wdir);
u = wspd .* cos(theta);
v = wspd .* sin(theta);

%% ================== BUILD INTERPOLANTS ==================
ENV.Ftemp  = scatteredInterpolant(lon, lat, tempC, "natural", "nearest");
ENV.Fpress = scatteredInterpolant(lon, lat, press, "natural", "nearest");
ENV.Fhumid = scatteredInterpolant(lon, lat, humid, "natural", "nearest");
ENV.Fu     = scatteredInterpolant(lon, lat, u, "natural", "nearest");
ENV.Fv     = scatteredInterpolant(lon, lat, v, "natural", "nearest");

%% ================== LOAD STATIC DATA ==================
ENV.Hospitals = readtable(fullfile(scriptDir,"hospital.csv"));
ENV.Relays    = readtable(fullfile(scriptDir,"relay.csv"));

ENV.TimestampUTC = datetime("now","TimeZone","UTC");

fprintf("Environment successfully built at %s UTC\n", ...
    string(ENV.TimestampUTC));
fprintf("=========================================\n\n");

end