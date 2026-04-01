function ENV = environment_engine_cached()

%% ================== CONFIG ==================
scriptDir = fileparts(mfilename('fullpath'));
cacheFile = fullfile(scriptDir, "weather_cache.mat");
cacheDurationMinutes = 15;
%% ================= COMMUNICATION PARAMETERS =================
P = struct();

P.BHz    = 10e6;    % Bandwidth (10 MHz)
P.NF_dB  = 5;       % Noise Figure
P.TxPower_dBm = 30; % Transmit Power
P.freqGHz = 2.4;    % Frequency (GHz)

%% ================== CHECK CACHE ==================
if isfile(cacheFile)
    load(cacheFile, "ENV", "cacheTime");

    minutesPassed = minutes(datetime("now") - cacheTime);

    if minutesPassed < cacheDurationMinutes
        fprintf("Using cached weather (%.1f minutes old)\n", minutesPassed);
        plotEnvironment(ENV);   % <-- Plot map
        return;
    else
        fprintf("Cache expired (%.1f minutes old). Refreshing weather...\n", minutesPassed);
    end
else
    fprintf("No cache found. Fetching weather...\n");
end

%% ================== FETCH NEW WEATHER ==================
ENV = build_live_environment();

%% ================== SAVE CACHE ==================
cacheTime = datetime("now");
save(cacheFile, "ENV", "cacheTime");

fprintf("Weather cache updated.\n");

%% ================== DISPLAY MAP ==================
plotEnvironment(ENV);

end