clc; clear; close all;

%% ================== FILE PATHS ==================
scriptDir = fileparts(mfilename('fullpath'));

hospitalFile = fullfile(scriptDir,"hospital.csv");
relayFile    = fullfile(scriptDir,"relay.csv");
weatherFile  = fullfile(scriptDir,"weather_interpolated_results","weather_interpolated_grid.csv");

if ~isfile(hospitalFile) || ~isfile(relayFile) || ~isfile(weatherFile)
    error("Missing one or more required input files.");
end

%% ================== LOAD DATA ==================
Th = readtable(hospitalFile);
Tr = readtable(relayFile);
Tw = readtable(weatherFile);

%% ================== TAG DATA TYPE ==================
Th.Type = repmat("Hospital",height(Th),1);
Tr.Type = repmat("Relay",height(Tr),1);
Tw.Type = repmat("WeatherGrid",height(Tw),1);

%% ================== STANDARDIZE COLUMN FORMAT ==================
% Ensure common columns exist

Th.U = nan(height(Th),1);
Th.V = nan(height(Th),1);
Th.TempC = nan(height(Th),1);
Th.PressurehPa = nan(height(Th),1);
Th.HumidityPct = nan(height(Th),1);
Th.WindSpeedMS = nan(height(Th),1);
Th.WindDirDeg = nan(height(Th),1);

Tr.U = nan(height(Tr),1);
Tr.V = nan(height(Tr),1);
Tr.TempC = nan(height(Tr),1);
Tr.PressurehPa = nan(height(Tr),1);
Tr.HumidityPct = nan(height(Tr),1);
Tr.WindSpeedMS = nan(height(Tr),1);
Tr.WindDirDeg = nan(height(Tr),1);

%% ================== COMBINE ALL ==================
Environment = [ ...
    Th(:,["Type","Latitude","Longitude","TempC","PressurehPa","HumidityPct","WindSpeedMS","WindDirDeg","U","V"]); ...
    Tr(:,["Type","Latitude","Longitude","TempC","PressurehPa","HumidityPct","WindSpeedMS","WindDirDeg","U","V"]); ...
    Tw(:,["Type","Latitude","Longitude","TempC","PressurehPa","HumidityPct","WindSpeedMS","WindDirDeg","U","V"])
];

%% ================== SAVE MASTER FILE ==================
outFile = fullfile(scriptDir,"environment_master.csv");
writetable(Environment,outFile);

fprintf("Environment master file created: %s\n", outFile);
disp("Hospitals + Relays + Weather grid merged successfully.");