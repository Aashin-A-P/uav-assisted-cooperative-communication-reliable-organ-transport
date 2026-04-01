clc; clear; close all;

%% ================== INPUT FILES ==================
scriptDir = fileparts(mfilename('fullpath'));
hospitalFile = fullfile(scriptDir, "hospital.csv");
relayFile = fullfile(scriptDir, "relay.csv");
liveWeatherFile = fullfile(scriptDir, "weather_openweather_latest.csv");

if ~isfile(hospitalFile), error("Missing file: %s", hospitalFile); end
if ~isfile(relayFile), error("Missing file: %s", relayFile); end

Th = readtable(hospitalFile, "TextType", "string");
Tr = readtable(relayFile, "TextType", "string");

%% ================== MAP BOUNDS FROM YOUR NETWORK ==================
hospLat = Th.Latitude(:);
hospLon = Th.Longitude(:);
relayLat = Tr.Latitude(:);
relayLon = Tr.Longitude(:);

R = 6371000;
lat0 = mean([hospLat; relayLat]) * pi/180;
lon0 = mean([hospLon; relayLon]) * pi/180;

latlon2m = @(la,lo) deal( ...
    R*((lo*pi/180)-lon0).*cos(lat0), ...
    R*((la*pi/180)-lat0));

m2latlon = @(x,y) deal( ...
    (y/R + lat0)*180/pi, ...
    (x./(R*cos(lat0)) + lon0)*180/pi);

[xH,yH] = latlon2m(hospLat, hospLon);
[xR,yR] = latlon2m(relayLat, relayLon);
allX = [xH; xR];
allY = [yH; yR];

pad_m = 5000;
xmin = min(allX) - pad_m;
xmax = max(allX) + pad_m;
ymin = min(allY) - pad_m;
ymax = max(allY) + pad_m;

%% ================== MODEL SETTINGS ==================
% Grid/time
Delta_g = 340;            % meters
Delta_t = 60;             % seconds
Nt = 120;                 % total time steps

% Temperature parameters
T0 = 30.0;                % deg C (fallback)
gx = -2e-5;               % degC per meter
gy = 1e-5;                % degC per meter
AT = 1.0;                 % deg C diurnal-like oscillation amplitude
PT = 24*3600;             % seconds
phiT = 0.0;               % rad
sigmaT = 0.7;             % innovation std
tauT = 30*60;             % seconds
ellT = 1500;              % spatial smoothing scale (m)

% Wind parameters
V0 = 6.0;                 % m/s (fallback)
AV = 1.5;                 % m/s
PV = 6*3600;              % seconds
phiV = 0.0;               % rad

Theta0_deg = 90;          % baseline direction in degrees
ATheta_deg = 25;          % direction oscillation amplitude in degrees
PTheta = 8*3600;          % seconds
phiTheta = 0.0;           % rad

sigmaU = 0.8;             % m/s
tauU = 20*60;             % seconds
ellU = 1800;              % m

sigmaV = 0.8;             % m/s
tauV = 20*60;             % seconds
ellV = 1800;              % m

% Optional calibration from latest fetched weather snapshot
if isfile(liveWeatherFile)
    Tw = readtable(liveWeatherFile, "TextType", "string");
    if all(ismember(["TempC","WindSpeedMS","Status"], string(Tw.Properties.VariableNames)))
        ok = strcmpi(string(Tw.Status), "OK");
        if any(ok)
            T0 = mean(Tw.TempC(ok), "omitnan");
            V0 = mean(Tw.WindSpeedMS(ok), "omitnan");
        end
    end
end

Theta0 = deg2rad(Theta0_deg);
ATheta = deg2rad(ATheta_deg);

%% ================== GRID BUILD ==================
x = xmin:Delta_g:xmax;
y = ymin:Delta_g:ymax;
[X,Y] = meshgrid(x,y);
[nY,nX] = size(X);
xc = (xmin + xmax)/2;
yc = (ymin + ymax)/2;

%% ================== PREALLOCATE OUTPUT FIELDS ==================
Tfield = zeros(nY,nX,Nt);
Vfield = zeros(nY,nX,Nt);
ThetaField = zeros(nY,nX,Nt);   % radians

% Anomaly states
deltaT_prev = zeros(nY,nX);
deltaU_prev = zeros(nY,nX);
deltaV_prev = zeros(nY,nX);

% Temporal AR coefficients
rhoT = exp(-Delta_t/tauT);
rhoU = exp(-Delta_t/tauU);
rhoV = exp(-Delta_t/tauV);

% Gaussian kernels in grid-cell units
sigCellT = max(ellT/Delta_g, 0.8);
sigCellU = max(ellU/Delta_g, 0.8);
sigCellV = max(ellV/Delta_g, 0.8);
kT = gaussian_kernel(sigCellT);
kU = gaussian_kernel(sigCellU);
kV = gaussian_kernel(sigCellV);

%% ================== TIME LOOP ==================
for t = 1:Nt
    tSec = t * Delta_t;

    % 1) Deterministic means
    muT = T0 + gx*(X-xc) + gy*(Y-yc) + AT*sin(2*pi*tSec/PT + phiT);
    muV = V0 + AV*sin(2*pi*tSec/PV + phiV);
    muTheta = Theta0 + ATheta*sin(2*pi*tSec/PTheta + phiTheta);
    muU = muV * cos(muTheta);
    muVcomp = muV * sin(muTheta);

    % 2) Spatially correlated noise
    ZT = randn(nY,nX);
    ZU = randn(nY,nX);
    ZV = randn(nY,nX);
    etaT = conv2(ZT, kT, "same");
    etaU = conv2(ZU, kU, "same");
    etaV = conv2(ZV, kV, "same");

    % Normalize innovations to unit variance
    etaT = etaT / max(std(etaT(:)), eps);
    etaU = etaU / max(std(etaU(:)), eps);
    etaV = etaV / max(std(etaV(:)), eps);

    % 3) Advection shift by mean wind
    shiftX = muU * Delta_t;       % meters
    shiftY = muVcomp * Delta_t;   % meters
    deltaT_adv = advect_field(deltaT_prev, x, y, shiftX, shiftY);
    deltaU_adv = advect_field(deltaU_prev, x, y, shiftX, shiftY);
    deltaV_adv = advect_field(deltaV_prev, x, y, shiftX, shiftY);

    % 4) AR(1) anomaly update
    deltaT = rhoT*deltaT_adv + sigmaT*sqrt(1-rhoT^2)*etaT;
    deltaU = rhoU*deltaU_adv + sigmaU*sqrt(1-rhoU^2)*etaU;
    deltaV = rhoV*deltaV_adv + sigmaV*sqrt(1-rhoV^2)*etaV;

    % 5) Construct final fields
    Tnow = muT + deltaT;
    Unow = muU + deltaU;
    VnowComp = muVcomp + deltaV;
    Vnow = hypot(Unow, VnowComp);
    ThetaNow = atan2(VnowComp, Unow);

    Tfield(:,:,t) = Tnow;
    Vfield(:,:,t) = Vnow;
    ThetaField(:,:,t) = ThetaNow;

    % carry state
    deltaT_prev = deltaT;
    deltaU_prev = deltaU;
    deltaV_prev = deltaV;
end

%% ================== SAVE OUTPUT ==================
[gridLat, gridLon] = m2latlon(X, Y);

outDir = fullfile(scriptDir, "weather_spatiotemporal_results");
if ~exist(outDir, "dir"), mkdir(outDir); end

save(fullfile(outDir, "weather_spatiotemporal_fields.mat"), ...
    "Tfield","Vfield","ThetaField","X","Y","gridLat","gridLon", ...
    "Delta_g","Delta_t","Nt","T0","V0");

% Save one snapshot table (last step) for direct reuse
tLast = Nt;
Tlast2D = Tfield(:,:,tLast);
Vlast2D = Vfield(:,:,tLast);
ThetaLastDeg2D = rad2deg(ThetaField(:,:,tLast));
snapTbl = table( ...
    gridLat(:), gridLon(:), Tlast2D(:), Vlast2D(:), ThetaLastDeg2D(:), ...
    'VariableNames', {'Latitude','Longitude','TempC','WindSpeedMS','WindDirDeg'});
writetable(snapTbl, fullfile(outDir, "weather_snapshot_laststep.csv"));

%% ================== QUICK VISUALIZATION ==================
% Temperature on map at last time step
fig1 = figure("Color","w","Position",[80 80 1450 780]);
gx = geoaxes(fig1);
geobasemap(gx, "satellite");
hold(gx, "on");

Tlast = Tfield(:,:,tLast);
geoscatter(gx, gridLat(:), gridLon(:), 18, Tlast(:), "filled");
geoscatter(gx, hospLat, hospLon, 55, "y", "filled");
geoscatter(gx, relayLat, relayLon, 30, "b", "filled");
cb = colorbar;
cb.Label.String = "Temperature (C)";
title(gx, sprintf("Spatio-Temporal Field (t=%d): Temperature", tLast));
legend(gx, "Grid Temp", "Hospitals", "Relays", "Location", "northoutside");
saveas(fig1, fullfile(outDir, "temperature_map_laststep.png"));

% Wind vectors in local XY at last step
fig2 = figure("Color","w","Position",[120 120 1200 780]);
contourf(X/1000, Y/1000, Vfield(:,:,tLast), 20, "LineColor", "none");
hold on;
skip = max(round(min(nX,nY)/25), 3);
Ulast = Vfield(:,:,tLast) .* cos(ThetaField(:,:,tLast));
Vlast = Vfield(:,:,tLast) .* sin(ThetaField(:,:,tLast));
quiver(X(1:skip:end,1:skip:end)/1000, Y(1:skip:end,1:skip:end)/1000, ...
       Ulast(1:skip:end,1:skip:end), Vlast(1:skip:end,1:skip:end), 1.2, "k");
scatter(xH/1000, yH/1000, 28, "y", "filled");
scatter(xR/1000, yR/1000, 20, "b", "filled");
colorbar; xlabel("X (km)"); ylabel("Y (km)");
title(sprintf("Spatio-Temporal Field (t=%d): Wind Speed + Direction", tLast));
axis equal tight; grid on;
saveas(fig2, fullfile(outDir, "wind_field_laststep.png"));

disp("Done: Spatio-temporal weather fields generated and saved.");

%% ================== LOCAL FUNCTIONS ==================
function k = gaussian_kernel(sigCell)
    r = max(1, ceil(3*sigCell));
    ax = -r:r;
    [Xk,Yk] = meshgrid(ax,ax);
    k = exp(-(Xk.^2 + Yk.^2)/(2*sigCell^2));
    k = k / sum(k(:));
end

function Fadv = advect_field(Fprev, x, y, shiftX, shiftY)
    % Semi-Lagrangian backtrace: value at (x,y) comes from (x-shiftX,y-shiftY)
    [Xg,Yg] = meshgrid(x,y);
    Xsrc = Xg - shiftX;
    Ysrc = Yg - shiftY;
    Fadv = interp2(Xg, Yg, Fprev, Xsrc, Ysrc, "linear", 0);
end
