clc; clear; close all;

%% ================== HOSPITAL SET ==================
H = {
    "MIOT Hospitals",                         13.021221305087122, 80.18399507329879
    "Government Royapettah Hospital",         13.055498069165232, 80.2648511955569
    "Rajiv Gandhi Govt General Hospital",     13.08153114180747,  80.2776430685756
    "Sri Ramachandra Medical Centre",         13.03925226152519,  80.14349035322947
    "Frontline Hospital & Research Institute",13.1022933105255,   80.19031625570929
    "Velammal Medical College Hospital",      13.079292822735503, 80.11440874156483
    "Dr Kamakshi Memorial Hospitals",         12.95184673893021,  80.2094058632643
    "Gleneagles Health City",                 12.89805795018799,  80.20624846857314
    "Chettinad Hospital & Research Institute",12.796856116913377, 80.218488188645
    "Sree Balaji Medical College & Hospital", 12.95541658997134,  80.13781040081767
    "SRM Medical College (Kattankulathur)",   12.82104353039475,  80.04814100904443
    "Hindu Mission Hospital",                 12.923851514454665, 80.11410716671865
    "Saveetha Medical College & Hospital",    13.026403198082061, 80.01397525156452
    "Tagore Medical College & Hospital",      12.860384521955428, 80.13614546857262
    "Karpagam Hospital",                      13.200639912595816, 79.89083476390348
};

N = size(H,1);
names = string(H(:,1));
lat = cell2mat(H(:,2));
lon = cell2mat(H(:,3));

%% ================== AIRPORT ==================
airportName = "Chennai International Airport";
airportLat = 12.99659175499222;
airportLon = 80.1708075900248;

%% ================== MAPPING TOOLBOX VISUALIZATION ==================
figure('Color','w','Position',[80 80 1300 720]);

gx = geoaxes;
geobasemap(gx,"satellite");   % try: "streets", "topographic"
hold on;

title(gx,"Hospital Network for UAV Evaluation – Chennai");

% Plot hospitals
geoscatter(gx, lat, lon, 50, 'y', 'filled');

% Plot airport
geoscatter(gx, airportLat, airportLon, ...
           120, 'b', 'p', 'filled');

% Label hospitals
for i = 1:N
    text(gx, lat(i)+0.002, lon(i)+0.002, ...
        sprintf("H%d",i), ...
        'Color','w', ...
        'FontWeight','bold', ...
        'FontSize',9);
end

text(gx, airportLat+0.002, airportLon+0.002, ...
    "Airport", ...
    'Color','c', ...
    'FontWeight','bold');

legend(gx, ...
    "Hospitals","Airport", ...
    'Location',"northoutside");

%% ================== LAT/LON → LOCAL METERS (FOR DISTANCES) ==================
R = 6371000;
lat0 = mean(lat) * pi/180;
lon0 = mean(lon) * pi/180;

x = R * ((lon*pi/180) - lon0) .* cos(lat0);
y = R * ((lat*pi/180) - lat0);
HospitalsXY = [x y];

%% ================== DISTANCE MATRIX ==================
disp(" ");
disp("===== PAIRWISE DISTANCE MATRIX (km) =====");

D = zeros(N,N);
for i = 1:N
    for j = 1:N
        D(i,j) = norm(HospitalsXY(i,:) - HospitalsXY(j,:)) / 1000;
    end
end

fprintf("%6s"," ");
for j = 1:N
    fprintf("%6s",sprintf("H%d",j));
end
fprintf("\n");

for i = 1:N
    fprintf("%6s",sprintf("H%d",i));
    for j = 1:N
        fprintf("%6.1f",D(i,j));
    end
    fprintf("\n");
end

%% ================== NEAREST NEIGHBOUR ==================
disp(" ");
disp("===== NEAREST HOSPITAL DISTANCE (km) =====");

for i = 1:N
    d = D(i,:);
    d(i) = inf;
    [m,idx] = min(d);
    fprintf("H%-2d (%s) → Nearest: H%-2d | %.2f km\n", ...
        i, names(i), idx, m);
end

%% ================== LEGEND PRINT ==================
disp(" ");
disp("===== HOSPITAL LEGEND =====");
for i = 1:N
    fprintf("H%-2d : %s\n", i, names(i));
end
disp("A  : Chennai International Airport");
