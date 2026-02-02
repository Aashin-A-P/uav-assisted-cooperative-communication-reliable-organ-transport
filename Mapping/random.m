clc; clear; close all;

%% ================== CONFIG ==================
rng(42);                 % for reproducibility
areaSize = 50000;        % 50 km x 50 km area
numHospitals = 10;

%% ================== RANDOM NODE GENERATION ==================
% Hospitals (H1 to H10)
Hospitals = areaSize * rand(numHospitals, 2);

% Airport (A1)
Airport = areaSize * rand(1, 2);

%% ================== VISUALIZATION ==================
figure('Color','w','Position',[100 100 1000 700]);
hold on; grid on; axis equal;

title("Randomized Hospital Network (Abstract Simulation)");
xlabel("X (meters)");
ylabel("Y (meters)");

xlim([0 areaSize]);
ylim([0 areaSize]);

% Plot hospitals
plot(Hospitals(:,1), Hospitals(:,2), ...
     'ks','MarkerSize',9,'MarkerFaceColor','y');

% Plot airport
plot(Airport(1), Airport(2), ...
     'kp','MarkerSize',14,'MarkerFaceColor',[0.2 0.6 1]);

% Label hospitals
for i = 1:numHospitals
    text(Hospitals(i,1)+800, Hospitals(i,2)+800, ...
        sprintf("H%d", i), ...
        'FontWeight','bold', 'FontSize',10);
end

% Label airport
text(Airport(1)+800, Airport(2)+800, ...
     "A1", 'FontWeight','bold', 'Color',[0 0.3 0.7]);

legend("Hospitals","Airport","Location","northoutside");

%% ================== DISTANCE MATRIX ==================
disp(" ");
disp("===== PAIRWISE DISTANCE MATRIX (km) =====");

D = zeros(numHospitals, numHospitals);
for i = 1:numHospitals
    for j = 1:numHospitals
        D(i,j) = norm(Hospitals(i,:) - Hospitals(j,:)) / 1000;
    end
end

fprintf("%6s"," ");
for j = 1:numHospitals
    fprintf("%6s", sprintf("H%d",j));
end
fprintf("\n");

for i = 1:numHospitals
    fprintf("%6s", sprintf("H%d",i));
    for j = 1:numHospitals
        fprintf("%6.1f", D(i,j));
    end
    fprintf("\n");
end

%% ================== NEAREST NEIGHBOUR CHECK ==================
disp(" ");
disp("===== NEAREST HOSPITAL DISTANCE (km) =====");

for i = 1:numHospitals
    d = D(i,:);
    d(i) = inf;
    [minDist, idx] = min(d);

    fprintf("H%-2d → Nearest: H%-2d | Distance: %.2f km\n", ...
        i, idx, minDist);
end

%% ================== AIRPORT DISTANCES ==================
disp(" ");
disp("===== AIRPORT → HOSPITAL DISTANCES (km) =====");

for i = 1:numHospitals
    dA = norm(Hospitals(i,:) - Airport) / 1000;
    fprintf("A1 → H%-2d : %.2f km\n", i, dA);
end