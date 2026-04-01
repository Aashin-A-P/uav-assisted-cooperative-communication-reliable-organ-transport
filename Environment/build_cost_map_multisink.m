function CostMap = build_cost_map_multisink(ENV, Net, P, srcHospital, dstHospital)

fprintf("\nBuilding communication cost map...\n");

% Airport coordinates (given by you)
airportLat = 12.9965918;
airportLon = 80.1708076;

% Gateways
srcG  = hospital_to_nearest_relay(ENV, srcHospital);
dstG  = hospital_to_nearest_relay(ENV, dstHospital);
ctrlG = point_to_nearest_relay(ENV, airportLat, airportLon);

% Bounding region (based on relays)
latMin = min(ENV.Relays.Latitude)  - 0.02;
latMax = max(ENV.Relays.Latitude)  + 0.02;
lonMin = min(ENV.Relays.Longitude) - 0.02;
lonMax = max(ENV.Relays.Longitude) + 0.02;

nGrid = 80;   % resolution (increase later)

latv = linspace(latMin, latMax, nGrid);
lonv = linspace(lonMin, lonMax, nGrid);

Cost = zeros(nGrid, nGrid);

% Weights (you can tune)
wDst  = 1.0;   % destination most important
wCtrl = 0.7;   % airport second
wSrc  = 0.3;   % source lowest

for i = 1:nGrid
    for j = 1:nGrid
        
        lat = latv(i);
        lon = lonv(j);

        % UAV -> relay link
        U = uav_to_relays(ENV, P, lat, lon);

        % Choose strongest SNR relay
        [~, r] = max(U.SNR_dB);

        % Backbone routing delays
        dDst  = distances(Net.G, r, dstG);
        dSrc  = distances(Net.G, r, srcG);
        dCtrl = distances(Net.G, r, ctrlG);

        if ~isfinite(dDst);  dDst  = 10; end
        if ~isfinite(dSrc);  dSrc  = 10; end
        if ~isfinite(dCtrl); dCtrl = 10; end

        % Combined cost
        Cost(i,j) = U.Delay_s(r) ...
          + wDst*dDst ...
          + wCtrl*dCtrl ...
          + wSrc*dSrc ...
          + 0.2*(1 - U.Psucc(r));

    end
end

CostMap.Cost = Cost;
CostMap.latv = latv;
CostMap.lonv = lonv;

fprintf("Cost map built.\n");

end