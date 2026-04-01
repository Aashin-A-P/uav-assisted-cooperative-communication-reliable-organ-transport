function CostMap = build_multi_sink_cost_map(ENV, Net, P, ...
                                             srcHospital, dstHospital, ...
                                             airportLat, airportLon)

latMin = min(ENV.Relays.Latitude)  - 0.02;
latMax = max(ENV.Relays.Latitude)  + 0.02;
lonMin = min(ENV.Relays.Longitude) - 0.02;
lonMax = max(ENV.Relays.Longitude) + 0.02;

N = 80;

latv = linspace(latMin, latMax, N);
lonv = linspace(lonMin, lonMax, N);

Cost = zeros(N,N);

% Hospitals
srcLat = ENV.Hospitals.Latitude(srcHospital);
srcLon = ENV.Hospitals.Longitude(srcHospital);

dstLat = ENV.Hospitals.Latitude(dstHospital);
dstLon = ENV.Hospitals.Longitude(dstHospital);

% Gateway relays
rSrc  = nearest_relay(ENV, srcLat, srcLon);
rDst  = nearest_relay(ENV, dstLat, dstLon);
rCtrl = nearest_relay(ENV, airportLat, airportLon);

dToSrc  = distances(Net.G, rSrc);
dToDst  = distances(Net.G, rDst);
dToCtrl = distances(Net.G, rCtrl);

maxDelay = 0;
maxDist  = 0;

% First pass normalization
for i = 1:N
    for j = 1:N
        
        lat = latv(i);
        lon = lonv(j);
        
        U = uav_to_relays(ENV, P, lat, lon);
        
        bestDelay = inf;
        
        for r = 1:length(U.Delay_s)
            if isinf(U.Delay_s(r)), continue; end
            
            totalDelay = U.Delay_s(r) + ...
                         min([dToSrc(r), dToDst(r), dToCtrl(r)]);
            
            bestDelay = min(bestDelay, totalDelay);
        end
        
        if ~isinf(bestDelay)
            maxDelay = max(maxDelay, bestDelay);
        end
        
        dGoal = haversine_km(lat, lon, dstLat, dstLon);
        maxDist = max(maxDist, dGoal);
    end
end

% Cost computation
coastLon = 80.23;

for i = 1:N
    for j = 1:N
        
        lat = latv(i);
        lon = lonv(j);
        
        U = uav_to_relays(ENV, P, lat, lon);
        
        bestDelay = inf;
        snrPenalty = 0;
        
        for r = 1:length(U.Delay_s)
            if isinf(U.Delay_s(r)), continue; end
            
            totalDelay = U.Delay_s(r) + ...
                         min([dToSrc(r), dToDst(r), dToCtrl(r)]);
            
            bestDelay = min(bestDelay, totalDelay);
            
            if U.SNR_dB(r) < 8
                snrPenalty = snrPenalty + 1;
            end
        end
        
        if isinf(bestDelay)
            delayNorm = 1;
        else
            delayNorm = bestDelay / maxDelay;
        end
        
        distNorm = haversine_km(lat, lon, dstLat, dstLon) / maxDist;
        
        % ---- Strong mission-priority weighting ----
        C = 0.65 * distNorm + 0.30 * delayNorm + 0.05 * snrPenalty;
        
        % ---- Sea penalty ----
        if lon > coastLon
            C = C + 5;
        end
        
        Cost(i,j) = C;
    end
end

CostMap.Cost = Cost;
CostMap.latv = latv;
CostMap.lonv = lonv;

end