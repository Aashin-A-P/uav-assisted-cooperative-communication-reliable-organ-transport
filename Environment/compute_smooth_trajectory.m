function Path = compute_smooth_trajectory(CostMap, ENV, ...
                                          srcHospital, dstHospital)

Cost = CostMap.Cost;
latv = CostMap.latv;
lonv = CostMap.lonv;

[Gy, Gx] = gradient(Cost);

lat = ENV.Hospitals.Latitude(srcHospital);
lon = ENV.Hospitals.Longitude(srcHospital);

dstLat = ENV.Hospitals.Latitude(dstHospital);
dstLon = ENV.Hospitals.Longitude(dstHospital);

stepSize = 0.0012;
maxSteps = 1500;

pathLat = [];
pathLon = [];

for k = 1:maxSteps
    
    pathLat(end+1) = lat;
    pathLon(end+1) = lon;
    
    if haversine_km(lat,lon,dstLat,dstLon) < 0.1
        break;
    end
    
    [~, i] = min(abs(latv - lat));
    [~, j] = min(abs(lonv - lon));
    
    dLat = -Gy(i,j);
    dLon = -Gx(i,j);
    
    normVal = sqrt(dLat^2 + dLon^2);
    if normVal > 0
        dLat = dLat / normVal;
        dLon = dLon / normVal;
    end
    
    % Strong goal attraction
    dLat = dLat + 3*(dstLat - lat);
    dLon = dLon + 3*(dstLon - lon);
    
    lat = lat + stepSize * dLat;
    lon = lon + stepSize * dLon;
    
    % Boundary clamp
    lat = min(max(lat, min(latv)), max(latv));
    lon = min(max(lon, min(lonv)), max(lonv));
end

Path.lat = pathLat;
Path.lon = pathLon;

end