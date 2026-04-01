function idx = nearest_relay(ENV, lat, lon)

R = ENV.Relays;

minDist = inf;
idx = 1;

for i = 1:height(R)
    
    d = haversine_km(lat, lon, ...
                     R.Latitude(i), ...
                     R.Longitude(i));
    
    if d < minDist
        minDist = d;
        idx = i;
    end
end

end