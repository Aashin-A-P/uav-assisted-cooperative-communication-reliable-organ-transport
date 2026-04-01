function relayIdx = point_to_nearest_relay(ENV, lat, lon)

R = ENV.Relays;
d = zeros(height(R),1);

for k = 1:height(R)
    d(k) = haversine_km(lat, lon, R.Latitude(k), R.Longitude(k));
end

[~, relayIdx] = min(d);

end