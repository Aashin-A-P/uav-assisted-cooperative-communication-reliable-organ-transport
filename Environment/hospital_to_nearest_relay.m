function relayIdx = hospital_to_nearest_relay(ENV, hospitalIdx)

H = ENV.Hospitals;
R = ENV.Relays;

hLat = H.Latitude(hospitalIdx);
hLon = H.Longitude(hospitalIdx);

d = zeros(height(R),1);

for k = 1:height(R)
    d(k) = haversine_km(hLat, hLon, ...
                        R.Latitude(k), R.Longitude(k));
end

[~, relayIdx] = min(d);

end