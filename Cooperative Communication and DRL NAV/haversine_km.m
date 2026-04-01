function d = haversine_km(lat1, lon1, lat2, lon2)

R = 6371;

phi1 = deg2rad(lat1);
phi2 = deg2rad(lat2);

dphi = deg2rad(lat2-lat1);
dl = deg2rad(lon2-lon1);

a = sin(dphi/2)^2 + cos(phi1)*cos(phi2)*sin(dl/2)^2;
c = 2*atan2(sqrt(a),sqrt(1-a));

d = R*c;

end