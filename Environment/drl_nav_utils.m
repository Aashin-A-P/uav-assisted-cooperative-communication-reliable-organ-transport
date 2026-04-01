function d_km = drl_nav_haversine_km(lat1, lon1, lat2, lon2)

R = 6371; % Earth radius in km

phi1 = deg2rad(lat1);
phi2 = deg2rad(lat2);

dphi = deg2rad(lat2 - lat1);
dlambda = deg2rad(lon2 - lon1);

a = sin(dphi/2).^2 + cos(phi1).*cos(phi2).*sin(dlambda/2).^2;
c = 2*atan2(sqrt(a), sqrt(1-a));

d_km = R * c;

end