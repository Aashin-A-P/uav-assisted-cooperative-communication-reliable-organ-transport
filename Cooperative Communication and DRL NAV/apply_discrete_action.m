function [lat2, lon2] = apply_discrete_action(lat, lon, a, step)

dLat = step/111320;
dLon = step/(111320*cosd(lat));

switch round(a)
    case 1, lat2 = lat+dLat; lon2 = lon;
    case 2, lat2 = lat-dLat; lon2 = lon;
    case 3, lat2 = lat; lon2 = lon+dLon;
    case 4, lat2 = lat; lon2 = lon-dLon;
    otherwise, lat2 = lat; lon2 = lon;
end

end