function W = getWeatherAt(ENV, lat, lon)

W.TempC = ENV.Ftemp(lon, lat);
W.PressurehPa = ENV.Fpress(lon, lat);
W.HumidityPct = ENV.Fhumid(lon, lat);

U = ENV.Fu(lon, lat);
V = ENV.Fv(lon, lat);

W.WindSpeedMS = hypot(U,V);
W.WindDirDeg = mod(rad2deg(atan2(V,U)),360);
W.U = U;
W.V = V;

end