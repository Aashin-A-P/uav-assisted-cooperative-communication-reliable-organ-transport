function Smooth = smooth_path(Path)

t = 1:length(Path.lat);
tt = linspace(1,length(Path.lat), length(Path.lat)*5);

Smooth.lat = spline(t, Path.lat, tt);
Smooth.lon = spline(t, Path.lon, tt);

end