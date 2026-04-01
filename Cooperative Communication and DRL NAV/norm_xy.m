function [xN,yN] = norm_xy(lat,lon,cfg)

yN = (lat - cfg.latMin)/(cfg.latMax - cfg.latMin);
xN = (lon - cfg.lonMin)/(cfg.lonMax - cfg.lonMin);

yN = min(max(yN,0),1);
xN = min(max(xN,0),1);

end