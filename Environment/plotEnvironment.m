function plotEnvironment(ENV)

hosp = ENV.Hospitals;
rel  = ENV.Relays;

figure('Color','w','Position',[100 80 1400 750]);
gx = geoaxes;
geobasemap(gx,"satellite");
hold(gx,"on");

title(gx,"Hospital and Relay UAV Network");

% Plot Hospitals
geoscatter(gx, hosp.Latitude, hosp.Longitude, ...
    70, 'y', 'filled');

% Plot Relays
geoscatter(gx, rel.Latitude, rel.Longitude, ...
    40, 'b', 'filled');

% Label Hospitals
for i = 1:height(hosp)
    text(gx, hosp.Latitude(i)+0.0015, ...
        hosp.Longitude(i)+0.0015, ...
        "H"+string(i), ...
        'Color','w', ...
        'FontWeight','bold', ...
        'FontSize',9);
end

% Label Relays
for i = 1:height(rel)
    text(gx, rel.Latitude(i)+0.001, ...
        rel.Longitude(i)+0.001, ...
        "R"+string(rel.RelayID(i)), ...
        'Color','w', ...
        'FontWeight','bold', ...
        'FontSize',8);
end

legend(gx,"Hospitals","Relay UAVs","Location","northoutside");

fprintf("Environment map displayed successfully.\n\n");

end