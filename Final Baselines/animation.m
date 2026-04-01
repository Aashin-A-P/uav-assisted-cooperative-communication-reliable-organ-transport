clc; clear; close all;

%% ================== HOSPITAL SET ==================
H = {
    "MIOT Hospitals",13.021221305087122,80.18399507329879
    "Government Royapettah Hospital",13.055498069165232,80.2648511955569
    "Rajiv Gandhi Govt General Hospital",13.08153114180747,80.2776430685756
    "Sri Ramachandra Medical Centre",13.03925226152519,80.14349035322947
    "Frontline Hospital & Research Institute",13.1022933105255,80.19031625570929
    "Velammal Medical College Hospital",13.079292822735503,80.11440874156483
    "Dr Kamakshi Memorial Hospitals",12.95184673893021,80.2094058632643
    "Gleneagles Health City",12.89805795018799,80.20624846857314
    "Chettinad Hospital & Research Institute",12.796856116913377,80.218488188645
    "Sree Balaji Medical College & Hospital",12.95541658997134,80.13781040081767
    "SRM Medical College (Kattankulathur)",12.82104353039475,80.04814100904443
    "Hindu Mission Hospital",12.923851514454665,80.11410716671865
    "Saveetha Medical College & Hospital",13.026403198082061,80.01397525156452
    "Tagore Medical College & Hospital",12.860384521955428,80.13614546857262
    "Karpagam Hospital",13.200639912595816,79.89083476390348
};

lat = cell2mat(H(:,2));
lon = cell2mat(H(:,3));
N = numel(lat);

%% ================== AIRPORT ==================
airportLat = 12.99659175499222;
airportLon = 80.1708075900248;

nodeLat = [lat; airportLat];
nodeLon = [lon; airportLon];
numNodes = numel(nodeLat);

%% ================== SETTINGS ==================
fps = 25;
steps = 120;
uavSize = 60;
ROUTES_PER_SRC = 2;

if ~exist("videos","dir"), mkdir("videos"); end

baselineNames = {
    "B1_Direct"
    "B2_SingleUAV_DF"
    "B3_TwoHop_Ground"
    "B4_SingleUAV_AF"
};

rng(10);

%% ================== BASELINE LOOP ==================
for b = 1:4

    v = VideoWriter("videos/" + baselineNames{b} + ".mp4","MPEG-4");
    v.FrameRate = fps;
    open(v);

    fig = figure('Color','w','Position',[80 80 1300 720]);
    gx = geoaxes(fig);
    geobasemap(gx,"satellite");
    hold(gx,'on');

    geolimits(gx,[12.75 13.25],[79.9 80.4]);
    title(gx,"UAV Baseline – " + baselineNames{b});

    geoscatter(gx,lat,lon,60,'y','filled');
    geoscatter(gx,airportLat,airportLon,120,'b','p','filled');

    drawnow; pause(1.2);

    % route subset
    routes = [];
    for s = 1:numNodes
        d = setdiff(1:numNodes,s);
        pick = d(randperm(numel(d),min(ROUTES_PER_SRC,numel(d))));
        routes = [routes; [repmat(s,numel(pick),1) pick(:)]];
    end

    for r = 1:size(routes,1)

        src = routes(r,1);
        dst = routes(r,2);

        srcLat = nodeLat(src); srcLon = nodeLon(src);
        dstLat = nodeLat(dst); dstLon = nodeLon(dst);

        % delete ONLY previous handles
        if exist('hPath','var') && isvalid(hPath), delete(hPath); end
        if exist('hUAV','var')  && isvalid(hUAV),  delete(hUAV);  end

        switch b
            case {1,2,4}
                lats = linspace(srcLat,dstLat,steps);
                lons = linspace(srcLon,dstLon,steps);

            case 3
                idx = setdiff(1:N,[src dst]);
                d1 = hypot(nodeLat(idx)-srcLat,nodeLon(idx)-srcLon);
                d2 = hypot(nodeLat(idx)-dstLat,nodeLon(idx)-dstLon);
                [~,k] = min(d1+d2);
                mid = idx(k);

                lats = [linspace(srcLat,nodeLat(mid),steps/2), ...
                        linspace(nodeLat(mid),dstLat,steps/2)];
                lons = [linspace(srcLon,nodeLon(mid),steps/2), ...
                        linspace(nodeLon(mid),dstLon,steps/2)];
        end

        % path
        switch b
            case 1, hPath = geoplot(gx,lats,lons,'r','LineWidth',2);
            case 2, hPath = geoplot(gx,lats,lons,'g','LineWidth',2);
            case 3, hPath = geoplot(gx,lats,lons,'c','LineWidth',2);
            case 4, hPath = geoplot(gx,lats,lons,'m--','LineWidth',2);
        end

        % UAV only for B2 & B4
        if b==2 || b==4
            hUAV = geoscatter(gx,lats(1),lons(1), ...
                uavSize,'r','^','filled','MarkerEdgeColor','k');
        end

        for t = 1:steps
            if b==2
                hUAV.LatitudeData  = lats(t);
                hUAV.LongitudeData = lons(t);
            elseif b==4
                idx = max(1,round(1+(steps-1)*(t/steps)^1.6));
                hUAV.LatitudeData  = lats(idx);
                hUAV.LongitudeData = lons(idx);
            end
            drawnow limitrate;
            writeVideo(v,getframe(fig));
        end
    end

    close(v); close(fig);
    disp("✔ Saved " + baselineNames{b});
end

disp("✅ FINAL FIX: Triangle UAV visible and stable");