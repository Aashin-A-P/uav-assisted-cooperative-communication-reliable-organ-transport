function Path = compute_cost_aware_path(CostMap, ENV, srcHospital, dstHospital)

Cost = CostMap.Cost;
latv = CostMap.latv;
lonv = CostMap.lonv;

n = size(Cost,1);

% Convert lat/lon to grid index
[src_i, src_j] = latlon_to_index(latv, lonv, ...
    ENV.Hospitals.Latitude(srcHospital), ...
    ENV.Hospitals.Longitude(srcHospital));

[dst_i, dst_j] = latlon_to_index(latv, lonv, ...
    ENV.Hospitals.Latitude(dstHospital), ...
    ENV.Hospitals.Longitude(dstHospital));

% A* search
openSet = false(n,n);
closedSet = false(n,n);

gScore = inf(n,n);
fScore = inf(n,n);

parent = zeros(n,n,2);

gScore(src_i,src_j) = 0;
fScore(src_i,src_j) = heuristic(src_i,src_j,dst_i,dst_j);

openSet(src_i,src_j) = true;

while any(openSet(:))

    % Find node with lowest fScore
    tmp = fScore;
    tmp(~openSet) = inf;
    [~, idx] = min(tmp(:));
    [i,j] = ind2sub(size(tmp), idx);

    if i==dst_i && j==dst_j
        break;
    end

    openSet(i,j) = false;
    closedSet(i,j) = true;

    % 8-direction neighbors
    for di=-1:1
        for dj=-1:1
            if di==0 && dj==0
                continue;
            end
            ni=i+di; nj=j+dj;

            if ni<1||nj<1||ni>n||nj>n||closedSet(ni,nj)
                continue;
            end

            tentative = gScore(i,j) + Cost(ni,nj);

            if tentative < gScore(ni,nj)
                parent(ni,nj,:) = [i j];
                gScore(ni,nj) = tentative;
                fScore(ni,nj) = tentative + heuristic(ni,nj,dst_i,dst_j);
                openSet(ni,nj) = true;
            end
        end
    end
end

% Reconstruct path
i=dst_i; j=dst_j;
path_i=[]; path_j=[];
while ~(i==src_i && j==src_j)
    path_i=[i path_i];
    path_j=[j path_j];
    p=parent(i,j,:);
    i=p(1); j=p(2);
end
path_i=[src_i path_i];
path_j=[src_j path_j];

Path.lat = latv(path_i);
Path.lon = lonv(path_j);

end


function h = heuristic(i,j,di,dj)
h = sqrt((i-di)^2 + (j-dj)^2);
end


function [ii,jj] = latlon_to_index(latv, lonv, lat, lon)
[~, ii] = min(abs(latv - lat));
[~, jj] = min(abs(lonv - lon));
end