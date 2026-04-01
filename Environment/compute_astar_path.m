function Path = compute_astar_path(CostMap, ENV, srcHospital, dstHospital)

Cost = CostMap.Cost;
latv = CostMap.latv;
lonv = CostMap.lonv;

N = length(latv);

% Convert hospitals to grid indices
[~, si] = min(abs(latv - ENV.Hospitals.Latitude(srcHospital)));
[~, sj] = min(abs(lonv - ENV.Hospitals.Longitude(srcHospital)));

[~, di] = min(abs(latv - ENV.Hospitals.Latitude(dstHospital)));
[~, dj] = min(abs(lonv - ENV.Hospitals.Longitude(dstHospital)));

startNode = sub2ind([N N], si, sj);
goalNode  = sub2ind([N N], di, dj);

% Build graph (8-connected grid)
G = digraph;

for i = 2:N-1
    for j = 2:N-1
        
        idx = sub2ind([N N], i, j);
        
        for dx = -1:1
            for dy = -1:1
                
                if dx==0 && dy==0, continue; end
                
                ni = i + dx;
                nj = j + dy;
                
                nidx = sub2ind([N N], ni, nj);
                
                weight = Cost(ni,nj);
                
                G = addedge(G, idx, nidx, weight);
            end
        end
    end
end

[pathIdx, ~] = shortestpath(G, startNode, goalNode);

[path_i, path_j] = ind2sub([N N], pathIdx);

Path.lat = latv(path_i);
Path.lon = lonv(path_j);

end