function run_trained_policy_map_ppo()

clc; clear;

load("trained_ppo_agent.mat","agent");

env = make_drl_env_ppo(45*60);

[obs, logged] = reset(env);

latPath = [];
lonPath = [];
handoffIdx = [];

prevRelay = [];
done = false;
maxRunSteps = 800;

for t = 1:maxRunSteps

    action = getAction(agent, obs);

    [obs, ~, done, logged] = step(env, action);

    latPath(end+1) = obs(1);
    lonPath(end+1) = obs(2);

    curRelay = logged.prevRelay;
    if ~isempty(prevRelay) && ~isempty(curRelay) && curRelay ~= prevRelay
        handoffIdx(end+1) = numel(latPath);
    end
    prevRelay = curRelay;

    if done
        break;
    end
end

ENV = environment_engine_cached();

src = logged.srcHospital;
dst = logged.dstHospital;

srcLat = ENV.Hospitals.Latitude(src);
srcLon = ENV.Hospitals.Longitude(src);
dstLat = ENV.Hospitals.Latitude(dst);
dstLon = ENV.Hospitals.Longitude(dst);

figure('Color','w','Position',[100 80 1500 800]);
gx = geoaxes;
geobasemap(gx,"satellite");
hold(gx,"on");

geoscatter(gx, ENV.Relays.Latitude, ENV.Relays.Longitude, 35, 'b', 'filled');
geoscatter(gx, ENV.Hospitals.Latitude, ENV.Hospitals.Longitude, 55, 'w', 'filled');

geoscatter(gx, srcLat, srcLon, 140, 'g', 'filled');
geoscatter(gx, dstLat, dstLon, 140, 'r', 'filled');

geoplot(gx, latPath, lonPath, 'y-', 'LineWidth', 3);

if ~isempty(handoffIdx)
    geoscatter(gx, latPath(handoffIdx), lonPath(handoffIdx), 110, 'c', 'filled');
end

title(gx, sprintf("PPO Delivery UAV Trajectory | Stage %d | %s", logged.stage, logged.organ));

legend(gx, "Relay UAVs","Hospitals","Source","Destination","UAV Path","Handoff Points", ...
    "Location","northoutside");

grid(gx,"on");

% summary
citLeft = max(0, logged.CITsec - logged.elapsedSec);

disp("==============================================");
disp("PPO Mission Summary");
disp("==============================================");
fprintf("Stage              : %d\n", logged.stage);
fprintf("Organ              : %s\n", logged.organ);
fprintf("Source Hospital     : H%d\n", src);
fprintf("Destination Hospital: H%d\n", dst);
fprintf("Steps              : %d\n", numel(latPath));
fprintf("Handoffs           : %d\n", numel(handoffIdx));
fprintf("Energy Left        : %.3f\n", logged.energy);
fprintf("CIT Left (sec)     : %.1f\n", citLeft);
disp("==============================================");

end