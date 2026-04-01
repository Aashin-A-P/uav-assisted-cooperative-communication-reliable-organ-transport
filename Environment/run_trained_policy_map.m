function run_trained_policy_map_ppo()

clc; clear;

% ================= Load PPO Agent =================
load("trained_ppo_agent.mat","agent");

env = make_drl_env_ppo(45*60);

% Reset environment
[obs, logged] = reset(env);

latPath = [];
lonPath = [];
handoffIdx = [];

prevRelay = [];
stepCount = 0;

done = false;

while ~done && stepCount < 700
    
    stepCount = stepCount + 1;
    
    % PPO returns cell
    actionCell = getAction(agent, obs);
    action = actionCell{1};  
    
    [obs, reward, done, logged] = step(env, action);
    
    latPath(end+1) = obs(1);
    lonPath(end+1) = obs(2);
    
    % Detect relay handoff
    currentRelay = logged.prevRelay;
    if ~isempty(prevRelay) && currentRelay ~= prevRelay
        handoffIdx(end+1) = length(latPath);
    end
    prevRelay = currentRelay;
    
end

% ================= Extract Info =================
ENV = environment_engine_cached();

src = logged.srcHospital;
dst = logged.dstHospital;

srcLat = ENV.Hospitals.Latitude(src);
srcLon = ENV.Hospitals.Longitude(src);

dstLat = ENV.Hospitals.Latitude(dst);
dstLon = ENV.Hospitals.Longitude(dst);

remainingEnergy = logged.energy;
remainingCIT = logged.CITsec - logged.elapsedSec;

% ================= Plot =================
figure('Color','w','Position',[100 80 1400 800]);

gx = geoaxes;
geobasemap(gx,"satellite");
hold(gx,"on");

% Relays
geoscatter(gx, ENV.Relays.Latitude, ENV.Relays.Longitude, ...
    30, 'b', 'filled');

% Hospitals
geoscatter(gx, ENV.Hospitals.Latitude, ENV.Hospitals.Longitude, ...
    50, 'w', 'filled');

% Source & Destination
geoscatter(gx, srcLat, srcLon, 120, 'g', 'filled');
geoscatter(gx, dstLat, dstLon, 120, 'r', 'filled');

% UAV Path
geoplot(gx, latPath, lonPath, ...
    'y-', 'LineWidth', 3);

% Handoff markers
if ~isempty(handoffIdx)
    geoscatter(gx, latPath(handoffIdx), lonPath(handoffIdx), ...
        100, 'c', 'filled');
end

title(gx, sprintf("PPO Organ Delivery | Stage %d | Organ: %s", ...
    logged.stage, logged.organ));

legend(gx, ...
    "Relay UAVs", ...
    "Hospitals", ...
    "Source", ...
    "Destination", ...
    "UAV Trajectory", ...
    "Handoff Points", ...
    "Location","northoutside");

grid(gx,"on");

% ================= Console Summary =================
disp("==============================================");
disp("PPO Mission Summary");
disp("==============================================");
fprintf("Curriculum Stage     : %d\n", logged.stage);
fprintf("Organ Type           : %s\n", logged.organ);
fprintf("Source Hospital      : H%d\n", src);
fprintf("Destination Hospital : H%d\n", dst);
fprintf("Total Steps          : %d\n", stepCount);
fprintf("Total Handoffs       : %d\n", length(handoffIdx));
fprintf("Remaining Energy     : %.3f\n", remainingEnergy);
fprintf("Remaining CIT (sec)  : %.2f\n", remainingCIT);
disp("==============================================");

if remainingCIT <= 0
    disp("❌ Mission Failed (CIT expired)");
elseif remainingEnergy <= 0
    disp("❌ Mission Failed (Battery depleted)");
elseif done
    disp("✅ Mission Completed Successfully");
else
    disp("⚠ Mission ended due to step limit");
end

end