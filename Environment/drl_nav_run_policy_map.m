function drl_nav_run_policy_map()

load("drl_nav_trained_agent.mat","agent");

ENV0 = environment_engine_cached();
env  = drl_nav_make_env(45*60);

obs = reset(env);

latTrack = [];
lonTrack = [];

isDone = false;
while ~isDone
    action = getAction(agent, obs);
    [obs,~,isDone,~] = step(env, action);
    latTrack(end+1) = obs(1);
    lonTrack(end+1) = obs(2);
end

figure; hold on; grid on;

plot(ENV0.Relays.Longitude, ENV0.Relays.Latitude, 'k.', 'MarkerSize', 10);
plot(ENV0.Hospitals.Longitude, ENV0.Hospitals.Latitude, 'ro', 'MarkerSize', 7, 'LineWidth', 1.5);
plot(lonTrack, latTrack, 'b', 'LineWidth', 2);

xlabel("Longitude");
ylabel("Latitude");
title("DRL-NAV UAV Trajectory (Trained PPO Policy)");
legend("Relays","Hospitals","UAV Path","Location","best");

end