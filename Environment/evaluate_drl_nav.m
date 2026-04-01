function evaluate_drl_nav(agent, env)

% Number of evaluation episodes
numEval = 20;

% Turn OFF exploration (very important)
agent.UseExplorationPolicy = false;

successCount = 0;
timeoutCount = 0;
totalSteps = 0;
finalDistKm = zeros(numEval,1);

fprintf('\nRunning %d evaluation episodes...\n\n', numEval);

for ep = 1:numEval

    % Reset environment
    obs = reset(env);

    isDone = false;
    steps = 0;

    while ~isDone

        % Get deterministic action
        action = getAction(agent, obs);

        % Step environment
        [nextObs, ~, isDone, ~] = step(env, action);

        obs = nextObs;
        steps = steps + 1;

    end

    totalSteps = totalSteps + steps;

    % Extract final normalized distance
    distN = obs(5);

    % Convert normalized distance back to km
    % (You normalized as dist/20)
    finalDistKm(ep) = distN * 20;

    % Success if distance < 0.5 km
    % (You used successRadius between 0.25–0.5)
    if finalDistKm(ep) < 0.5
        successCount = successCount + 1;
    else
        timeoutCount = timeoutCount + 1;
    end

    fprintf('Episode %d → Steps: %d | FinalDist: %.3f km\n', ...
            ep, steps, finalDistKm(ep));

end

% -------- RESULTS --------
fprintf('\n============================\n');
fprintf('SUCCESS RATE: %.2f %%\n', (successCount/numEval)*100);
fprintf('Average Steps: %.2f\n', totalSteps/numEval);
fprintf('Average Final Distance: %.3f km\n', mean(finalDistKm));
fprintf('============================\n\n');

end