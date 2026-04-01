function out = best_relay_attach_baseline(Net, U, destRelayIdx, prevRelayIdx)

n = numel(U.Delay_s);

% Shortest path delays from every relay to destination over backbone
% Use distances from graph
numRelays = numnodes(Net.G);
dRelayToDest = distances(Net.G, 1:numRelays, destRelayIdx)';

% Cost = UAV->relay delay + relay->dest delay
cost = U.Delay_s(:) + dRelayToDest(:);

% Penalize switching too frequently (hysteresis)
handoffPenalty = 0.15; % seconds-equivalent penalty (tune)
if ~isempty(prevRelayIdx) && prevRelayIdx>=1 && prevRelayIdx<=n
    cost = cost + handoffPenalty * ( (1:n)' ~= prevRelayIdx );
end

% Invalidate unreachable relays
cost(~isfinite(cost)) = inf;

[bestCost, bestRelayIdx] = min(cost);

out.bestRelayIdx = bestRelayIdx;
out.bestCost_s = bestCost;
out.allCost_s = cost;
out.relayToDestDelay_s = dRelayToDest;
end