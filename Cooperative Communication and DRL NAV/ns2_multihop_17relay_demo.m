function out = ns2_multihop_17relay_demo(varargin)
% NS2-style multihop simulation for 17-relay drone topology.
% Flow model (same concept as drone simulation):
%   UAV -> Access Relay -> Multihop Relay Chain -> Control Relay
%
% Example:
%   out = ns2_multihop_17relay_demo();
%   out = ns2_multihop_17relay_demo("numPackets",300,"enableRelayFailures",true);

cfg = parse_cfg(varargin{:});
rng(cfg.seed);

ENV = environment_engine_cached();
P = get_comm_params();

nRelays = height(ENV.Relays);
if nRelays ~= 17
    warning("Expected 17 relays, found %d. Simulation will run with available relays.", nRelays);
end

[relayDelay, relaySNR] = build_backbone_matrices(ENV, P, cfg);
[relayGraph, edgeTable] = build_ns2_graph_definition(relayDelay, relaySNR);
if cfg.controlRelayId > 0 && cfg.controlRelayId <= nRelays
    ctrlRelay = cfg.controlRelayId;
else
    ctrlRelay = nearest_relay_to_control(ENV, cfg);
end

alive = true(nRelays,1);
if cfg.enableRelayFailures
    [failRelays, failPacketIdx] = make_failure_schedule(nRelays, cfg);
else
    failRelays = [];
    failPacketIdx = [];
end

% NS2 event-state (per-node availability)
nextReady = zeros(nRelays,1);

logs = repmat(empty_log(), cfg.numPackets, 1);
paths = cell(cfg.numPackets, 1);
accessRelaySeq = zeros(cfg.numPackets,1);

t = 0;
delivered = 0;
dropNoRoute = 0;
dropLowSNR = 0;
dropRetry = 0;

for k = 1:cfg.numPackets
    % packet generation process
    t = t + exprnd(1 / max(cfg.packetRatePPS,1e-9));

    % apply scheduled failures at packet index k
    if cfg.enableRelayFailures
        idx = find(failPacketIdx == k);
        for ii = 1:numel(idx)
            r = failRelays(idx(ii));
            if r >= 1 && r <= nRelays
                alive(r) = false;
            end
        end
    end

    accessRelay = select_access_relay(k, nRelays, cfg);
    accessRelaySeq(k) = accessRelay;

    if ~alive(accessRelay) || ~alive(ctrlRelay)
        logs(k) = make_log(k, t, nan, false, "NoRoute", 0, nan, nan, 0);
        paths{k} = [];
        dropNoRoute = dropNoRoute + 1;
        continue;
    end

    [pathRelayIds, routeDelay, minPathSNR, routed] = control_multihop_path_ns2( ...
        accessRelay, ctrlRelay, alive, relayDelay, relaySNR, cfg.snrMin_dB);

    if ~routed
        logs(k) = make_log(k, t, nan, false, "NoRoute", 0, nan, nan, 0);
        paths{k} = [];
        dropNoRoute = dropNoRoute + 1;
        continue;
    end

    if minPathSNR < cfg.snrMin_dB
        logs(k) = make_log(k, t, nan, false, "LowSNR", numel(pathRelayIds)-1, nan, minPathSNR, 0);
        paths{k} = pathRelayIds;
        dropLowSNR = dropLowSNR + 1;
        continue;
    end

    [ok, tDone, retryCount] = tx_on_path(pathRelayIds, relayDelay, relaySNR, nextReady, t, cfg);
    nextReady = tDone.nextReady;

    if ok
        delivered = delivered + 1;
        e2e = tDone.t - t;
        logs(k) = make_log(k, t, tDone.t, true, "Delivered", numel(pathRelayIds)-1, e2e, minPathSNR, cfg.payloadBytes*8);
    else
        dropRetry = dropRetry + 1;
        e2e = tDone.t - t;
        logs(k) = make_log(k, t, tDone.t, false, "RetryExceeded", numel(pathRelayIds)-1, e2e, minPathSNR, retryCount);
    end

    paths{k} = pathRelayIds;
end

good = [logs.delivered];
delays = [logs.e2eDelaySec];
delays = delays(good & isfinite(delays));
if isempty(delays)
    meanDelay = nan;
    p95Delay = nan;
else
    meanDelay = mean(delays);
    p95Delay = prctile(delays,95);
end

tEnd = max([logs.arrivalTimeSec],[],"omitnan");
if isempty(tEnd) || ~isfinite(tEnd) || tEnd <= 0
    tEnd = cfg.numPackets / max(cfg.packetRatePPS,1e-9);
end
thrBps = sum([logs.payloadBitsDelivered],"omitnan") / tEnd;
pdr = delivered / max(cfg.numPackets,1);

out = struct();
out.config = cfg;
out.controlRelay = ctrlRelay;
out.accessRelaySequence = accessRelaySeq;
out.paths = paths;
out.logs = logs;
out.relayGraph = relayGraph;
out.edgeTable = edgeTable;
out.delivered = delivered;
out.dropNoRoute = dropNoRoute;
out.dropLowSNR = dropLowSNR;
out.dropRetry = dropRetry;
out.pdr = pdr;
out.meanDelaySec = meanDelay;
out.p95DelaySec = p95Delay;
out.throughputBps = thrBps;

fprintf("\n=== NS2 Multihop (17 Relay) Summary ===\n");
fprintf("Control Relay: R%d\n", ctrlRelay);
fprintf("Packets: %d | Delivered: %d\n", cfg.numPackets, delivered);
fprintf("Drops -> NoRoute:%d LowSNR:%d RetryExceeded:%d\n", dropNoRoute, dropLowSNR, dropRetry);
fprintf("PDR: %.2f %%\n", 100*pdr);
fprintf("Mean Delay: %.4f s | P95: %.4f s\n", meanDelay, p95Delay);
fprintf("Throughput: %.2f kbps\n", thrBps/1e3);
fprintf("=======================================\n");

if cfg.showFigures
    draw_ns2_multihop_figures(relayGraph, ENV, nRelays, ctrlRelay, accessRelaySeq, paths, logs, out, cfg);
end
end

function cfg = parse_cfg(varargin)
cfg = struct();
cfg.numPackets = 300;
cfg.packetRatePPS = 6;
cfg.payloadBytes = 1200;
cfg.maxRetriesPerHop = 2;
cfg.snrMin_dB = 5;
cfg.linkConnectSNR_dB = 0;
cfg.maxHopDelay_s = 0.03;
cfg.controlLat = 12.9965918;
cfg.controlLon = 80.1708076;
cfg.seed = 21;
cfg.showFigures = true;
cfg.maxAnimatePackets = 80;
cfg.pauseSec = 0.03;
cfg.pauseBeforeFlowSec = 1.5;
cfg.enableRelayFailures = false;
cfg.failureCount = 3;
cfg.failureStartPacket = 80;
cfg.failureEndPacket = 220;
cfg.fixedAccessRelayId = 0; % 0 => rotating access relay
cfg.controlRelayId = 0;     % 0 => nearest relay to control station
cfg.topologyLayout = "geo"; % "geo" uses real lon/lat, "circle" uses abstract layout

if mod(numel(varargin),2) ~= 0
    error("Use name-value pairs.");
end
for i = 1:2:numel(varargin)
    key = string(varargin{i});
    val = varargin{i+1};
    if isfield(cfg, key)
        cfg.(key) = val;
    else
        error("Unknown parameter: %s", key);
    end
end
end

function [relayDelay, relaySNR] = build_backbone_matrices(ENV, P, cfg)
n = height(ENV.Relays);
relayDelay = inf(n,n);
relaySNR = -inf(n,n);
for i = 1:n
    relayDelay(i,i) = 0;
    relaySNR(i,i) = inf;
    for j = i+1:n
        m = link_metrics_snr_prob( ...
            ENV.Relays.Latitude(i), ENV.Relays.Longitude(i), ...
            ENV.Relays.Latitude(j), ENV.Relays.Longitude(j), P);
        if m.SNR_dB >= cfg.linkConnectSNR_dB && m.Delay_s <= cfg.maxHopDelay_s
            relayDelay(i,j) = m.Delay_s;
            relayDelay(j,i) = m.Delay_s;
            relaySNR(i,j) = m.SNR_dB;
            relaySNR(j,i) = m.SNR_dB;
        end
    end
end
end

function [G, edgeTable] = build_ns2_graph_definition(relayDelay, relaySNR)
n = size(relayDelay,1);
[ii, jj] = find(triu(relayDelay,1) < inf);
if isempty(ii)
    G = graph();
    edgeTable = table();
    return;
end

w = relayDelay(sub2ind(size(relayDelay), ii, jj));
snr = relaySNR(sub2ind(size(relaySNR), ii, jj));
G = graph(ii, jj, w, n);
edgeTable = table(ii, jj, w, snr, 'VariableNames', {'fromRelay','toRelay','delaySec','snrDB'});
end

function ctrlRelay = nearest_relay_to_control(ENV, cfg)
n = height(ENV.Relays);
d = zeros(n,1);
for i = 1:n
    d(i) = haversine_km(ENV.Relays.Latitude(i), ENV.Relays.Longitude(i), cfg.controlLat, cfg.controlLon);
end
[~, ctrlRelay] = min(d);
end

function relay = select_access_relay(pktIdx, nRelays, cfg)
% Deterministic handoff-like sequence for presentation.
if cfg.fixedAccessRelayId > 0
    relay = min(max(round(cfg.fixedAccessRelayId), 1), nRelays);
    return;
end
if nRelays <= 1
    relay = 1;
    return;
end
period = max(8, round(cfg.numPackets / 12));
relay = 1 + mod(floor((pktIdx-1)/period), nRelays);
end

function [pathRelayIds, e2eDelay, minPathSNR, delivered] = control_multihop_path_ns2( ...
    accessRelayIdx, ctrlRelayIdx, aliveMask, relayDelay, relaySNR, snrMin)

pathRelayIds = [];
e2eDelay = inf;
minPathSNR = -inf;
delivered = false;

if accessRelayIdx <= 0 || ctrlRelayIdx <= 0
    return;
end
if ~aliveMask(accessRelayIdx) || ~aliveMask(ctrlRelayIdx)
    return;
end

aliveIds = find(aliveMask);
srcNew = find(aliveIds == accessRelayIdx, 1);
dstNew = find(aliveIds == ctrlRelayIdx, 1);
if isempty(srcNew) || isempty(dstNew)
    return;
end

W = relayDelay(aliveIds, aliveIds);
[ri, ci] = find(triu(W,1) < inf);
if isempty(ri)
    return;
end
wi = W(sub2ind(size(W), ri, ci));
G = graph(ri, ci, wi);

[pathNew, pathCost] = shortestpath(G, srcNew, dstNew);
if isempty(pathNew) || ~isfinite(pathCost)
    return;
end

pathRelayIds = aliveIds(pathNew);
e2eDelay = pathCost;

minPathSNR = inf;
for k = 1:(numel(pathRelayIds)-1)
    ii = pathRelayIds(k);
    jj = pathRelayIds(k+1);
    minPathSNR = min(minPathSNR, relaySNR(ii,jj));
end
if ~isfinite(minPathSNR)
    minPathSNR = -inf;
end

delivered = (minPathSNR >= snrMin);
end

function [ok, doneState, retryCnt] = tx_on_path(pathIds, relayDelay, relaySNR, nextReady, t0, cfg)
ok = true;
retryCnt = 0;
t = t0;

for h = 1:(numel(pathIds)-1)
    u = pathIds(h);
    v = pathIds(h+1);
    oneHop = relayDelay(u,v);
    if ~isfinite(oneHop)
        ok = false;
        break;
    end

    snr = relaySNR(u,v);
    pSucc = snr_to_success_prob(snr, cfg.snrMin_dB);

    tStart = max(t, nextReady(u));
    tEnd = tStart + oneHop;

    success = false;
    for r = 0:cfg.maxRetriesPerHop
        if rand() < pSucc
            success = true;
            retryCnt = retryCnt + r;
            break;
        end
        tEnd = tEnd + oneHop;
    end

    nextReady(u) = tEnd;
    if ~success
        ok = false;
        break;
    end

    t = tEnd;
end

doneState = struct("t", t, "nextReady", nextReady);
end

function p = snr_to_success_prob(snr_dB, snrMin_dB)
p = 1 ./ (1 + exp(-0.9*(snr_dB - snrMin_dB)));
p = min(max(p, 1e-3), 0.999);
end

function [failRelays, failPacketIdx] = make_failure_schedule(nRelays, cfg)
k = min(max(1, cfg.failureCount), nRelays);
failRelays = randperm(nRelays, k);
lo = min(cfg.failureStartPacket, cfg.failureEndPacket);
hi = max(cfg.failureStartPacket, cfg.failureEndPacket);
failPacketIdx = randi([max(1,lo), max(lo,hi)], 1, k);
end

function draw_ns2_multihop_figures(relayGraph, ENV, nRelays, ctrlRelay, accessRelaySeq, paths, logs, out, cfg)
if strcmpi(string(cfg.topologyLayout), "geo")
    xy = [ENV.Relays.Longitude, ENV.Relays.Latitude];
    xLabelTxt = 'Longitude';
    yLabelTxt = 'Latitude';
else
    th = linspace(0, 2*pi, nRelays+1).';
    th(end) = [];
    xy = [cos(th), sin(th)];
    xLabelTxt = 'Topology X';
    yLabelTxt = 'Topology Y';
end

fig1 = figure('Name','NS2 Multihop Topology (17 Relays)','Color','w');
ax1 = axes(fig1);
hold(ax1,'on'); grid(ax1,'on'); axis(ax1,'equal');
xlabel(ax1,xLabelTxt); ylabel(ax1,yLabelTxt);
title(ax1,'Phase 1: NS2 Relay Graph Definition');

if numnodes(relayGraph) > 0
    plot(ax1, relayGraph, 'XData', xy(:,1), 'YData', xy(:,2), ...
        'NodeLabel', {}, 'LineWidth', 1.1, 'EdgeColor', [0.6 0.6 0.6]);
end

scatter(ax1, xy(:,1), xy(:,2), 42, [0.15 0.15 0.15], 'filled');
scatter(ax1, xy(ctrlRelay,1), xy(ctrlRelay,2), 140, [0.8 0 0.8], 'filled');
text(xy(ctrlRelay,1), xy(ctrlRelay,2), sprintf('  CTRL R%d',ctrlRelay), 'FontWeight','bold');

for r = 1:nRelays
    text(xy(r,1), xy(r,2), sprintf(' R%d', r), 'FontSize', 8, 'Color', [0.1 0.1 0.1]);
end

drawnow;
pause(cfg.pauseBeforeFlowSec);

hPath = plot(ax1, nan, nan, '-', 'LineWidth', 2.2, 'Color', [0.1 0.7 0.2]);
hPkt  = scatter(ax1, nan, nan, 130, [1 0.9 0], 'filled', 'MarkerEdgeColor','k');
hAcc  = scatter(ax1, nan, nan, 120, [0 0.6 1], 'filled');

title(ax1,'Phase 2: Packet Flow (Access Relay -> Multihop -> Control Relay)');

nAnim = min(cfg.maxAnimatePackets, numel(logs));
for k = 1:nAnim
    ar = accessRelaySeq(k);
    set(hAcc, 'XData', xy(ar,1), 'YData', xy(ar,2));

    pth = paths{k};
    ok = logs(k).delivered;
    if isempty(pth)
        set(hPath,'XData',nan,'YData',nan,'Color',[0.9 0.2 0.2]);
        set(hPkt,'XData',xy(ar,1),'YData',xy(ar,2),'MarkerFaceColor',[0.9 0.2 0.2]);
        title(ax1, sprintf('Packet %d/%d | Access:R%d -> CTRL:R%d | %s', ...
            k, nAnim, ar, ctrlRelay, logs(k).status));
        drawnow;
        pause(cfg.pauseSec);
        continue;
    end

    px = xy(pth,1); py = xy(pth,2);
    if ok
        col = [0.1 0.7 0.2];
    else
        col = [0.9 0.2 0.2];
    end
    set(hPath,'XData',px,'YData',py,'Color',col);

    for h = 1:numel(pth)
        set(hPkt,'XData',px(h),'YData',py(h),'MarkerFaceColor',col);
        title(ax1, sprintf('Packet %d/%d | Access:R%d | Hop %d/%d | %s', ...
            k, nAnim, ar, h, numel(pth), logs(k).status));
        drawnow;
        pause(cfg.pauseSec);
    end
end

fig2 = figure('Name','NS2 Multihop KPIs','Color','w');
tiledlayout(fig2,2,2,'Padding','compact','TileSpacing','compact');

nexttile;
bar([out.delivered, out.dropNoRoute, out.dropLowSNR, out.dropRetry], 'FaceColor',[0.2 0.5 0.9]);
set(gca,'XTickLabel',{'Delivered','NoRoute','LowSNR','RetryFail'});
title('Packet Outcomes');
ylabel('Count');
grid on;

nexttile;
d = [logs.e2eDelaySec];
d = d([logs.delivered] & isfinite(d));
if isempty(d)
    histogram(0);
else
    histogram(d,12);
end
title('E2E Delay Histogram');
xlabel('Delay (s)'); ylabel('Packets');
grid on;

nexttile;
vals = [100*out.pdr, out.meanDelaySec*1000, out.p95DelaySec*1000, out.throughputBps/1e3];
bar(vals, 'FaceColor',[0.1 0.7 0.4]);
set(gca,'XTickLabel',{'PDR %','Mean ms','P95 ms','Thr kbps'});
title('Key KPIs');
grid on;

nexttile;
arr = [logs.arrivalTimeSec];
succ = [logs.delivered];
bits = [logs.payloadBitsDelivered];
valid = succ & isfinite(arr);
if nnz(valid) > 2
    a = sort(arr(valid));
    tBins = linspace(min(a), max(a), 12);
    kbps = zeros(numel(tBins)-1,1);
    for i = 1:(numel(tBins)-1)
        ii = valid & arr >= tBins(i) & arr < tBins(i+1);
        kbps(i) = sum(bits(ii)) / max(tBins(i+1)-tBins(i),1e-6) / 1e3;
    end
    plot(tBins(1:end-1), kbps, '-o', 'LineWidth', 1.6);
else
    plot(0,0,'o');
end
title('Throughput Trend');
xlabel('Time (s)'); ylabel('kbps');
grid on;
end

function x = empty_log()
x = struct("pktId",0,"genTimeSec",nan,"arrivalTimeSec",nan,"delivered",false, ...
    "status","","hopCount",0,"e2eDelaySec",nan,"minSNRdB",nan,"payloadBitsDelivered",0);
end

function x = make_log(id,t0,t1,ok,status,hops,e2e,minSnr,bits)
x = struct("pktId",id,"genTimeSec",t0,"arrivalTimeSec",t1,"delivered",ok, ...
    "status",status,"hopCount",hops,"e2eDelaySec",e2e,"minSNRdB",minSnr,"payloadBitsDelivered",bits);
end
