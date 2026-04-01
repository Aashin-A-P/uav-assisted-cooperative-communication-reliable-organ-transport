function out = simulate_ns2_sdr_relay_network(varargin)
% NS2-style packet simulation on relay graph with SDR-like link behavior.
% Example:
%   out = simulate_ns2_sdr_relay_network();
%   out = simulate_ns2_sdr_relay_network("numPackets",300,"srcHospital",9,"dstHospital",15);

cfg = parse_cfg(varargin{:});
rng(cfg.seed);

ENV = environment_engine_cached();
P = get_comm_params();

R = ENV.Relays;
nR = height(R);

if cfg.srcHospital < 1 || cfg.srcHospital > height(ENV.Hospitals)
    error("srcHospital out of range.");
end
if cfg.dstHospital < 1 || cfg.dstHospital > height(ENV.Hospitals)
    error("dstHospital out of range.");
end

srcLat = ENV.Hospitals.Latitude(cfg.srcHospital);
srcLon = ENV.Hospitals.Longitude(cfg.srcHospital);
dstLat = ENV.Hospitals.Latitude(cfg.dstHospital);
dstLon = ENV.Hospitals.Longitude(cfg.dstHospital);

srcRelay = nearest_relay_idx(R, srcLat, srcLon);
dstRelay = nearest_relay_idx(R, dstLat, dstLon);

[G, linkTable] = build_relay_graph(R, P, cfg);

if numnodes(G) == 0
    error("Relay graph is empty.");
end
if srcRelay == dstRelay
    warning("Source and destination map to same relay. Trivial path.");
end

nextNodeReady = zeros(nR,1);
txLogs = repmat(empty_log_row(), cfg.numPackets, 1);
packetPaths = cell(cfg.numPackets,1);

generated = 0;
delivered = 0;
droppedNoPath = 0;
droppedRetries = 0;

t = 0;
for pktId = 1:cfg.numPackets
    inter = exprnd(1 / max(cfg.packetRatePPS, 1e-9));
    t = t + inter;
    generated = generated + 1;

    [pathNodes, pathCost] = shortestpath(G, srcRelay, dstRelay);
    if isempty(pathNodes) || ~isfinite(pathCost)
        droppedNoPath = droppedNoPath + 1;
        packetPaths{pktId} = [];
        txLogs(pktId) = make_log(pktId, t, nan, false, "NoPath", 0, nan, nan, 0);
        continue;
    end
    packetPaths{pktId} = pathNodes;

    [ok, tDone, retriesUsed, e2eDelay, minSNR, bitsDelivered] = ...
        transmit_one_packet(pathNodes, linkTable, nextNodeReady, t, cfg);

    nextNodeReady = tDone.nextNodeReady;

    if ok
        delivered = delivered + 1;
        txLogs(pktId) = make_log(pktId, t, tDone.time, true, "Delivered", numel(pathNodes)-1, e2eDelay, minSNR, bitsDelivered);
    else
        droppedRetries = droppedRetries + 1;
        txLogs(pktId) = make_log(pktId, t, tDone.time, false, "RetryExceeded", numel(pathNodes)-1, e2eDelay, minSNR, retriesUsed);
    end
end

goodIdx = [txLogs.delivered];
delays = [txLogs.e2eDelaySec];
delays = delays(goodIdx & isfinite(delays));

if isempty(delays)
    meanDelay = nan;
    p95Delay = nan;
else
    meanDelay = mean(delays);
    p95Delay = prctile(delays, 95);
end

totalTime = max([txLogs.arrivalTimeSec], [], "omitnan");
if isempty(totalTime) || ~isfinite(totalTime) || totalTime <= 0
    totalTime = cfg.numPackets / max(cfg.packetRatePPS, 1e-9);
end

throughputBps = sum([txLogs.payloadBitsDelivered], "omitnan") / totalTime;
deliveryRatio = delivered / max(generated, 1);

out = struct();
out.config = cfg;
out.srcHospital = cfg.srcHospital;
out.dstHospital = cfg.dstHospital;
out.srcRelay = srcRelay;
out.dstRelay = dstRelay;
out.graphNodes = numnodes(G);
out.graphEdges = numedges(G);
out.generated = generated;
out.delivered = delivered;
out.droppedNoPath = droppedNoPath;
out.droppedRetries = droppedRetries;
out.deliveryRatio = deliveryRatio;
out.meanDelaySec = meanDelay;
out.p95DelaySec = p95Delay;
out.throughputBps = throughputBps;
out.logs = txLogs;
out.packetPaths = packetPaths;

fprintf("\n=== NS2-SDR Relay Simulation Summary ===\n");
fprintf("Hospitals: %d -> %d | Relays: R%d -> R%d\n", cfg.srcHospital, cfg.dstHospital, srcRelay, dstRelay);
fprintf("Graph: %d nodes, %d edges\n", out.graphNodes, out.graphEdges);
fprintf("Packets: generated=%d, delivered=%d\n", generated, delivered);
fprintf("Drop(NoPath)=%d | Drop(RetryExceeded)=%d\n", droppedNoPath, droppedRetries);
fprintf("PDR: %.2f %%\n", 100*out.deliveryRatio);
fprintf("Mean Delay: %.4f s | P95 Delay: %.4f s\n", out.meanDelaySec, out.p95DelaySec);
fprintf("Throughput: %.2f kbps\n", out.throughputBps/1e3);
fprintf("========================================\n");

if cfg.visualize
    animate_packet_flow(ENV, G, srcRelay, dstRelay, packetPaths, txLogs, cfg);
end

end

function cfg = parse_cfg(varargin)
cfg = struct();
cfg.srcHospital = 9;
cfg.dstHospital = 15;
cfg.numPackets = 200;
cfg.packetRatePPS = 4;
cfg.payloadBytes = 1200;
cfg.maxRetriesPerHop = 2;
cfg.snrLinkMin_dB = 2;     % graph connectivity threshold
cfg.snrDecodeMin_dB = 5;   % packet decode threshold
cfg.maxOneHopDelay_s = 0.02;
cfg.baseProcDelay_s = 0.0015;
cfg.queueDiscipline = "fifo";
cfg.seed = 42;
cfg.visualize = true;
cfg.maxAnimatePackets = 40;
cfg.animatePauseSec = 0.06;

% SDR-like parameters
cfg.sdr.sampleRateHz = 1.0e6;
cfg.sdr.channelBandwidthHz = 2.0e6;
cfg.sdr.modulationOrder = 4;    % QPSK
cfg.sdr.codingRate = 0.75;
cfg.sdr.implementationLoss_dB = 2.0;

if mod(numel(varargin),2) ~= 0
    error("Use name-value pairs.");
end

for i = 1:2:numel(varargin)
    k = string(varargin{i});
    v = varargin{i+1};
    if isfield(cfg, k)
        cfg.(k) = v;
    elseif startsWith(k, "sdr.")
        s = extractAfter(k, "sdr.");
        cfg.sdr.(s) = v;
    else
        error("Unknown parameter: %s", k);
    end
end
end

function [G, linkTable] = build_relay_graph(R, P, cfg)
nR = height(R);
rows = [];

for i = 1:nR
    for j = i+1:nR
        m = link_metrics_snr_prob(R.Latitude(i), R.Longitude(i), R.Latitude(j), R.Longitude(j), P);
        if m.SNR_dB >= cfg.snrLinkMin_dB && m.Delay_s <= cfg.maxOneHopDelay_s
            rows = [rows; i, j, m.SNR_dB, m.Delay_s]; %#ok<AGROW>
            rows = [rows; j, i, m.SNR_dB, m.Delay_s]; %#ok<AGROW>
        end
    end
end

if isempty(rows)
    G = digraph();
    linkTable = table();
    return;
end

src = rows(:,1);
dst = rows(:,2);
snr = rows(:,3);
propDelay = rows(:,4);

effSNR = snr - cfg.sdr.implementationLoss_dB;
rateBps = effective_rate_bps(effSNR, cfg);
edgeCost = propDelay + cfg.baseProcDelay_s + (cfg.payloadBytes*8)./max(rateBps,1);

G = digraph(src, dst, edgeCost, nR);
linkTable = table(src, dst, snr, effSNR, propDelay, rateBps, edgeCost, ...
    'VariableNames', {'src','dst','snr_dB','effSnr_dB','propDelay_s','rateBps','edgeCost_s'});
end

function [ok, tDone, retriesUsed, e2eDelay, minSNR, payloadBitsDelivered] = ...
    transmit_one_packet(pathNodes, linkTable, nextNodeReady, tArrival, cfg)

currT = tArrival;
retriesUsed = 0;
ok = true;
minSNR = inf;
payloadBitsDelivered = cfg.payloadBytes * 8;

for h = 1:(numel(pathNodes)-1)
    u = pathNodes(h);
    v = pathNodes(h+1);
    row = linkTable(linkTable.src == u & linkTable.dst == v, :);
    if isempty(row)
        ok = false;
        break;
    end

    txStart = max(currT, nextNodeReady(u));
    linkDelay = row.edgeCost_s(1);
    txEnd = txStart + linkDelay;

    thisSNR = row.effSnr_dB(1);
    minSNR = min(minSNR, thisSNR);

    pSucc = snr_to_success_prob(thisSNR, cfg.snrDecodeMin_dB);
    delivered = false;
    for r = 0:cfg.maxRetriesPerHop
        if rand() < pSucc
            delivered = true;
            retriesUsed = retriesUsed + r;
            break;
        end
        txEnd = txEnd + linkDelay;
    end

    nextNodeReady(u) = txEnd;

    if ~delivered
        ok = false;
        payloadBitsDelivered = 0;
        break;
    end

    currT = txEnd;
end

if ~isfinite(minSNR)
    minSNR = nan;
end

tDone = struct("time", currT, "nextNodeReady", nextNodeReady);
e2eDelay = currT - tArrival;
end

function p = snr_to_success_prob(snr_dB, snrDecodeMin_dB)
% Smooth success curve around decode threshold.
slope = 0.8;
p = 1 ./ (1 + exp(-slope*(snr_dB - snrDecodeMin_dB)));
p = min(max(p, 1e-3), 0.999);
end

function rbps = effective_rate_bps(snr_dB, cfg)
snrLin = 10.^(snr_dB/10);
shannon = cfg.sdr.channelBandwidthHz .* log2(1 + snrLin);
modCeil = cfg.sdr.sampleRateHz * log2(cfg.sdr.modulationOrder);
rbps = min(shannon, modCeil) * cfg.sdr.codingRate;
end

function idx = nearest_relay_idx(R, lat, lon)
n = height(R);
d = zeros(n,1);
for i = 1:n
    d(i) = haversine_km(lat, lon, R.Latitude(i), R.Longitude(i));
end
[~, idx] = min(d);
end

function z = empty_log_row()
z = struct( ...
    "pktId", 0, ...
    "genTimeSec", nan, ...
    "arrivalTimeSec", nan, ...
    "delivered", false, ...
    "status", "", ...
    "hopCount", 0, ...
    "e2eDelaySec", nan, ...
    "minSNRdB", nan, ...
    "payloadBitsDelivered", 0);
end

function z = make_log(pktId, genT, arrT, ok, status, hops, e2e, minSnr, bits)
z = struct( ...
    "pktId", pktId, ...
    "genTimeSec", genT, ...
    "arrivalTimeSec", arrT, ...
    "delivered", ok, ...
    "status", status, ...
    "hopCount", hops, ...
    "e2eDelaySec", e2e, ...
    "minSNRdB", minSnr, ...
    "payloadBitsDelivered", bits);
end

function animate_packet_flow(ENV, G, srcRelay, dstRelay, packetPaths, txLogs, cfg)
if numnodes(G) < 1
    return;
end

fig = figure('Name','NS2-SDR Packet Flow','Color','w');
ax = axes(fig);
hold(ax,'on');
grid(ax,'on');
xlabel(ax,'Longitude');
ylabel(ax,'Latitude');
title(ax,'Relay Graph and Packet Flow');

relayLon = ENV.Relays.Longitude;
relayLat = ENV.Relays.Latitude;

plot(G, 'XData', relayLon, 'YData', relayLat, ...
    'NodeLabel', {}, 'ArrowSize', 8, 'LineWidth', 0.8, ...
    'EdgeAlpha', 0.35, 'NodeColor', [0.2 0.2 0.2], 'MarkerSize', 4, ...
    'Parent', ax);

scatter(ax, ENV.Hospitals.Longitude, ENV.Hospitals.Latitude, 20, 'r', 'filled');
scatter(ax, ENV.Hospitals.Longitude(cfg.srcHospital), ENV.Hospitals.Latitude(cfg.srcHospital), 90, 'g', 'filled');
scatter(ax, ENV.Hospitals.Longitude(cfg.dstHospital), ENV.Hospitals.Latitude(cfg.dstHospital), 90, 'm', 'filled');
scatter(ax, relayLon(srcRelay), relayLat(srcRelay), 100, 'b', 'filled');
scatter(ax, relayLon(dstRelay), relayLat(dstRelay), 100, 'c', 'filled');

hPath = plot(ax, nan, nan, 'c-', 'LineWidth', 2.5);
hPkt  = scatter(ax, nan, nan, 110, [1 0.9 0], 'filled', 'MarkerEdgeColor', 'k');

nAnim = min(cfg.maxAnimatePackets, numel(txLogs));
for k = 1:nAnim
    path = packetPaths{k};
    delivered = txLogs(k).delivered;
    statusTxt = string(txLogs(k).status);

    if isempty(path)
        set(hPath,'XData',nan,'YData',nan,'Color',[0.8 0.2 0.2]);
        set(hPkt,'XData',relayLon(srcRelay),'YData',relayLat(srcRelay), ...
            'MarkerFaceColor',[0.9 0.2 0.2]);
        title(ax, sprintf('Packet %d/%d | STATUS: %s (No route)', k, nAnim, statusTxt));
        drawnow;
        pause(cfg.animatePauseSec);
        continue;
    end

    lonPath = relayLon(path);
    latPath = relayLat(path);
    if delivered
        set(hPath,'Color',[0.1 0.7 0.2]);
    else
        set(hPath,'Color',[0.9 0.2 0.2]);
    end
    set(hPath,'XData',lonPath,'YData',latPath);

    for h = 1:numel(path)
        set(hPkt,'XData',lonPath(h),'YData',latPath(h));
        if delivered
            set(hPkt,'MarkerFaceColor',[1 0.9 0]);
        else
            set(hPkt,'MarkerFaceColor',[0.95 0.3 0.2]);
        end
        title(ax, sprintf('Packet %d/%d | STATUS: %s | Hop %d/%d', ...
            k, nAnim, statusTxt, h, numel(path)));
        drawnow;
        pause(cfg.animatePauseSec);
    end
end
end
