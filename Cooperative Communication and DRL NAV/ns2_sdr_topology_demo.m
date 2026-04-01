function out = ns2_sdr_topology_demo(varargin)
% NS2-style event simulation on relay graph (abstract topology view).
% Uses real environment + link model, but visualizes node graph (not map coords).
%
% Example:
%   out = ns2_sdr_topology_demo();
%   out = ns2_sdr_topology_demo("numPackets",300,"packetRatePPS",8);

cfg = parse_cfg(varargin{:});
rng(cfg.seed);

ENV = environment_engine_cached();
P = get_comm_params();
R = ENV.Relays;
nR = height(R);

srcRelay = nearest_relay_idx(R, ENV.Hospitals.Latitude(cfg.srcHospital), ENV.Hospitals.Longitude(cfg.srcHospital));
dstRelay = nearest_relay_idx(R, ENV.Hospitals.Latitude(cfg.dstHospital), ENV.Hospitals.Longitude(cfg.dstHospital));

[G, linkTable, usedSNRth] = build_graph_with_adaptive_threshold(R, P, cfg, srcRelay, dstRelay);

if numnodes(G) < 1
    error("Graph creation failed.");
end

nextReady = zeros(nR,1);
logs = repmat(empty_log(), cfg.numPackets, 1);
paths = cell(cfg.numPackets,1);

t = 0;
generated = 0;
delivered = 0;
dropNoPath = 0;
dropRetry = 0;

for k = 1:cfg.numPackets
    t = t + exprnd(1 / max(cfg.packetRatePPS, 1e-9));
    generated = generated + 1;

    [pathNodes, c] = shortestpath(G, srcRelay, dstRelay);
    if isempty(pathNodes) || ~isfinite(c)
        dropNoPath = dropNoPath + 1;
        logs(k) = make_log(k, t, nan, false, "NoPath", 0, nan, nan, 0);
        paths{k} = [];
        continue;
    end

    [ok, doneState, retryCount, e2e, minSNR, bits] = ...
        tx_packet(pathNodes, linkTable, nextReady, t, cfg);
    nextReady = doneState.nextReady;
    paths{k} = pathNodes;

    if ok
        delivered = delivered + 1;
        logs(k) = make_log(k, t, doneState.t, true, "Delivered", numel(pathNodes)-1, e2e, minSNR, bits);
    else
        dropRetry = dropRetry + 1;
        logs(k) = make_log(k, t, doneState.t, false, "RetryExceeded", numel(pathNodes)-1, e2e, minSNR, retryCount);
    end
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

lastT = max([logs.arrivalTimeSec],[],"omitnan");
if isempty(lastT) || ~isfinite(lastT) || lastT <= 0
    lastT = cfg.numPackets / max(cfg.packetRatePPS, 1e-9);
end

throughputBps = sum([logs.payloadBitsDelivered],"omitnan") / lastT;
pdr = delivered / max(generated,1);

out = struct();
out.config = cfg;
out.usedSNRthreshold_dB = usedSNRth;
out.srcRelay = srcRelay;
out.dstRelay = dstRelay;
out.generated = generated;
out.delivered = delivered;
out.droppedNoPath = dropNoPath;
out.droppedRetries = dropRetry;
out.pdr = pdr;
out.meanDelaySec = meanDelay;
out.p95DelaySec = p95Delay;
out.throughputBps = throughputBps;
out.logs = logs;
out.paths = paths;
out.graph = G;

fprintf("\n=== NS2-Style Topology Simulation ===\n");
fprintf("Source Relay: R%d | Destination Relay: R%d\n", srcRelay, dstRelay);
fprintf("Graph: %d nodes, %d edges | SNR threshold used: %.1f dB\n", numnodes(G), numedges(G), usedSNRth);
fprintf("Generated: %d | Delivered: %d\n", generated, delivered);
fprintf("Drop(NoPath): %d | Drop(Retry): %d\n", dropNoPath, dropRetry);
fprintf("PDR: %.2f %% | Mean Delay: %.4f s | P95: %.4f s\n", 100*pdr, meanDelay, p95Delay);
fprintf("Throughput: %.2f kbps\n", throughputBps/1e3);
fprintf("=====================================\n");

if cfg.showFigures
    draw_topology_and_results(G, srcRelay, dstRelay, paths, logs, out, cfg);
end
end

function cfg = parse_cfg(varargin)
cfg = struct();
cfg.srcHospital = 9;
cfg.dstHospital = 15;
cfg.numPackets = 300;
cfg.packetRatePPS = 6;
cfg.payloadBytes = 1200;
cfg.maxRetriesPerHop = 2;
cfg.snrLinkMin_dB = 2;
cfg.snrDecodeMin_dB = 5;
cfg.maxOneHopDelay_s = 0.03;
cfg.baseProcDelay_s = 0.0015;
cfg.seed = 12;
cfg.showFigures = true;
cfg.maxAnimatePackets = 60;
cfg.pauseSec = 0.03;

cfg.sdr.sampleRateHz = 1e6;
cfg.sdr.channelBandwidthHz = 2e6;
cfg.sdr.modulationOrder = 4;
cfg.sdr.codingRate = 0.75;
cfg.sdr.implementationLoss_dB = 2;

if mod(numel(varargin),2) ~= 0
    error("Use name-value pairs.");
end
for i = 1:2:numel(varargin)
    k = string(varargin{i});
    v = varargin{i+1};
    if isfield(cfg, k)
        cfg.(k) = v;
    elseif startsWith(k,"sdr.")
        cfg.sdr.(extractAfter(k,"sdr.")) = v;
    else
        error("Unknown parameter: %s", k);
    end
end
end

function [G, L, usedThr] = build_graph_with_adaptive_threshold(R, P, cfg, srcRelay, dstRelay)
thrList = [cfg.snrLinkMin_dB, 0, -2, -5, -8];
usedThr = thrList(end);
G = digraph();
L = table();

for thr = thrList
    [Gcand, Lcand] = build_graph(R, P, cfg, thr);
    if numnodes(Gcand) < 1
        continue;
    end
    [pth, c] = shortestpath(Gcand, srcRelay, dstRelay);
    G = Gcand;
    L = Lcand;
    usedThr = thr;
    if ~isempty(pth) && isfinite(c)
        return;
    end
end
end

function [G, L] = build_graph(R, P, cfg, snrThr)
nR = height(R);
rows = [];
for i = 1:nR
    for j = i+1:nR
        m = link_metrics_snr_prob(R.Latitude(i), R.Longitude(i), R.Latitude(j), R.Longitude(j), P);
        if m.SNR_dB >= snrThr && m.Delay_s <= cfg.maxOneHopDelay_s
            rows = [rows; i, j, m.SNR_dB, m.Delay_s; j, i, m.SNR_dB, m.Delay_s]; %#ok<AGROW>
        end
    end
end
if isempty(rows)
    G = digraph();
    L = table();
    return;
end

src = rows(:,1); dst = rows(:,2); snr = rows(:,3); prop = rows(:,4);
effSNR = snr - cfg.sdr.implementationLoss_dB;
rbps = eff_rate(effSNR, cfg);
w = prop + cfg.baseProcDelay_s + (cfg.payloadBytes*8)./max(rbps,1);
G = digraph(src, dst, w, nR);
L = table(src,dst,snr,effSNR,prop,rbps,w, ...
    'VariableNames',{'src','dst','snr_dB','effSnr_dB','propDelay_s','rateBps','edgeCost_s'});
end

function [ok, doneState, retryCnt, e2eDelay, minSnr, bits] = tx_packet(pathNodes, L, nextReady, t0, cfg)
ok = true;
retryCnt = 0;
bits = cfg.payloadBytes*8;
t = t0;
minSnr = inf;

for h = 1:(numel(pathNodes)-1)
    u = pathNodes(h); v = pathNodes(h+1);
    row = L(L.src==u & L.dst==v, :);
    if isempty(row)
        ok = false;
        bits = 0;
        break;
    end

    d = row.edgeCost_s(1);
    effSNR = row.effSnr_dB(1);
    minSnr = min(minSnr, effSNR);
    pSucc = snr_success(effSNR, cfg.snrDecodeMin_dB);

    tStart = max(t, nextReady(u));
    tEnd = tStart + d;
    success = false;
    for r = 0:cfg.maxRetriesPerHop
        if rand() < pSucc
            success = true;
            retryCnt = retryCnt + r;
            break;
        end
        tEnd = tEnd + d;
    end

    nextReady(u) = tEnd;
    if ~success
        ok = false;
        bits = 0;
        break;
    end
    t = tEnd;
end

if ~isfinite(minSnr)
    minSnr = nan;
end
doneState = struct("t",t,"nextReady",nextReady);
e2eDelay = t - t0;
end

function p = snr_success(snr_dB, snrMin)
p = 1 ./ (1 + exp(-0.8*(snr_dB - snrMin)));
p = min(max(p,1e-3),0.999);
end

function rbps = eff_rate(snr_dB, cfg)
lin = 10.^(snr_dB/10);
sh = cfg.sdr.channelBandwidthHz .* log2(1 + lin);
cap = cfg.sdr.sampleRateHz * log2(cfg.sdr.modulationOrder);
rbps = min(sh,cap) * cfg.sdr.codingRate;
end

function draw_topology_and_results(G, srcRelay, dstRelay, paths, logs, out, cfg)
f1 = figure('Name','NS2 Relay Topology (Abstract)','Color','w');
ax = axes(f1); hold(ax,'on'); grid(ax,'on');
n = numnodes(G);
theta = linspace(0, 2*pi, n+1).';
theta(end) = [];
rad = 1.0;
xEven = rad * cos(theta);
yEven = rad * sin(theta);

pg = plot(ax, G, 'XData', xEven, 'YData', yEven, 'NodeLabel', {}, 'LineWidth', 1.0, 'ArrowSize', 9);

xy = [xEven(:), yEven(:)];
scatter(ax, xy(:,1), xy(:,2), 36, [0.2 0.2 0.2], 'filled');
scatter(ax, xy(srcRelay,1), xy(srcRelay,2), 120, [0 0.7 0], 'filled');
scatter(ax, xy(dstRelay,1), xy(dstRelay,2), 120, [0.8 0 0.8], 'filled');
text(xy(srcRelay,1), xy(srcRelay,2), sprintf('  SRC R%d', srcRelay), 'FontWeight','bold');
text(xy(dstRelay,1), xy(dstRelay,2), sprintf('  DST R%d', dstRelay), 'FontWeight','bold');
title(ax, 'NS2-style Relay Graph (Node-ID topology)');
xlabel(ax, 'Topology X');
ylabel(ax, 'Topology Y');

hPath = plot(ax, nan, nan, '-', 'LineWidth', 2.2, 'Color', [0 0.6 0.1]);
hPkt = scatter(ax, nan, nan, 120, [1 0.9 0], 'filled', 'MarkerEdgeColor','k');

nAnim = min(cfg.maxAnimatePackets, numel(logs));
for k = 1:nAnim
    pth = paths{k};
    ok = logs(k).delivered;
    if isempty(pth)
        set(hPath,'XData',nan,'YData',nan,'Color',[0.9 0.2 0.2]);
        set(hPkt,'XData',xy(srcRelay,1),'YData',xy(srcRelay,2),'MarkerFaceColor',[0.9 0.2 0.2]);
        title(ax, sprintf('Packet %d/%d | STATUS: %s', k, nAnim, logs(k).status));
        drawnow;
        pause(cfg.pauseSec);
        continue;
    end

    px = xy(pth,1); py = xy(pth,2);
    if ok, col = [0.1 0.7 0.2]; else, col = [0.9 0.2 0.2]; end
    set(hPath,'XData',px,'YData',py,'Color',col);
    for h = 1:numel(pth)
        set(hPkt,'XData',px(h),'YData',py(h),'MarkerFaceColor',col);
        title(ax, sprintf('Packet %d/%d | %s | Hop %d/%d', k, nAnim, logs(k).status, h, numel(pth)));
        drawnow;
        pause(cfg.pauseSec);
    end
end

f2 = figure('Name','NS2 Relay Results','Color','w');
tiledlayout(f2,2,2,'Padding','compact','TileSpacing','compact');

nexttile;
bar([out.delivered, out.droppedNoPath, out.droppedRetries], 'FaceColor',[0.2 0.5 0.9]);
set(gca,'XTickLabel',{'Delivered','NoPath','RetryFail'});
title('Packet Outcome');
ylabel('Count');
grid on;

nexttile;
d = [logs.e2eDelaySec];
d = d([logs.delivered] & isfinite(d));
if isempty(d)
    histogram(0);
else
    histogram(d, 12);
end
title('E2E Delay Histogram');
xlabel('Delay (s)');
ylabel('Packets');
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
goodArr = arr(succ & isfinite(arr));
if numel(goodArr) > 2
    goodArr = sort(goodArr);
    win = linspace(min(goodArr), max(goodArr), 12);
    thr = zeros(numel(win)-1,1);
    bits = [logs.payloadBitsDelivered];
    for i = 1:(numel(win)-1)
        idx = succ & arr >= win(i) & arr < win(i+1);
        thr(i) = sum(bits(idx)) / max(win(i+1)-win(i), 1e-6) / 1e3;
    end
    plot(win(1:end-1), thr, '-o', 'LineWidth', 1.6);
else
    plot(0,0,'o');
end
title('Throughput Trend');
xlabel('Time (s)');
ylabel('kbps');
grid on;
end

function idx = nearest_relay_idx(R, lat, lon)
n = height(R);
d = zeros(n,1);
for i = 1:n
    d(i) = haversine_km(lat, lon, R.Latitude(i), R.Longitude(i));
end
[~, idx] = min(d);
end

function x = empty_log()
x = struct("pktId",0,"genTimeSec",nan,"arrivalTimeSec",nan,"delivered",false, ...
    "status","","hopCount",0,"e2eDelaySec",nan,"minSNRdB",nan,"payloadBitsDelivered",0);
end

function x = make_log(id,t0,t1,ok,status,hops,e2e,minSnr,bits)
x = struct("pktId",id,"genTimeSec",t0,"arrivalTimeSec",t1,"delivered",ok, ...
    "status",status,"hopCount",hops,"e2eDelaySec",e2e,"minSNRdB",minSnr,"payloadBitsDelivered",bits);
end
