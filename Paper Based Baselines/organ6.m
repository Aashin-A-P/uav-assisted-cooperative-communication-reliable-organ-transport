clc; clear; close all;

%% ================== GLOBAL SETTINGS ==================
citySize = 100;
deadlineSteps = 420;
pauseTime = 0.08;
uavStep = 2;

%% ================== HOSPITAL NETWORK ==================
Hospitals = [10 20;
             20 85;
             45 60;
             80 85;
             85 30;
             90 70];

Hnames = ["Donor H1","Hosp H2","Hosp H3","Receiver H4","Hosp H5","Hosp H6"];
donorIdx = 1; receiverIdx = 4;

donor = Hospitals(donorIdx,:);
receiver = Hospitals(receiverIdx,:);

%% ================== NO-FLY ZONES ==================
noFly = [40 40 10;
         60 50 12];

newNoFly = [60 75 10];

%% ================== COMMUNICATION SETTINGS ==================
baseStation = [50 10];
baseRange  = 30;
relayRange = 22;

%% ================== UAV SETUP ==================
T.name = "Transport UAV";
T.pos  = donor;

RelayPos = [25 35;
            50 35;
            75 35;
            45 70;
            75 75];
nRelays = size(RelayPos,1);

%% ================== PATHS ==================
pathA = path_AstarStyle(donor, receiver, noFly);
pathB = path_RRTStyle(donor, receiver, noFly);
pathC = path_Optimized(donor, receiver, noFly);

paths = {pathA, pathB, pathC};
algoNames = ["A* Safe Path","RRT* Random Path","Optimized Path"];

%% ================== SELECT BEST PATH ==================
cost = zeros(1,3);
for i=1:3
    P = paths{i};
    ttime = size(P,1);
    reli  = mean(commReliability_5Relays(P, baseStation, RelayPos, baseRange, relayRange));
    cost(i)= 0.7*ttime - 50*reli;
end
[~,bestIdx] = min(cost);
currentPath = paths{bestIdx};

%% ================== FIGURE (LOCK SIZE!) ==================
fig = figure('Color','w');
set(fig,'Units','pixels','Position',[60 80 1280 720]);  % fixed 720p
set(fig,'Resize','off');                                % prevent resizing

%% ---------- LEFT: PATH PLANNING ----------
subplot(1,2,1); hold on; axis equal; grid on;
title("Path Planning (5 Relay UAV Network)");
xlim([0 citySize]); ylim([0 citySize]);
xlabel("X"); ylabel("Y");

hHosp = plot(Hospitals(:,1),Hospitals(:,2),'ks','MarkerFaceColor','y','MarkerSize',10);
for i=1:size(Hospitals,1)
    text(Hospitals(i,1)+1,Hospitals(i,2)+1,Hnames(i),'FontSize',9,'FontWeight','bold');
end

for i=1:size(noFly,1)
    drawCircle(noFly(i,1),noFly(i,2),noFly(i,3),[1 0.5 0.5],0.25);
end

plot(pathA(:,1),pathA(:,2),'b--','LineWidth',2);
plot(pathB(:,1),pathB(:,2),'m--','LineWidth',2);
plot(pathC(:,1),pathC(:,2),'c--','LineWidth',2);
plot(currentPath(:,1),currentPath(:,2),'r','LineWidth',4);

plot(RelayPos(:,1),RelayPos(:,2),'ko','MarkerSize',9,'MarkerFaceColor',[0.8 0.8 0.8]);
for i=1:nRelays
    text(RelayPos(i,1)+1, RelayPos(i,2), sprintf("Relay %d",i), 'FontWeight','bold');
end

text(5,95,"Selected Algorithm: "+algoNames(bestIdx),'FontSize',11,'FontWeight','bold','Color','r');

%% ---------- RIGHT: REAL TIME ----------
subplot(1,2,2); hold on; axis equal; grid on;
title("Real-Time Transport with Relay Handover");
xlim([0 citySize]); ylim([0 citySize]);
xlabel("X"); ylabel("Y");

plot(Hospitals(:,1),Hospitals(:,2),'ks','MarkerFaceColor','y','MarkerSize',10);
for i=1:size(Hospitals,1)
    text(Hospitals(i,1)+1,Hospitals(i,2)+1,Hnames(i),'FontSize',9,'FontWeight','bold');
end

for i=1:size(noFly,1)
    drawCircle(noFly(i,1),noFly(i,2),noFly(i,3),[1 0.5 0.5],0.25);
end

% Base station
drawCircle(baseStation(1),baseStation(2),baseRange,[0.6 1 0.6],0.15);
plot(baseStation(1),baseStation(2),'gp','MarkerSize',14,'MarkerFaceColor','g');
text(baseStation(1)+1, baseStation(2), "Base Station", 'FontWeight','bold');

% Relays + coverage (handles)
relayH = gobjects(nRelays,1);
for i=1:nRelays
    drawCircle(RelayPos(i,1),RelayPos(i,2),relayRange,[0.6 0.6 1],0.07);
    relayH(i) = plot(RelayPos(i,1),RelayPos(i,2),'ko','MarkerSize',10,'MarkerFaceColor',[0.8 0.8 0.8]);
    text(RelayPos(i,1)+1, RelayPos(i,2)-1, sprintf("R%d",i), 'FontWeight','bold');
end

% Transport UAV
tH = plot(T.pos(1),T.pos(2),'ro','MarkerSize',10,'MarkerFaceColor','r');
tLabel = text(T.pos(1)+1,T.pos(2),"Transport UAV",'FontWeight','bold');

% Links
linkTR = plot([T.pos(1) RelayPos(1,1)], [T.pos(2) RelayPos(1,2)], 'k-', 'LineWidth',2);
linkRB = plot([RelayPos(1,1) baseStation(1)], [RelayPos(1,2) baseStation(2)], 'g-', 'LineWidth',2);

trail = plot(T.pos(1),T.pos(2),'r','LineWidth',2);

drawnow;

%% ===== VIDEO RECORDING (FRAME SIZE FIX) =====
v = VideoWriter('Organ_5Relays_Handover_OK.mp4','MPEG-4');
v.FrameRate = 20;
open(v);

% capture first frame to lock size
firstFrame = getframe(fig);
writeVideo(v, firstFrame);
frameSize = size(firstFrame.cdata);

%% ================== SIMULATION LOOP ==================
pathStep = 1;
delivered = false;

for t = 1:deadlineSteps
    if pathStep > size(currentPath,1)
        delivered = true;
        break;
    end

    % Move transport UAV
    T.pos = currentPath(pathStep,:);
    set(tH,'XData',T.pos(1),'YData',T.pos(2));
    set(tLabel,'Position',[T.pos(1)+1 T.pos(2)]);

    % choose best relay
    [bestRelay, bestScore] = selectBestRelay(T.pos, baseStation, RelayPos, baseRange, relayRange);

    for i=1:nRelays
        set(relayH(i),'MarkerFaceColor',[0.8 0.8 0.8]);
    end
    set(relayH(bestRelay),'MarkerFaceColor','k');

    set(linkTR,'XData',[T.pos(1) RelayPos(bestRelay,1)], ...
               'YData',[T.pos(2) RelayPos(bestRelay,2)]);
    set(linkRB,'XData',[RelayPos(bestRelay,1) baseStation(1)], ...
               'YData',[RelayPos(bestRelay,2) baseStation(2)]);

    set(trail,'XData',currentPath(1:pathStep,1),'YData',currentPath(1:pathStep,2));

    % dynamic obstacle
    if t == 170
        drawCircle(newNoFly(1),newNoFly(2),newNoFly(3),[1 0.3 0.3],0.35);
        noFly=[noFly; newNoFly];

        donorNew = T.pos;
        pathA=path_AstarStyle(donorNew, receiver, noFly);
        pathB=path_RRTStyle(donorNew, receiver, noFly);
        pathC=path_Optimized(donorNew, receiver, noFly);
        paths={pathA,pathB,pathC};

        cost=zeros(1,3);
        for i=1:3
            P=paths{i};
            ttime=size(P,1);
            reli=mean(commReliability_5Relays(P, baseStation, RelayPos, baseRange, relayRange));
            cost(i)=0.7*ttime - 50*reli;
        end
        [~,bestIdx]=min(cost);
        currentPath=paths{bestIdx};
        pathStep=1;
    end

    subtitle(sprintf("Step: %d | Algo: %s | Relay: %d | LinkScore: %.2f", ...
        t, algoNames(bestIdx), bestRelay, bestScore));

    drawnow;

    frame = getframe(fig);

    % ensure constant frame size
    if ~isequal(size(frame.cdata), frameSize)
        % resize if needed
        frame.cdata = imresize(frame.cdata, [frameSize(1) frameSize(2)]);
    end

    writeVideo(v, frame);

    pause(pauseTime);
    pathStep = pathStep + uavStep;
end

close(v);
disp("✅ Video saved as: Organ_5Relays_Handover_OK.mp4");

if delivered
    title("✅ Organ Delivered Successfully (5 Relays + Handover)");
else
    title("❌ Deadline Missed");
end

%% ================== FUNCTIONS ==================
function [bestRelay, bestScore] = selectBestRelay(Tpos, base, RelayPos, baseRange, relayRange)
    n = size(RelayPos,1);
    score = zeros(n,1);
    for i=1:n
        dTR = norm(Tpos - RelayPos(i,:));
        dRB = norm(RelayPos(i,:) - base);
        trOk = (dTR <= relayRange);
        rbOk = (dRB <= baseRange);
        score(i) = 0.6*(trOk*(1 - dTR/relayRange)) + 0.4*(rbOk*(1 - dRB/baseRange));
    end
    [bestScore, bestRelay] = max(score);
    if bestScore <= 0
        bestRelay = 1;
        bestScore = 0;
    end
end

function P = path_AstarStyle(start,goal,noFly)
    P = [linspace(start(1),goal(1),260)' linspace(start(2),goal(2),260)'];
    P = avoidNoFly(P,noFly);
end

function P = path_RRTStyle(start,goal,noFly)
    mid = start + 0.5*(goal-start) + [15*randn 15*randn];
    P = [[linspace(start(1),mid(1),130)' linspace(start(2),mid(2),130)']; ...
         [linspace(mid(1),goal(1),130)' linspace(mid(2),goal(2),130)']];
    P = avoidNoFly(P,noFly);
end

function P = path_Optimized(start,goal,noFly)
    wp1 = start + 0.3*(goal-start) + [10 -7];
    wp2 = start + 0.7*(goal-start) + [-8 12];
    P = [[linspace(start(1),wp1(1),80)' linspace(start(2),wp1(2),80)']; ...
         [linspace(wp1(1),wp2(1),90)' linspace(wp1(2),wp2(2),90)']; ...
         [linspace(wp2(1),goal(1),80)' linspace(wp2(2),goal(2),80)']];
    P = avoidNoFly(P,noFly);
end

function P = avoidNoFly(P,noFly)
    for i=1:size(noFly,1)
        cx=noFly(i,1); cy=noFly(i,2); r=noFly(i,3);
        for k=1:size(P,1)
            d=norm(P(k,:) - [cx cy]);
            if d<r+3
                dir=(P(k,:) - [cx cy])/(d+0.01);
                P(k,:)=[cx cy] + dir*(r+6);
            end
        end
    end
end

function R = commReliability_5Relays(P, base, RelayPos, baseRange, relayRange)
    if size(P,1)==1
        bestR = 0;
        for i=1:size(RelayPos,1)
            dTR = norm(P - RelayPos(i,:));
            dRB = norm(RelayPos(i,:) - base);
            r1 = (dTR<=relayRange)*0.9;
            r2 = (dRB<=baseRange)*0.7;
            bestR = max(bestR, min(r1,r2));
        end
        R = bestR;
    else
        R = zeros(size(P,1),1);
        for k=1:size(P,1)
            R(k) = commReliability_5Relays(P(k,:), base, RelayPos, baseRange, relayRange);
        end
    end
end

function h = drawCircle(x,y,r,fc,alphaVal)
    th = linspace(0,2*pi,100);
    X = x + r*cos(th);
    Y = y + r*sin(th);
    h = fill(X,Y,fc,'EdgeColor','r','FaceAlpha',alphaVal);
end

