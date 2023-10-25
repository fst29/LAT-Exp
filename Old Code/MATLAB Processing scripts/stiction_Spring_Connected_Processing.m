close all
clear all
%% Office Mac
% Get the Raspberry Pi file
% [fileRP,pathRP] = uigetfile("*.csv")
fileRP = 'data.csv';
pathRP = '/Users/lg608/Dropbox (Cambridge University)/PhD/Experimental Work/Experimental Data/data/';
% pathRP = 'C:\Users\aa\Dropbox (Cambridge University)\PhD\Experimental Work\Experimental Data\data\';

folder = '1104/1104 - Run14/';
%% Good runs

%% Home Mac
% fileRP = 'data.csv';
% pathRP = '/Users/lg608/Documents/Armstrong/pos-control/';

fileLC = 'data_loadcell.csv';
% pathLC = '/Users/lg608/Documents/Armstrong/pos-control/';
dataRP = readtable([pathRP, folder, fileRP], 'Delimiter', ',');
dataLC = readmatrix([pathRP, folder, fileLC]);

data.RP.time = table2array(dataRP(:,1));
data.RP.targetP = table2array(dataRP(:,2));
data.RP.measP = table2array(dataRP(:,3));
% data.RP.torque = lowpass(dataRP(:,4), 10, 100);
data.RP.torque = table2array(dataRP(:,4));
% data.RP.torque = dataRP(:,5);
data.RP.current = table2array(dataRP(:,5));
data.RP.rotRaw = table2array(dataRP(:,6));
data.RP.pinionT = table2array(dataRP(:,7));
data.RP.pinionP = table2array(dataRP(:,8))/2048*2*pi;
% data.RP.pinionP = table2array(dataRP(:,8));
data.RP.pinionCurrent = table2array(dataRP(:,9));
data.RP.torqueD = table2array(dataRP(:,10))/257*4.69;
data.RP.staticPos = table2array(dataRP(:,11))/2048*2*pi;
data.RP.movedRaw = table2array(dataRP(:,12));
data.RP.direction = table2array(dataRP(:,13));
data.RP.coneID = table2array(dataRP(:,14));
data.RP.carriageID = table2array(dataRP(:,15));
data.RP.percentOutput = table2array(dataRP(:,16));
data.RP.posControl = table2array(dataRP(:,17));
data.RP.torqueStat = table2array(dataRP(:,18))*4.69/257;

data.LC.time = dataLC(:,2)/1000;

data.LC.torque =  dataLC(:,3) - dataLC(1,3);
data.LC.torqueRaw = dataLC(:,3) - dataLC(1,3);
data.LC.torqueLP = lowpass(dataLC(:,3) - dataLC(1,3), 10, 100);
% data.LC.torque =  dataLC(:,3);
% data.LC.torqueRaw = dataLC(:,3);
% procData.LC = data.LC;
syncData = fSyncData_v2(data.RP, data.LC);
procData.RP = fProcessData(syncData.RP);
procData.LC = syncData.LC;

%% Process stiction data
% moved = procData.RP.moved;
[p,moved] = findpeaks(procData.RP.movedRaw);
procData.RP.torqueSign = sign(procData.RP.percentOutput).*procData.RP.torque;
procData.RP.torqueStatSign = sign(procData.RP.percentOutput).*procData.RP.torqueStat;
% moved = moved(procData.RP.torque(moved)>0.1);
procData.RP.moved = moved;
procData.data = data;

%% Estimating p
x0 = 1;
options = optimoptions(@fmincon,'MaxFunctionEvaluations', 3e4);
x = fmincon(@(x)(fFindRatio_v2(x, procData)), x0, [], [], [], [], [], [], [], options);

p1 = x(1);
f = 6.689489472483746e-04; 
% f = 5.810969921762468e-04;
procData.RP.p1 = p1;
procData.RP.p= p1 + f(1)* procData.RP.pinionP*2048/2/pi;

% figure
% subplot(2,1,1)
% hold on
% plot(procData.RP.time, procData.RP.measP)
% plot(procData.RP.time, procData.RP.springP.*procData.RP.p)
% 
% subplot(2,1,2)
% hold on
% plot(procData.RP.time, procData.RP.measVp)
% plot(procData.RP.time, procData.RP.angleV.*procData.RP.p)

%% Torque correction
Fpr = 0.025;
procData.LC.torque = procData.LC.torque - Fpr./procData.RP.p;

%% Subplot for a single carriage position
torqueTol = 0.1;
CarriageUnique = uniquetol(data.RP.carriageID, 0.1);
for i = 1:length(CarriageUnique)
    iCarriage{i} = procData.RP.carriageID == i-1;
%     indexNoP = find(iCarriage{i} & procData.RP.measVp > 0);

%     indexNoP = find(iCarriage{i} & ((sign(procData.RP.percentOutput) > 0 & procData.RP.measP > 0)|(sign(procData.RP.percentOutput) < 0 & procData.RP.measP < 0)));
    indexNoP = find(iCarriage{i} & ((procData.RP.direction > 0 & procData.RP.measP > 0)|(procData.RP.direction < 0 & procData.RP.measP < 0)));
    avCarriage(i) = mean(procData.RP.pinionP(iCarriage{i}));
%     movedRangeP{i} = moved(ismember(moved, indexNoP) & abs(procData.LC.torqueLP(moved-1))>torqueTol);
    
    movedRangeP{i} = moved(ismember(moved, indexNoP) & abs(procData.LC.torque(moved-1))>torqueTol);
%     movedRangeP{i} = moved(ismember(moved, indexNoP));
    
%     indexNoP1 = find(iCarriage{i} & (sign(procData.RP.percentOutput) > 0 & procData.RP.measP > 0));
    indexNoP1 = find(iCarriage{i} & (procData.RP.direction > 0 & procData.RP.measP > 0));
%     movedRangeP1{i} = moved(ismember(moved, indexNoP1) & abs(procData.LC.torqueLP(moved-1))>torqueTol);
    
    movedRangeP1{i} = moved(ismember(moved, indexNoP1) & abs(procData.LC.torque(moved-1))>torqueTol);
%     movedRangeP1{i} = moved(ismember(moved, indexNoP1));
    
%     indexNoP2 = find(iCarriage{i} & (sign(procData.RP.percentOutput) < 0 & procData.RP.measP < 0));
    indexNoP2 = find(iCarriage{i} & (procData.RP.direction < 0 & procData.RP.measP < 0));
%     movedRangeP2{i} = moved(ismember(moved, indexNoP2) & abs(procData.LC.torqueLP(moved-1))>torqueTol);
    
    movedRangeP2{i} = moved(ismember(moved, indexNoP2) & abs(procData.LC.torque(moved-1))>torqueTol);
%     movedRangeP2{i} = moved(ismember(moved, indexNoP2));
    
%     indexNoN = find(iCarriage{i} & ((sign(procData.RP.percentOutput) > 0 & procData.RP.measP < 0)|(sign(procData.RP.percentOutput) < 0 & procData.RP.measP > 0)));
    indexNoN = find(iCarriage{i} & ((procData.RP.direction > 0 & procData.RP.measP < 0)|(procData.RP.direction < 0 & procData.RP.measP > 0)));
%     movedRangeN{i} = moved(ismember(moved, indexNoN) & abs(procData.LC.torqueLP(moved-1))>torqueTol);
    
    movedRangeN{i} = moved(ismember(moved, indexNoN));
%     movedRangeN{i} = moved(ismember(moved, indexNoN));
    
%     indexNoN1 = find(iCarriage{i} & (sign(procData.RP.percentOutput) > 0 & procData.RP.measP < 0));
    indexNoN1 = find(iCarriage{i} & (procData.RP.direction > 0 & procData.RP.measP < 0));
%     movedRangeN1{i} = moved(ismember(moved, indexNoN1) & abs(procData.LC.torqueLP(moved-1))>torqueTol);
    
    movedRangeN1{i} = moved(ismember(moved, indexNoN1));
%     movedRangeN1{i} = moved(ismember(moved, indexNoN1));
    
%     indexNoN2 = find(iCarriage{i} & (sign(procData.RP.percentOutput) < 0 & procData.RP.measP > 0));
    indexNoN2 = find(iCarriage{i} & (procData.RP.direction < 0 & procData.RP.measP > 0));
%     movedRangeN2{i} = moved(ismember(moved, indexNoN2) & abs(procData.LC.torqueLP(moved-1))>torqueTol);
    
    movedRangeN2{i} = moved(ismember(moved, indexNoN2));

%     movedRangeN2{i} = moved(ismember(moved, indexNoN2));
    
end
    
    
% Zoomed in position Plot
figure
subplot(3,1,1)
Ax1 = gca;
hold on
plot(procData.RP.time, procData.RP.torqueSign, 'k')
plot(procData.RP.time, -procData.LC.torque, 'b')
% plot(procData.RP.time, -procData.LC.torqueLP, 'b')
plot(procData.RP.time(moved - 1), procData.RP.torqueSign(moved - 1), 'rx', 'LineWidth', 1.5)
plot(procData.RP.time(moved - 1), -procData.LC.torque(moved - 1), 'gx', 'LineWidth', 1.5)
% plot(procData.RP.time(moved - 1), -procData.LC.torqueLP(moved - 1), 'gx', 'LineWidth', 1.5)
plot(procData.RP.time, procData.RP.posControl*max([max(abs(procData.RP.torqueSign)), max(abs(procData.LC.torque))]), 'k--');
xlabel('Time, s');
ylabel('Motor Torque, Nm')
title(['Stiction Measurement'])
legend({'Torque - Motor', 'Torque - Loadcell', 'Recorded Value - Motor', 'Recorded Value - Loadcell', 'Torque / Position control switch'})
grid minor

subplot(3,1,2)
Ax2 = gca;
hold on
plot(procData.RP.time, procData.RP.measP, 'k')
plot(procData.RP.time(moved - 1), procData.RP.staticPos(moved - 1), 'ro', 'LineWidth', 1.5)
plot(procData.RP.time, procData.RP.springP, 'b')
plot(procData.RP.time(moved - 1), procData.RP.springP(moved - 10), 'go', 'LineWidth', 1.5)
plot(procData.RP.time, procData.RP.posControl*max(abs(procData.RP.staticPos)), 'k--');
ylabel('Cone Rotation, rad')
legend({'Motor Rotation', 'Recorded Value - Motor rot', 'Spring Rotation', 'Recorded Value - Spring', 'Torque / Position control switch'})
grid minor

subplot(3,1,3)
Ax3 = gca;
hold on
plot(procData.RP.time, procData.RP.p, 'b', 'LineWidth', 3)
plot(procData.RP.time, procData.RP.posControl*max(abs(procData.RP.p)), 'k--');
xlabel('Time, s')
ylabel('p')
legend({'Carriage Position', 'Torque / Position control switch'})
grid minor
linkaxes([Ax1, Ax2, Ax3], 'x');
ylim([min(procData.RP.p) - 0.1, max(procData.RP.p) + 0.1])

indexP = find(procData.RP.measVp > 0);
movedP = moved(ismember(moved, indexP));
meanTorqueLCP = mean(-procData.LC.torque(movedP-1));

indexN = find(procData.RP.measVp < 0);
movedN = moved(ismember(moved, indexN));
meanTorqueLCN = mean(-procData.LC.torque(movedN-1));

legID = 1;
figure
subplot(1,2,1)
AxP = gca;
hold on

subplot(1,2,2)
AxN = gca;
hold on

legSpringID = 1;
figure
subplot(1,2,1)
AxSpringP = gca;
hold on

subplot(1,2,2)
AxSpringN = gca;
hold on

legStictionID = 1;
figure
subplot(1,2,1)
AxStictionP = gca;
hold on

subplot(1,2,2)
AxStictionN = gca;
hold on

colorRange = ['k', 'b', 'g', 'm', 'r', 'c'];
posVecP1 = [];
posVecP2 = [];
posVecN1 = [];
posVecN2 = [];

torVecP1 = [];
torVecP2 = [];
torVecN1 = [];
torVecN2 = [];

pVecP1 = [];
pVecP2 = [];
pVecN1 = [];
pVecN2 = [];

for i = 1:length(CarriageUnique)
    c = colorRange(i);
    
    plot(AxP, procData.RP.staticPos(movedRangeP{i}-1), procData.RP.torqueSign(movedRangeP{i}-1), [c, 'x'], 'MarkerSize', 8, 'LineWidth', 2)       
    plot(AxP, procData.RP.staticPos(movedRangeP{i}-1), -procData.LC.torque(movedRangeP{i}-1), [c, 'o'], 'MarkerSize', 8, 'LineWidth', 2)    
    
    plot(AxN, procData.RP.staticPos(movedRangeN{i}-1), procData.RP.torqueSign(movedRangeN{i}-1), [c, 'x'], 'MarkerSize', 8, 'LineWidth', 2)       
    plot(AxN, procData.RP.staticPos(movedRangeN{i}-1), -procData.LC.torque(movedRangeN{i}-1), [c, 'o'], 'MarkerSize', 8, 'LineWidth', 2)    

%     plot(Ax, procData.RP.staticPos(movedRangeP{i}-1), abs(procData.LC.torqueLP(movedRangeP{i}-1)), [c, 'o'], 'MarkerSize', 8, 'LineWidth', 2)    
    
    % Winding up
    p = procData.RP.p(movedRangeP1{i}-1);
    posVecP1 = [posVecP1; procData.RP.staticPos(movedRangeP1{i}-1)];
    torVecP1 = [torVecP1; -procData.LC.torque(movedRangeP1{i}-1)];
    pVecP1 = [pVecP1; p];
    
    p = procData.RP.p(movedRangeP2{i}-1);
    posVecP2 = [posVecP2; procData.RP.staticPos(movedRangeP2{i}-1)];
    torVecP2 = [torVecP2; -procData.LC.torque(movedRangeP2{i}-1)];
    pVecP2 = [pVecP2; p];
    
    %Winding down
    p = procData.RP.p(movedRangeN1{i}-1);
    posVecN1 = [posVecN1; procData.RP.staticPos(movedRangeN1{i}-1)];
    torVecN1 = [torVecN1; -procData.LC.torque(movedRangeN1{i}-1)];
    pVecN1 = [pVecN1; p];
    
    p = procData.RP.p(movedRangeN2{i}-1);
    posVecN2 = [posVecN2; procData.RP.staticPos(movedRangeN2{i}-1)];
    torVecN2 = [torVecN2; -procData.LC.torque(movedRangeN2{i}-1)];
    pVecN2 = [pVecN2; p];
    
    %Winding up
    fTP1{i} = fit(procData.RP.staticPos(movedRangeP1{i}-1), -procData.LC.torque(movedRangeP1{i}-1), 'poly1');
    fTP2{i} = fit(procData.RP.staticPos(movedRangeP2{i}-1), -procData.LC.torque(movedRangeP2{i}-1), 'poly1');
    %Winding down
    fTN1{i} = fit(procData.RP.staticPos(movedRangeN1{i}-1), -procData.LC.torque(movedRangeN1{i}-1), 'poly1');
    fTN2{i} = fit(procData.RP.staticPos(movedRangeN2{i}-1), -procData.LC.torque(movedRangeN2{i}-1), 'poly1');

    slopeTP1(i) = fTP1{i}.p1;
    slopeTP2(i) = fTP2{i}.p1;

    slopeTN1(i) = fTN1{i}.p1;
    slopeTN2(i) = fTN2{i}.p1;

    offsetTP1(i) = fTP1{i}.p2;
    offsetTP2(i) = fTP2{i}.p2;
 
    offsetTN1(i) = fTN1{i}.p2;
    offsetTN2(i) = fTN2{i}.p2;
    
    pRangeP1 = linspace(min(procData.RP.staticPos(movedRangeP1{i}-1)), max(procData.RP.staticPos(movedRangeP1{i}-1)), 10);
    pRangeP2 = linspace(min(procData.RP.staticPos(movedRangeP2{i}-1)), max(procData.RP.staticPos(movedRangeP2{i}-1)), 10);
    
    pRangeN1 = linspace(min(procData.RP.staticPos(movedRangeN1{i}-1)), max(procData.RP.staticPos(movedRangeN1{i}-1)), 10);
    pRangeN2 = linspace(min(procData.RP.staticPos(movedRangeN2{i}-1)), max(procData.RP.staticPos(movedRangeN2{i}-1)), 10);
    
    plot(AxP, pRangeP1, fTP1{i}(pRangeP1), [c, '-'], 'MarkerSize', 8, 'LineWidth', 2)
    plot(AxP, pRangeP2, fTP2{i}(pRangeP2), [c, '--'], 'MarkerSize', 8, 'LineWidth', 2)
    
    plot(AxN, pRangeN1, fTN1{i}(pRangeN1), [c, '-'], 'MarkerSize', 8, 'LineWidth', 2)
    plot(AxN, pRangeN2, fTN2{i}(pRangeN2), [c, '--'], 'MarkerSize', 8, 'LineWidth', 2)

    plot(AxSpringP, procData.RP.measP(movedRangeP{i}-10), procData.RP.springP(movedRangeP{i}-10), [c, 'o'], 'MarkerSize', 8, 'LineWidth', 2)
    
    plot(AxSpringN, procData.RP.measP(movedRangeN{i}-10), procData.RP.springP(movedRangeN{i}-10), [c, 'o'], 'MarkerSize', 8, 'LineWidth', 2)

    % Fitting linear curve to spring data
    fSP1{i} = fit(procData.RP.staticPos(movedRangeP1{i}-10), procData.RP.springP(movedRangeP1{i}-10), 'poly1');
    fSP2{i} = fit(procData.RP.staticPos(movedRangeP2{i}-10), procData.RP.springP(movedRangeP2{i}-10), 'poly1');
    
    fSN1{i} = fit(procData.RP.staticPos(movedRangeN1{i}-10), procData.RP.springP(movedRangeN1{i}-10), 'poly1');
    fSN2{i} = fit(procData.RP.staticPos(movedRangeN2{i}-10), procData.RP.springP(movedRangeN2{i}-10), 'poly1');
    
    
    slopeSP1(i) = fSP1{i}.p1;
    slopeSP2(i) = fSP2{i}.p1;
    
    slopeSN1(i) = fSN1{i}.p1;
    slopeSN2(i) = fSN2{i}.p1;
    
    plot(AxSpringP, pRangeP1, fSP1{i}(pRangeP1), [c, '-'], 'MarkerSize', 8, 'LineWidth', 2)
    plot(AxSpringP, pRangeP2, fSP2{i}(pRangeP2), [c, '--'], 'MarkerSize', 8, 'LineWidth', 2)
    
    plot(AxSpringN, pRangeN1, fSN1{i}(pRangeN1), [c, '-'], 'MarkerSize', 8, 'LineWidth', 2)
    plot(AxSpringN, pRangeN2, fSN2{i}(pRangeN2), [c, '--'], 'MarkerSize', 8, 'LineWidth', 2)
    
    %Estimating stiction from data
    k = 0.52;
    conePosP{i} = [flip(procData.RP.staticPos(movedRangeP2{i}-1)); procData.RP.staticPos(movedRangeP1{i}-1)];
    totalForceP{i} = [flip(-procData.LC.torque(movedRangeP2{i}-1)); -procData.LC.torque(movedRangeP1{i}-1)];
    
    % Data structures for Latex
    motRotP{i} = procData.RP.staticPos(movedRangeP{i}-1);
    motRotN{i} = procData.RP.staticPos(movedRangeN{i}-1);
    torP{i} = -procData.LC.torque(movedRangeP{i}-1);
    torN{i} = -procData.LC.torque(movedRangeN{i}-1);

    motRotVecP1(i, :) = pRangeP1;
    motRotVecP2(i, :) = pRangeP2;
    motRotVecN1(i, :) = pRangeN1;
    motRotVecN2(i, :) = pRangeN2;

    torFitP1(i, :) = fTP1{i}(pRangeP1)';
    torFitP2(i, :) = fTP2{i}(pRangeP2)';
    torFitN1(i, :) = fTN1{i}(pRangeN1)';
    torFitN2(i, :) = fTN2{i}(pRangeN2)';

%     totalForce{i} = [flip(abs(procData.LC.torqueLP(movedRangeP2{i}-1))); abs(procData.LC.torqueLP(movedRangeP1{i}-1))];
    springForceP{i} = k*[fSP2{i}(flip(procData.RP.staticPos(movedRangeP2{i}-1))); fSP1{i}(procData.RP.staticPos(movedRangeP1{i}-1))];
    stictionP{i} = totalForceP{i} - springForceP{i};
    
    % Unwinding
    conePosN{i} = [procData.RP.staticPos(movedRangeN1{i}-1); flip(procData.RP.staticPos(movedRangeN2{i}-1))];
    totalForceN{i} = [-procData.LC.torque(movedRangeN1{i}-1); flip(-procData.LC.torque(movedRangeN2{i}-1))];
    
    springForceN{i} = k*[fSN2{i}(procData.RP.staticPos(movedRangeN1{i}-1)); fSN1{i}(flip(procData.RP.staticPos(movedRangeN2{i}-1)))];
    stictionN{i} = totalForceN{i} - springForceN{i};
    
    plot(AxStictionP, conePosP{i}, stictionP{i}, [c, '-x'], 'MarkerSize', 8, 'LineWidth', 2)
    plot(AxStictionP, conePosP{i}, totalForceP{i}, [c, '--o'], 'MarkerSize', 8, 'LineWidth', 2)
    plot(AxStictionP, conePosP{i}, springForceP{i}, [c, '-.*'], 'MarkerSize', 8, 'LineWidth', 2)
    
    plot(AxStictionN, conePosN{i}, stictionN{i}, [c, '-x'], 'MarkerSize', 8, 'LineWidth', 2)
    plot(AxStictionN, conePosN{i}, totalForceN{i}, [c, '--o'], 'MarkerSize', 8, 'LineWidth', 2)
    plot(AxStictionN, conePosN{i}, springForceN{i}, [c, '-.*'], 'MarkerSize', 8, 'LineWidth', 2)
    
    % Legend data
    legEntryP{legID} = sprintf('Motor Torque, Carriage Position: %.2f', avCarriage(i));
    legEntryN{legID} = sprintf('Motor Torque, Carriage Position: %.2f', avCarriage(i));
    legID = legID + 1;
    legEntryP{legID} = sprintf('Loadcell Torque, Carriage Position: %.2f', avCarriage(i));
    legEntryN{legID} = sprintf('Loadcell Torque, Carriage Position: %.2f', avCarriage(i));
    legID = legID + 1;
    legEntryP{legID} = sprintf('Fit for Carriage Position: %.2f, Slope: %.2f', avCarriage(i), fTP1{i}.p1);
    legEntryN{legID} = sprintf('Fit for Carriage Position: %.2f, Slope: %.2f', avCarriage(i), fTN1{i}.p1);
    legID = legID + 1;
    legEntryP{legID} = sprintf('Fit for Carriage Position: %.2f, (Negative) Slope: %.2f', avCarriage(i), -fTP2{i}.p1);
    legEntryN{legID} = sprintf('Fit for Carriage Position: %.2f, (Negative) Slope: %.2f', avCarriage(i), -fTN2{i}.p1);
    legID = legID + 1;
    legSpringEntryP{legSpringID} = sprintf('Carriage Position: %.2f', avCarriage(i));
    legSpringEntryN{legSpringID} = sprintf('Carriage Position: %.2f', avCarriage(i));
    legSpringID = legSpringID + 1;
    legSpringEntryP{legSpringID} = sprintf('Carriage Position: %.2f, Slope: %.2f', avCarriage(i), fSP1{i}.p1);
    legSpringEntryN{legSpringID} = sprintf('Carriage Position: %.2f, Slope: %.2f', avCarriage(i), fSN1{i}.p1);
    legSpringID = legSpringID + 1;
    legSpringEntryP{legSpringID} = sprintf('Carriage Position: %.2f, (Negative) Slope: %.2f', avCarriage(i), -fSP2{i}.p1);
    legSpringEntryN{legSpringID} = sprintf('Carriage Position: %.2f, (Negative) Slope: %.2f', avCarriage(i), -fSN2{i}.p1);
    legSpringID = legSpringID + 1;
    legStictionEntryP{legStictionID} = sprintf('Stiction, Carriage Position: %.2f', avCarriage(i));
    legStictionEntryN{legStictionID} = sprintf('Stiction, Carriage Position: %.2f', avCarriage(i));
    legStictionID = legStictionID+1;
    legStictionEntryP{legStictionID} = sprintf('Total Torque, Carriage Position: %.2f', avCarriage(i));
    legStictionEntryN{legStictionID} = sprintf('Total Torque, Carriage Position: %.2f', avCarriage(i));
    legStictionID = legStictionID+1;
    legStictionEntryP{legStictionID} = sprintf('Spring Force, Carriage Position: %.2f', avCarriage(i));
    legStictionEntryN{legStictionID} = sprintf('Spring Force, Carriage Position: %.2f', avCarriage(i));
    legStictionID = legStictionID+1;
   
end

% plot(Ax, procData.RP.measP(moved - 1), abs(procData.LC.torque(moved - 1)), 'r*')
legend(AxP, legEntryP)
xlabel(AxP, 'Cone Rotation, rad'); ylabel(AxP, 'Stiction Torque, Nm'); title(AxP, 'Stiction Measurement - Winding Up')
% ylim([0, max(abs(procData.LC.torque)) + 0.03])
% ylim([-0.6, 0.6])    

legend(AxSpringP, legSpringEntryP)
xlabel(AxSpringP, 'Cone Rotation, rad'); ylabel(AxSpringP, 'Spring Rotation, rad'); title(AxSpringP, 'Spring Compression - Winding Up')
% ylim([0, max(abs(procData.LC.torque)) + 0.03])
% ylim([-0.5, 0.5]) 

legend(AxStictionP, legStictionEntryP)
xlabel(AxStictionP, 'Cone Rotation, rad'); ylabel(AxStictionP, 'Torque, Nm'); title(AxStictionP, 'Stiction Estimation - Winding Up')

legend(AxN, legEntryN)
xlabel(AxN, 'Cone Rotation, rad'); ylabel(AxN, 'Stiction Torque, Nm'); title(AxN, 'Stiction Measurement - Winding Down')
% ylim([0, max(abs(procData.LC.torque)) + 0.03])
% ylim([-0.6, 0.6])    

legend(AxSpringN, legSpringEntryN)
xlabel(AxSpringN, 'Cone Rotation, rad'); ylabel(AxSpringN, 'Spring Rotation, rad'); title(AxSpringN, 'Spring Compression - Winding Down')
% ylim([0, max(abs(procData.LC.torque)) + 0.03])
% ylim([-0.5, 0.5]) 

legend(AxStictionN, legStictionEntryN)
xlabel(AxStictionN, 'Cone Rotation, rad'); ylabel(AxStictionN, 'Torque, Nm'); title(AxStictionN, 'Stiction Estimation - Winding Down')

%% Fitting a custom model
% Fitting a custom model
model = fittype(@(p1, p2, p3, p4, p, x) p1*x + p2 + p3*p.*x + p4*p, 'problem', 'p', 'independent', 'x', 'dependent' ,'y');
% model = fittype(@( p2, p3, p4, p, x) p2 + p3*p.*x + p4*p, 'problem', 'p', 'independent', 'x', 'dependent' ,'y');


fCustP1 = fit(posVecP1, torVecP1, model, 'problem', pVecP1, 'StartPoint', [0.6, 0.15, -0.3, -0.05]);
fCustP2 = fit(posVecP2, torVecP2, model, 'problem', pVecP2, 'StartPoint', [0.6, -0.15, -0.3, 0.05]);
fCustN1 = fit(posVecN1, torVecN1, model, 'problem', pVecN1, 'StartPoint', [0.6, 0.15, -0.3, -0.05]);
fCustN2 = fit(posVecN2, torVecN2, model, 'problem', pVecN2, 'StartPoint', [0.6, -0.15, -0.3, 0.05]);

p = p1 + f(1)* avCarriage*2048/2/pi;

    pOffsetP1 = fCustP1.p2 + p*(fCustP1.p4);
    pOffsetP2 = fCustP2.p2 + p*(fCustP2.p4);
    pOffsetN1 = fCustN1.p2 + p*(fCustN1.p4);
    pOffsetN2 = fCustN2.p2 + p*(fCustN2.p4);
    
    pSlopeP1 = (fCustP1.p1) + p*(fCustP1.p3);
    pSlopeP2 = fCustP2.p1 + p*fCustP2.p3;
    pSlopeN1 = fCustN1.p1 + p*(fCustN1.p3);
    pSlopeN2 = fCustN2.p1 + p*fCustN2.p3;

%%
  k = 0.52;
% k = 1;
  
fTCP1 = fit(p', slopeTP1', 'poly1');
fTCP2 = fit(p', slopeTP2', 'poly1');

fSCP1 = fit(p', k*slopeSP1', 'poly1');
fSCP2 = fit(p', k*slopeSP2', 'poly1');

fTCN1 = fit(p', slopeTN1', 'poly1');
fTCN2 = fit(p', slopeTN2', 'poly1');

fSCN1 = fit(p', k*slopeSN1', 'poly1');
fSCN2 = fit(p', k*slopeSN2', 'poly1');

fPP1 = fit(p', pSlopeP1', 'poly1');
fPP2 = fit(p', pSlopeP2', 'poly1');
fPN1 = fit(p', pSlopeN1', 'poly1');
fPN2 = fit(p', pSlopeN2', 'poly1');

figure
hold on
plot(p, slopeTP1, 'kx', 'MarkerSize', 8, 'LineWidth', 2)
plot(p, slopeTP2, 'ko', 'MarkerSize', 8, 'LineWidth', 2)
plot(p, slopeTN1, 'ksquare', 'MarkerSize', 8, 'LineWidth', 2)
plot(p, slopeTN2, 'k*', 'MarkerSize', 8, 'LineWidth', 2)

plot(p, fTCP1(p), 'k', 'MarkerSize', 8, 'LineWidth', 2)
plot(p, fTCP2(p), '--k', 'MarkerSize', 8, 'LineWidth', 2)
plot(p, fTCN1(p), ':k', 'MarkerSize', 8, 'LineWidth', 2)
plot(p, fTCN2(p), '-.k', 'MarkerSize', 8, 'LineWidth', 2)

plot(p, k*slopeSP1, 'bx', 'MarkerSize', 8, 'LineWidth', 2)
plot(p, k*slopeSP2, 'bo', 'MarkerSize', 8, 'LineWidth', 2)
plot(p, k*slopeSN1, 'bsquare', 'MarkerSize', 8, 'LineWidth', 2)
plot(p, k*slopeSN2, 'b*', 'MarkerSize', 8, 'LineWidth', 2)

plot(p, fSCP1(p), 'b', 'MarkerSize', 8, 'LineWidth', 2)
plot(p, fSCP2(p), '--b', 'MarkerSize', 8, 'LineWidth', 2)
plot(p, fSCN1(p), ':b', 'MarkerSize', 8, 'LineWidth', 2)
plot(p, fSCN2(p), '-.b', 'MarkerSize', 8, 'LineWidth', 2)

plot(p, fPP1(p), 'g', 'MarkerSize', 8, 'LineWidth', 2)
plot(p, fPP2(p), '--g', 'MarkerSize', 8, 'LineWidth', 2)
plot(p, fPN1(p), ':g', 'MarkerSize', 8, 'LineWidth', 2)
plot(p, fPN2(p), '-.g', 'MarkerSize', 8, 'LineWidth', 2)

legEntryP = {
    'Torque slope change - Winding Up';
    '(Negative) Torque slope change - Winding Up';
    'Torque slope change - Winding Down';
    '(Negative) Torque slope change - Winding Down';
    sprintf('Winding Up - Fit for torque slope change, gradient: %.2f', fTCP1.p1);
    sprintf('Winding Up - Fit for (negative) torque slope change, gradient: %.2f', fTCP2.p1);
    sprintf('Winding Down - Fit for torque slope change, gradient: %.2f', fTCN1.p1);
    sprintf('Winding Down - Fit for (negative) torque slope change, gradient: %.2f', fTCN2.p1);
    'Spring slope change - Winding Up';
    '(Negative) Spring slope change - Winding Up';
    'Spring slope change - Winding Down';
    '(Negative) Spring slope change - Winding Down';
    sprintf('Winding Up - Fit for spring slope change, gradient: %.2f', fSCP1.p1);
    sprintf('Winding Up - Fit for (negative) spring slope change, gradient: %.2f', fSCP2.p1)
    sprintf('Winding Down - Fit for spring slope change, gradient: %.2f', fSCN1.p1);
    sprintf('Winding Down - Fit for (negative) spring slope change, gradient: %.2f', fSCN2.p1);

    sprintf('Winding Up - Fit for p dependent Fit, gradient: %.2f', fPP1.p1);
    sprintf('Winding Up - Fit for (negative) p dependent Fite, gradient: %.2f', fPP2.p1);
    sprintf('Winding Down - Fit for p dependent Fit, gradient: %.2f', fPN1.p1);
    sprintf('Winding Down - Fit for (negative) p dependent Fit, gradient: %.2f', fPN2.p1);};
xlabel('p'); ylabel('Slope'); title('Slope comparison for torque and spring')
legend(legEntryP)
    
figure
hold on
% plot(avCarriage, offsetTP1, 'kx', 'MarkerSize', 8, 'LineWidth', 2)
% plot(avCarriage, -offsetTP2, 'bx', 'MarkerSize', 8, 'LineWidth', 2)
% plot(avCarriage, offsetTN1, 'ko', 'MarkerSize', 8, 'LineWidth', 2)
% plot(avCarriage, -offsetTN2, 'bo', 'MarkerSize', 8, 'LineWidth', 2)

plot(avCarriage, pOffsetP1, 'gx', 'MarkerSize', 8, 'LineWidth', 2)
plot(avCarriage, -pOffsetP2, 'mo', 'MarkerSize', 8, 'LineWidth', 2)
plot(avCarriage, pOffsetN1, 'k*', 'MarkerSize', 8, 'LineWidth', 2)
plot(avCarriage, -pOffsetN2, 'b^', 'MarkerSize', 8, 'LineWidth', 2)
xlabel('Carriage Position, rad'); ylabel('Constant Offset, Nm'); title('Fit Offset')
legend({
%     'Winding up - Positive Torque';
%     'Winding up - Negative Torque';
%     'Winding down - Positive Torque';
%     'Winding down - Negative Torque';
     sprintf('Winding up - Positive Torque. Gradient: %.2f', fCustP1.p4);
    sprintf('Winding up - Negative Torque. Gradient: %.2f', fCustP2.p4);
    sprintf('Winding down - Positive Torque. Gradient: %.2f', fCustN1.p4);
    sprintf('Winding down - Negative Torque. Gradient: %.2f', fCustN2.p4)});
grid minor
ylim([0, 0.2])
