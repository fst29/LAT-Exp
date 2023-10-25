close all
clear all
%% Office Mac
% Get the Raspberry Pi file
% [fileRP,pathRP] = uigetfile("*.csv")
fileRP = 'data.csv';
pathRP = '/Users/lg608/Dropbox (Cambridge University)/PhD/Experimental Work/Experimental Data/data/';

folder = '1212/1212 - Run2/';

%% Good runs
% 
%% Home Mac
fileLC = 'data_loadcell.csv';
% pathLC = '/Users/lg608/Documents/Armstrong/pos-control/';
dataRP = readtable([pathRP, folder, fileRP], 'Delimiter', ',');
dataLC = readmatrix([pathRP, folder, fileLC]);

data.RP.time = table2array(dataRP(:,1));
data.RP.targetP = table2array(dataRP(:,2));
data.RP.measP = table2array(dataRP(:,3));
% data.RP.torque = lowpass(table2array(dataRP(:,4)), 10, 100);
data.RP.torque = table2array(dataRP(:,4));
% data.RP.torque = dataRP(:,5);
data.RP.current = table2array(dataRP(:,5));
data.RP.rotRaw = table2array(dataRP(:,6));
data.RP.pinionT = table2array(dataRP(:,7));
data.RP.pinionP = table2array(dataRP(:,8))/2048*2*pi;
data.RP.pinionCurrent = table2array(dataRP(:,9));
data.RP.torqueD = table2array(dataRP(:,10))/257*4.69;
data.RP.staticPos = table2array(dataRP(:,11))/2048*2*pi;
data.RP.moved = table2array(dataRP(:,12));
data.RP.direction = table2array(dataRP(:,13));
data.RP.coneID = table2array(dataRP(:,14));
data.RP.carriageID = table2array(dataRP(:,15));
data.RP.percentOutput = table2array(dataRP(:,16));
data.RP.posControl = table2array(dataRP(:,17));
data.RP.torqueStat = table2array(dataRP(:,18))*4.69/257;

data.LC.time = dataLC(:,2)/1000;

data.LC.torque =  dataLC(:,3) - dataLC(1,3) - 0.008;
data.LC.torqueRaw = dataLC(:,3) - dataLC(1,3) + 0.0;

% data.LC.torque =  dataLC(:,3);
% data.LC.torqueRaw = dataLC(:,3);
% procData.LC = data.LC;
syncData = fSyncData_v4(data.RP, data.LC);
procData.RP = fProcessData(syncData.RP);
%% Processing encoder data
% [direc1, direc2] = fGetDirection(data.RP);
procData.LC = syncData.LC;
% procData.LC.torque = procData.LC.torque - procData.LC.torque(1);

%% Process stiction data
% moved = procData.RP.moved;
[p,moved] = findpeaks(procData.RP.moved);
procData.RP.torqueSign = sign(procData.RP.percentOutput).*procData.RP.torque;
procData.RP.torqueStatSign = sign(procData.RP.percentOutput).*procData.RP.torqueStat;
procData.RP.moved = moved(procData.RP.torque(moved)>0.09);
% moved = moved(procData.RP.torque(moved)>0.1);

%% Estimating p
x0 = 1;
options = optimoptions(@fmincon,'MaxFunctionEvaluations', 3e4);
x = fmincon(@(x)(fFindRatio_v4(x, procData)), x0, [], [], [], [], [], [], [], options);

p1 = x(1);
f = 6.689489472483746e-04; 
% f = 5.810969921762468e-04;
% f = [0,1];
procData.RP.p1 = p1;
procData.RP.p= p1 + f(1)* procData.RP.pinionP*2048/2/pi;

%% Subplot for a single carriage position
moved = procData.RP.moved;
procData.data = data;
CarriageUnique = uniquetol(data.RP.carriageID, 0.1);
for i = 1:length(CarriageUnique)
    iCarriage{i} = procData.RP.carriageID == i-1;
    time1 = procData.RP.time(iCarriage{i} & procData.RP.measVp > 0);
    torque1 = procData.RP.torqueSign(iCarriage{i});
    pos1 = procData.RP.measP(iCarriage{i});
    expLength = 1:length(procData.RP.time);
%     indexNoP = find(iCarriage{i} & procData.RP.measVp > 0);
    indexNoP = find(iCarriage{i} & procData.RP.direction > 0);
    avCarriageP(i) = mean(procData.RP.pinionP(iCarriage{i}));
    movedInstP = moved(ismember(moved, indexNoP));
    [~, index] = sort(procData.RP.measP(movedInstP));
    movedRangeP{i} = movedInstP(index);
    
    indexNoN = find(iCarriage{i} & procData.RP.direction < 0);
    avCarriageN(i) = mean(procData.RP.pinionP(iCarriage{i}));
    avP(i) = mean(procData.RP.p(iCarriage{i}));
    movedRangeN{i} = moved(ismember(moved, indexNoN));
    
    movedInstN = moved(ismember(moved, indexNoN));
    [~, index] = sort(procData.RP.measP(movedInstN));
    movedRangeN{i} = movedInstN(index);
end
    
figure
subplot(2,1,1)
Ax1 = gca;
hold on
plot(procData.RP.time, procData.RP.torqueSign, 'k')
plot(procData.RP.time, -procData.LC.torque, 'b')
plot(procData.RP.time(moved - 1), procData.RP.torqueSign(moved - 1), 'rx', 'LineWidth', 1.5)
plot(procData.RP.time(moved - 1), -procData.LC.torque(moved - 1), 'gx', 'LineWidth', 1.5)
% plot(procData.RP.time(moved2-1), procData.RP.torqueSign(moved2-1), 'rx', 'LineWidth', 1.5)
% plot(procData.RP.time(moved2-1), -procData.LC.torque(moved2-1), 'gx', 'LineWidth', 1.5)
plot(procData.RP.time, procData.RP.posControl*max([max(abs(procData.RP.torqueSign)), max(abs(procData.LC.torque))]), 'k--');
xlabel('Time, s');
ylabel('Motor Torque, Nm')
title(['Stiction Measurement'])
legend({'Torque - Motor', 'Torque - Loadcell', 'Recorded Value - Motor', 'Recorded Value - Loadcell', 'Torque / Position control switch'})
grid minor

subplot(2,1,2)
Ax2 = gca;
hold on
plot(procData.RP.time, procData.RP.measP, 'k')
plot(procData.RP.time(moved - 1), procData.RP.staticPos(moved - 1), 'ro', 'LineWidth', 1.5)
% plot(procData.RP.time(moved2-1), procData.RP.staticPos(moved2-1), 'ro', 'LineWidth', 1.5)
plot(procData.RP.time, procData.RP.posControl*max(abs(procData.RP.staticPos)), 'k--');

xlabel('Time, s');
ylabel('Cone Rotation, rad')
legend({'Position Signal', 'Recorded Value', 'Torque / Position control switch'})
grid minor
linkaxes([Ax1, Ax2], 'x');

indexP = find(procData.RP.measVp > 0);
movedP = moved(ismember(moved, indexP));
meanTorqueLCP = mean(-procData.LC.torque(movedP-1));

indexN = find(procData.RP.measVp < 0);
movedN = moved(ismember(moved, indexN));
meanTorqueLCN = mean(-procData.LC.torque(movedN-1));



legID = 1;
figure
Ax = gca;
hold on
colorRange = ['k', 'b', 'g', 'm', 'r', 'c'];
for i = 1:length(CarriageUnique)
    c = colorRange(i);
    plot(Ax, procData.RP.measP(movedRangeP{i}-1), procData.RP.torqueSign(movedRangeP{i}-1), [c, '-x'], 'MarkerSize', 8, 'LineWidth', 2)
    plot(Ax, procData.RP.measP(movedRangeP{i}-1), -procData.LC.torque(movedRangeP{i}-1), [c, '--o'], 'MarkerSize', 8, 'LineWidth', 2)
    legEntry{legID} = sprintf('Motor Torque, Carriage Position: %.2f', avCarriageP(i));
    legID = legID + 1;
    legEntry{legID} = sprintf('Loadcell Torque, Carriage Position: %.2f', avCarriageP(i));
    legID = legID + 1;
end
legend(legEntry)
xlabel('Cone Rotation, rad'); ylabel('Stiction Torque, Nm'); title('Stiction Measurement')
ylim([0, 0.35])
legID = 1;
figure
Ax = gca;
hold on
colorRange = ['k', 'b', 'g', 'm', 'r', 'c'];
offset = 0.01;
for i = 1:length(CarriageUnique)
    c = colorRange(i);
    if i == 1
        plot(Ax, procData.RP.measP(movedRangeN{i}-1), -procData.RP.torqueSign(movedRangeN{i}-1), [c, '-x'], 'MarkerSize', 8, 'LineWidth', 2)
        plot(Ax, procData.RP.measP(movedRangeN{i}-1), procData.LC.torque(movedRangeN{i}-1) - offset, [c, '--o'], 'MarkerSize', 8, 'LineWidth', 2)
    else
        plot(Ax, procData.RP.measP(movedRangeN{i}-1), -procData.RP.torqueSign(movedRangeN{i}-1), [c, '-x'], 'MarkerSize', 8, 'LineWidth', 2)
        plot(Ax, procData.RP.measP(movedRangeN{i}-1), procData.LC.torque(movedRangeN{i}-1), [c, '--o'], 'MarkerSize', 8, 'LineWidth', 2)
    end
    legEntry{legID} = sprintf('Motor Torque, Carriage Position: %.2f', avP(i));
    legID = legID + 1;
    legEntry{legID} = sprintf('Loadcell Torque, Carriage Position: %.2f', avP(i));
    legID = legID + 1;
    RotN{i} = procData.RP.measP(movedRangeN{i}-1);% for 1019 - 0.75;
    TorqueMN{i} = -procData.RP.torqueSign(movedRangeN{i}-1);
    TorqueLCN{i} = procData.LC.torque(movedRangeN{i}-1);
end
legend(legEntry)
xlabel('Cone Rotation, rad'); ylabel('Stiction Torque, Nm'); title('Stiction Measurement')
% ylim([0, max(abs(procData.LC.torque)) + 0.03])
ylim([0, 0.35])  
