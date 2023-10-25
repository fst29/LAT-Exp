close all
clear all
%% Office Mac
% Get the Raspberry Pi file
% [fileRP,pathRP] = uigetfile("*.csv")
fileRP = 'data.csv';
pathRP = '/Users/lg608/Dropbox (Cambridge University)/PhD/Experimental Work/Experimental Data/data/';
folder = '1114/1114 - Run3/';

Fpr = 0.02;
%% Good runs
% Get Loadcell file
% [fileLC,pathLC] = uigetfile("*.csv")
fileLC = 'data_loadcell.csv';
pathLC = pathRP;

dataRP = readtable([pathRP, folder, fileRP], 'Delimiter', ',');
dataLC = readtable([pathLC, folder, fileLC], 'Delimiter', ',');

data.RP.time = table2array(dataRP(:,1));
data.RP.targetP = table2array(dataRP(:,2));
data.RP.measP = table2array(dataRP(:,3));
% data.RP.torque = lowpass(dataRP(:,4), 13, 100);
data.RP.torque = table2array(dataRP(:,4));
% data.RP.torque = dataRP(:,5);
data.RP.current = table2array(dataRP(:,5));
data.RP.rotRaw = table2array(dataRP(:,6));
data.RP.pinionT = table2array(dataRP(:,7));
data.RP.pinionP = table2array(dataRP(:,8));
data.RP.pinionC = table2array(dataRP(:,9));
data.RP.motV = table2array(dataRP(:,10));
data.RP.currentStat = table2array(dataRP(:,11));
data.RP.perOutput = table2array(dataRP(:,12));
% data.RP.date = datetime(table2array(dataRP(:,13)), 'InputFormat', 'eee MMM d HH:mm:ss yyyy');
% 
% dateBad = char(table2array(dataLC(:,1)));
% dateMod1 = dateBad(:,[1:7, 10:end]);
% for i = 1:length(dateMod1)
%     data.LC.date(i, :) = datetime([dateMod1(i, 1:7), '20', dateMod1(i,8:end)]);
% end

data.LC.time = table2array(dataLC(:,2))/1000;
data.LC.torque = table2array(dataLC(:,3)) - table2array(dataLC(1,3));

syncData = fSyncData_v2(data.RP, data.LC);
procData.RP = fProcessData(syncData.RP);
% procData.RP = fProcessData_v3(syncData.RP);
procData.RP.torqueSign = sign(procData.RP.perOutput).*procData.RP.torque;
%% Processing encoder data
% [direc1, direc2] = fGetDirection(data.RP);
procData.LC = syncData.LC;
procData.LC.torque = lowpass(procData.LC.torque, 23, 100); 
%% Torsion in the spring
procData = fTorsion_v5(procData);

%% Sample Friction Trajectory
%% Preload force - estimated
%% Inertia
I = 0.0006; %0.0006
% F = fFrictionModelA_v2(lowpass(procData.RP.measVp, 1, 100))';
F = fFrictionModelA(lowpass(procData.RP.measVp, 1, 100))';
% F(procData.RP.targetP==0)=Fpr;
F(procData.RP.targetP==0)=0;

dFdt = diff(F)/0.01;
const.Tdot = lowpass([dFdt(1); dFdt], 10, 100);
const.Torque = F;
procData.RP.Tf = F;
%% Simulation of Varspring Law with Torqueconst.p = procData.RP.p;
const.time = procData.RP.time;
const.u = procData.RP.measVp;
const.k = procData.RP.k;
const.p = procData.RP.p;
tSpan = procData.RP.time;
x0 =  0;
[t,xTorque] = ode45(@(t,x)fVarspringTorqueODE(t, x, const), tSpan, x0);
pFSimTorque = xTorque;
procData.RP.pFSimTorque = pFSimTorque;
%% Experimental Varpsring Law
dTdt = diff(procData.LC.torque)/0.01;
procData.LC.TDot = lowpass([dTdt(1); dTdt], 10, 100);
%% Plots
figure
hold on
plot(procData.RP.time, procData.RP.torque)
plot(procData.RP.time, abs(procData.LC.torque))
xlabel('Time, s')
ylabel('Torque, Nm')
legend({'Raspberry Pi', 'Loadcell'})

w = -8:0.01:8;
Ffric = fFrictionModelA(w);
figure
hold on
plot(w, Ffric, 'LineWidth', 3)
xlabel('\omega')
ylabel('Friction, Nm')
title('Friction Model')
%% Power loss calculations
% Fpr = -0.1;
Ptot1 = (lowpass(procData.RP.Tf, 5, 100) + I*procData.RP.measA).*procData.RP.measVp - (procData.RP.Traw - Fpr - I*procData.RP.angleA).*procData.RP.angleV;
Ptot2 = (I*procData.RP.measA).*procData.RP.measVp - (procData.RP.Traw - Fpr - I*procData.RP.angleA).*procData.RP.angleV;
Supply = procData.LC.torque.*procData.RP.measVp;
Internal = (-I*procData.RP.angleA).*procData.RP.angleV - (lowpass(procData.RP.Tf, 5, 100) + I*procData.RP.measA).*procData.RP.measVp;

figure
hold on
plot(procData.RP.time, -cumtrapz(procData.RP.time, (Supply)))
plot(procData.RP.time, -cumtrapz(procData.RP.time, (Internal)) - cumtrapz(procData.RP.springP, (procData.RP.Traw - Fpr)))
plot(procData.RP.time, -cumtrapz(procData.RP.time, (Supply)) - cumtrapz(procData.RP.time, lowpass(procData.RP.Tf, 5, 100).*procData.RP.measVp))

plot(procData.RP.time, -cumtrapz(procData.RP.time, -lowpass(procData.RP.Tf, 5, 100).*procData.RP.measVp))
plot(procData.RP.time, -cumtrapz(procData.RP.springP, (procData.RP.Traw - Fpr)))
plot(procData.RP.time, -cumtrapz(procData.RP.time, -(I*procData.RP.measA).*procData.RP.measVp-(I*procData.RP.angleA).*procData.RP.angleV));
legend({'Supply', 'Internal','Supply - Friction', 'Friction', 'Spring + Preload', 'Inertia'})

%% Power - Version 2
figure
subplot(3,1,1)
Ax1 = gca;
hold on
plot(procData.RP.time, (procData.LC.torque).*procData.RP.measVp, 'LineWidth', 3)
plot(procData.RP.time, (procData.RP.Traw - Fpr - I*procData.RP.angleA).*procData.RP.angleV - (lowpass(procData.RP.Tf, 5, 100) + I*procData.RP.measA).*procData.RP.measVp, 'LineWidth', 3)
ylabel('Power, W')
legend({'External', 'Internal'})
title('Instantaneous Power')
grid minor

subplot(3,1,2)
hold on
Ax2 = gca;
plot(procData.RP.time, (procData.LC.torque), 'LineWidth', 3)
plot(procData.RP.time, procData.RP.Traw - Fpr - I*procData.RP.angleA - lowpass(procData.RP.Tf, 5, 100) - I*procData.RP.measA, 'LineWidth', 3)
ylabel('Torques, Nm')
legend({'External', 'Internal'})

subplot(3,1,3)
hold on
Ax3 = gca;
plot(procData.RP.time, procData.RP.measVp, 'LineWidth', 3)
plot(procData.RP.time, procData.RP.angleV, 'LineWidth', 3)
ylabel('Velocities, rad/s')
xlabel('Time, s')
legend({'External', 'Internal'})
linkaxes([Ax1, Ax2, Ax3], 'x')

%% Other plots
figure
hold on
subplot(3,1,1)
hold on
plot(procData.RP.time, procData.RP.measVp, 'LineWidth', 3)
plot(procData.RP.time, procData.RP.angleV, 'LineWidth', 3)
plot(procData.RP.time, procData.RP.angleV.*procData.RP.p, 'LineWidth', 3);
xlabel('Time,s'); ylabel('Rotational velocity, rad/s')
legend({'Motor Shaft', 'Sprung Shaft', 'p \times Sprung Shaft'})
title('Run Trajectory')
grid minor

subplot(3,1,2)
hold on
plot(procData.RP.time, procData.RP.measP, 'LineWidth', 3)
plot(procData.RP.time, procData.RP.measPp, 'LineWidth', 3)
plot(procData.RP.time, procData.RP.springP, 'LineWidth', 3)
plot(procData.RP.time, procData.RP.pPos, 'LineWidth', 3)
xlabel('Time,s'); ylabel('Rotation, rad')
legend({'Driving Port', 'Driving Port - Integrated', 'Driven Port', 'p \times Driven Port'})
title('Run Trajectory')
grid minor


subplot(3,1,3)
hold on
plot(procData.RP.time, procData.RP.p, 'LineWidth', 3)
xlabel('Time,s'); ylabel('p')
title('Change of "p" value during the experiment')
grid minor

Inertia = I*procData.RP.measA;
modelLP = procData.RP.Traw - Fpr - lowpass(procData.RP.Tf.*procData.RP.p, 5, 100) - I*procData.RP.measA.*procData.RP.p - I*procData.RP.angleA.*procData.RP.p.^2;

figure
subplot(3,1, [1 2])
hold on
Ax1 = gca;
% p3 = plot(procData.RP.time, model, 'LineWidth', 3);
p1 = plot(procData.RP.time, procData.LC.torque.*procData.RP.p, 'LineWidth', 3);
p2 = plot(procData.RP.time, modelLP, 'LineWidth', 3);
% plot(procData.RP.time, -pFSimTorque - lowpass(const.Torque, 5, 100)+Fpr, 'LineWidth', 3);
xlabel('Time,s'); ylabel('Torque')
legend([p1, p2],{'Experiment', 'Model'})
title('Varspring Device Law')
grid minor

subplot(3,1,3)
hold on
Ax2 = gca;
plot(procData.RP.time, procData.RP.p, 'LineWidth', 3)
xlabel('Time,s'); ylabel('Rotation, rad')
title('Change of "p" value during the experiment')
grid minor
linkaxes([Ax1, Ax2], 'x');

figure
hold on
plot(procData.RP.time, procData.RP.Traw - Fpr, 'LineWidth', 2)
% plot(procData.RP.time, -lowpass(procData.RP.Tf.*procData.RP.p, 15, 100), 'LineWidth', 2)
plot(procData.RP.time, -procData.RP.Tf.*procData.RP.p, 'LineWidth', 2)
plot(procData.RP.time, -I*procData.RP.measA.*procData.RP.p - I*procData.RP.angleA.*procData.RP.p.^2, 'LineWidth', 2)
plot(procData.RP.time, procData.LC.torque.*procData.RP.p, 'LineWidth', 2)
xlabel('Time,s')
ylabel('Torque')
legend({'Device Law', 'Rolling resistance', 'Inertia', 'p \times Loadcell Torque'})
title('Contributions from entries in the model')

figure
subplot(2,2,1)
hold on
plot(procData.RP.time, procData.LC.torque.*procData.RP.p, 'LineWidth', 2)
plot(procData.RP.time, procData.RP.Traw, 'LineWidth', 2)
xlabel('Time, s');
ylabel('Torque, Nm');
legend({'p \times Loadcell',...
    'Model'})
title('Spring Torque')
subplot(2,2,2)
hold on
plot(procData.RP.time, procData.LC.torque.*procData.RP.p, 'LineWidth', 2)
grid minor

plot(procData.RP.time, procData.RP.Traw - Fpr, 'LineWidth', 2)
xlabel('Time, s');
ylabel('Torque, Nm');
legend({'p \times Loadcell',...
    'Model'})
title(sprintf('Spring Torque + Preload of %0.2f Nm', Fpr))
grid minor

subplot(2,2,3)
hold on
plot(procData.RP.time, procData.LC.torque.*procData.RP.p, 'LineWidth', 2)
plot(procData.RP.time, procData.RP.Traw - Fpr - I*procData.RP.measA.*procData.RP.p - I*procData.RP.angleA.*procData.RP.p.^2, 'LineWidth', 2)
xlabel('Time, s');
ylabel('Torque, Nm');
legend({'p \times Loadcell',...
    'Model'})
title(sprintf('Spring Torque + Preload of %0.2f Nm + Inertia', Fpr))
grid minor

subplot(2,2,4)
hold on
plot(procData.RP.time, procData.LC.torque.*procData.RP.p, 'LineWidth', 2)
plot(procData.RP.time, procData.RP.Traw - Fpr - I*procData.RP.measA.*procData.RP.p - I*procData.RP.angleA.*procData.RP.p.^2 - lowpass(procData.RP.Tf.*procData.RP.p, 5, 100), 'LineWidth', 2)
xlabel('Time, s');
ylabel('Torque, Nm');
legend({'p \times Loadcell',...
    'Model'})
title(sprintf('Spring Torque + Preload of %0.2f Nm + Inertia + Friction', Fpr))
grid minor
