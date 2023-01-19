function procData = fProcessData(data)
% Encoder resolution - 1024 slots per revolution
% encA = data.encA;
% encB = data.encB;
time = data.time;
% [directionMod, direction, rotation, rotation2] = fGetDirection(data);
angle = -data.rotRaw/2048*2*pi;
angle2 = data.rotRaw/2048*2*pi;
% angle = 2*(rotation/1024*2*pi)';
% angle2 = -(rotation2/1024*2*pi)';
% angle = cumsum(encB.*directionMod)/1024*2*pi;
procData = data;
procData.springP = angle;
procData.angle = angle;
procData.angle2 = angle2;
procData.measP = data.measP/2048*2*pi;
procData.targetP = data.targetP/2048*2*pi;
procData.targetRaw = data.targetP;
procData.targetT = data.targetP*4.69/257;
dt = 0.01;
dP1dt = diff(angle)/0.01;
procData.angleV = lowpass([dP1dt(1); dP1dt], 12, 100);
% procData.angleV = [dP1dt(1); dP1dt];
dV1dt = diff(procData.angleV)/0.01;
procData.angleA = lowpass([dV1dt(1); dV1dt], 12, 100);
% procData.angleA = [dV1dt(1); dV1dt];
dP2dt = diff(procData.measP)/0.01;
procData.measVp = lowpass([dP2dt(1); dP2dt], 12, 100);
% procData.measVp = [dP2dt(1); dP2dt];
dV2dt = diff(procData.measVp)/0.01;
procData.measA = lowpass([dV2dt(1); dV2dt], 12, 100);
% procData.measA = [dV2dt(1); dV2dt];


% procData.direction = lowpass(direction, 0.1, 100);

