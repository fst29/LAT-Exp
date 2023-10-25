function procData = fTorsion_v3(data)
procData = data;
% Data tests
% 0-5s Zero position
% 6-10s next position
% 11-15s final position
% Need to find the stiffness p
x0 = [3, 2, 1];
options = optimoptions(@fmincon,'MaxFunctionEvaluations', 3e4);
x = fmincon(@(x)(fFindRatio(x, data)), x0, [], [], [], [], [], [], [], options);

p1 = x(1);
p2 = x(2);
p3 = x(3);
pVec = [p1, p2, p3];

x1 = mean(data.RP.pinionP(data.RP.time > 1 & data.RP.time < 4));
x2 = mean(data.RP.pinionP(data.RP.time > 7 & data.RP.time < 9));
x3 = mean(data.RP.pinionP(data.RP.time > 12 & data.RP.time < 14));
xVec = [x1, x2, x3];

f = polyfit([x1, x2, x3], [p1, p2, p3], 1);
% f = [0,1];
procData.RP.p = polyval(f, procData.RP.pinionP);
procData.RP.p2= p1 + f(1)* procData.RP.pinionP;
procData.RP.f = f;
procData.RP.pVec = pVec;
procData.RP.xVec = xVec;
procData.RP.pPos = cumtrapz(procData.RP.time, procData.RP.p.*procData.RP.angleV);


theta0 = linspace(0, 1, 10);
Fstick0 = zeros(1, 10);

x0 = [0.5, 0.1, 0, 0, 0, 0.1, 0.05, Fstick0];
options = optimoptions(@fmincon,'MaxFunctionEvaluations', 3e4);
x = fmincon(@(x)(fMinStiffness_v3(x, procData)), x0, [], [], [], [], [], [], [], options);
k = x(1);
F = x(2);
procData.RP.Fpr = x(3);
vStatic = x(4);
procData.RP.Fnm = x(5);
procData.RP.Fnm = 0;
procData.RP.Ff = x(6);
procData.RP.I = x(7);
procData.RP.I = 0.0007;
procData.RP.FstickVec = x(8:end);
procData.RP.Fstick = lowpass(interp1(theta0, x(8:end), procData.RP.measP, 'nearest', 'extrap'), 0.1, 100);
procData.RP.Fstick = 0;

procData.RP.direction = sign(data.RP.angleV);
i = abs(procData.RP.angleV) < vStatic;
Fd = ~i*F;
Fs = i*2*F;
noMotion = (procData.RP.time > 8 & procData.RP.time < 10) | (procData.RP.time > 18 & procData.RP.time < 20);
T = lowpass(procData.RP.p.^(-1).*(-k*procData.RP.springP - Fs.*procData.RP.direction.*~noMotion - Fd.*procData.RP.direction.*~noMotion + procData.RP.Fpr + procData.RP.Fnm*noMotion + procData.RP.Ff.*procData.RP.measVp) -procData.RP.I*procData.RP.measA , 10, 100);
% T = procData.RP.p.^(-1).*(-k*procData.RP.springP - Fs.*procData.RP.direction.*~noMotion - Fd.*procData.RP.direction.*~noMotion + procData.RP.Fpr + procData.RP.Fnm*noMotion + procData.RP.Ff.*procData.RP.measVp) -procData.RP.I*procData.RP.measA + procData.RP.Fstick;
procData.RP.Traw = -k*procData.RP.springP;
procData.RP.Tm = T;
procData.RP.k = k;
procData.RP.F = F;
procData.RP.vStatic = vStatic;
dpdt = diff(procData.RP.p)/0.01;
procData.RP.pDot = [dpdt(1); dpdt];
procData.RP.pAcc = procData.RP.pDot.*procData.RP.angleV + procData.RP.p.*procData.RP.angleA;



