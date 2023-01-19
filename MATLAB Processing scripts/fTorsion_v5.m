function procData = fTorsion_v5(data)
procData = data;
% Data tests
% 0-5s Zero position
% 6-10s next position
% 11-15s final position
% Need to find the stiffness p
x0 = 1;
options = optimoptions(@fmincon,'MaxFunctionEvaluations', 3e4);
x = fmincon(@(x)(fFindRatio_v3(x, data)), x0, [], [], [], [], [], [], [], options);

p1 = x(1);
% f = 7e-04; 
f = 6.689489472483746e-04;
% f = [0,1];
procData.RP.p1 = p1;
procData.RP.p= p1 + f(1)* procData.RP.pinionP;
procData.RP.f = f;
[~, procData.RP.pPos] = ode45(@(t, x)fIntVelDriven(t, x, procData), procData.RP.time, 0);
[~, procData.RP.measPp] = ode45(@(t, x)fIntVelDriving(t, x, procData), procData.RP.time, 0);

% procData.RP.pPos = cumtrapz(procData.RP.time, procData.RP.p.*procData.RP.angleV);

% x0 = 1;
% options = optimoptions(@fmincon,'MaxFunctionEvaluations', 3e4);
% x = fmincon(@(x)(fMinStiffness_v4(x, procData)), x0, [], [], [], [], [], [], [], options);
% k = x(1);
k = 0.52;
% k1 = 0.52;
procData.RP.direction = sign(data.RP.angleV);
procData.RP.Traw = -k*procData.RP.springP;
% procData.RP.Traw(procData.RP.springP>=0) = -k*procData.RP.springP(procData.RP.springP>=0);
% procData.RP.Traw(procData.RP.springP<0) = -k1*procData.RP.springP(procData.RP.springP<0);
procData.RP.k = k;

dpdt = diff(procData.RP.p)/0.01;
procData.RP.pDot = [dpdt(1); dpdt];
procData.RP.pAcc = procData.RP.pDot.*procData.RP.angleV + procData.RP.p.^(-1).*procData.RP.angleA;

dpdt = diff(1./procData.RP.p)/0.01;
procData.RP.pDotInv = lowpass([dpdt(1); dpdt], 10,100);
procData.RP.pMotorAcc = 1./procData.RP.p.*procData.RP.measA + procData.RP.pDotInv.*procData.RP.measVp;



