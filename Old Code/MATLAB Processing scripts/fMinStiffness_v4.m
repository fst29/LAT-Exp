function delta = fMinStiffness_v4(x, data)
k = x(1);
i = data.RP.time < 1;
p = data.RP.p1;
T = -p^(-1)*k*data.RP.pPos(i);
T1 = data.LC.torque(i);

dT = T1 - T;
delta = sum(abs(dT));

