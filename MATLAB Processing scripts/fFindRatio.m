function delta = fFindRatio(x, data)
p1 = x(1);
p2 = x(2);
p3 = x(3);

i1 = data.RP.time > 1 & data.RP.time < 4;
i2 = data.RP.time > 7 & data.RP.time < 9;
i3 = data.RP.time > 12 & data.RP.time < 14;

p = p1*i1 + p2*i2 + p3*i3;

i = i1 | i2 | i3;

rotM = data.RP.measVp(i);
rotS = data.RP.angleV(i);

dP = rotM - p(i).*rotS;
% dP = rotM - p(i).^(-1).*rotS;
delta = sum(abs(dP));

