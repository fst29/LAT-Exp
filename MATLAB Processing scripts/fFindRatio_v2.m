function delta = fFindRatio_v2(x, data)
p1 = x(1);

i1 = data.RP.time < 1;

p = p1*i1;

i = i1;

rotM = data.RP.measVp(i);
rotS = data.RP.angleV(i);

dP = rotM - p(i).*rotS;
delta = sum(abs(dP));

