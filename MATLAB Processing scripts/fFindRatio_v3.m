function delta = fFindRatio_v3(x, data)
p1 = x(1);

i1 = data.RP.time < 3;

p = p1*i1;

i = i1;

rotM = data.RP.measVp(i);
rotS = data.RP.angleV(i);

dP = rotM - p(i).*rotS;
delta = sum(abs(dP));

