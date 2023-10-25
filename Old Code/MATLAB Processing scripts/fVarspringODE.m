function pFDot = fVarspringODE(t, x, const)
u = interp1(const.time, const.u, t);
p = interp1(const.time, const.p, t);
k = const.k;
F = interp1(const.time, const.Tdot, t);
if abs(x)<0.07
    F = 0;
end
pFDot = k/p*u + F;