function pFDot = fVarspringTorqueODE_v2(t, x, const)
u = interp1(const.time, const.u, t);
p = interp1(const.time, const.p, t);
pFDot = 1/p*u;