function pFDot = fVarspringTorqueODE(t, x, const)
u = interp1(const.time, const.u, t);
p = interp1(const.time, const.p, t);
k = const.k;
pFDot = k/p*u;