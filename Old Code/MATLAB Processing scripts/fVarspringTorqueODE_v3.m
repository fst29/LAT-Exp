function pFDot = fVarspringTorqueODE_v3(t, x, const)
u = interp1(const.time, const.V, t);
p = const.p;
pFDot = 1/p*u;
