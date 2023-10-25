function F = fFrictionModelA(w)
Fas = 0.15;
Fac = 0.095;
wac = 0.05;

F(abs(w)<wac) = Fas*sign(w(abs(w)<wac));
F(abs(w)>=wac) = Fac*sign(w(abs(w)>=wac));
F = F;