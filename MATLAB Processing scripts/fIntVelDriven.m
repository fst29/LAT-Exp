function [xDot] = fIntVelDriven(t, x, data)
vel = interp1(data.RP.time, data.RP.angleV, t);
p = interp1(data.RP.time, data.RP.p, t);
xDot = p*vel;