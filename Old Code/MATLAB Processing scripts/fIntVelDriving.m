function [xDot] = fIntVelDriving(t, x, data)
vel = interp1(data.RP.time, data.RP.measVp, t);
xDot = vel;