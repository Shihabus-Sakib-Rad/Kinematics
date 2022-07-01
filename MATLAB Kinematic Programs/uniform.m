function [f] = uniform(tt,Hs,He,start,ending)

% Function uniform(tt,Hs,He,start,ending)
% Determines the follower displacement, velocity, acceleration, and jerk.
% Inputs are:
%   tt  = cam angle (deg.)
%   Hs  = start height
%   He  = end height
%   start = start angle (deg.)
%   ending = end angle (deg.)

fact=pi/180;
beta = ending - start;
beta=beta*fact;
rise = abs(He-Hs);
theta=tt-start;
theta=theta*fact;
if He-Hs == 0
    f(1) = Hs;
    f(2) = 0;
    f(3) = 0;
    f(4) = 0;
    f(5) = 0;
    f(6) = 0;
    f(7) = 0;
elseif He-Hs > 0
    f(1) = Hs + (rise/beta)*(theta);
    f(2) = (rise/beta);
    f(3) = 0;
    f(4) = 0;
    f(5) = 0;
    f(6) = 0;
    f(7) = 0;    
else
    f(1) = Hs - (rise/beta)*(theta);
    f(2) = -(rise/beta);
    f(3) = 0;
    f(4) = 0;
    f(5) = 0;
    f(6) = 0;
    f(7) = 0;     
end
