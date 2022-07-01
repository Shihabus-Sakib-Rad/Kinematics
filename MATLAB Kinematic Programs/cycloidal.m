function [f] = cycloidal(tt,Hs,He,start,ending)

% function [f] = cycloidal(tt,Hs,He,start,ending)

% Determines the follower displacement, velocity, acceleration, and jerk.
% Inputs are:
%   tt  = cam angle (deg.)
%   Hs  = start height
%   He  = end height
%   start = start angle (deg.)
%   ending = end angle (deg.)

fact=pi/180;
beta = ending - start;
beta = beta*fact;
rise = abs(He-Hs);
theta=tt-start;
theta=theta*fact;
if He-Hs == 0
    f(1) = Hs;
    f(2) = 0;
    f(3) = 0;
    f(4) = 0;
elseif Hs < He
    f(1) = (He-Hs)*(theta/beta-1/(2*pi)*sin(2*pi*theta/beta));
    f(2) = (He-Hs)/beta*(1-cos(2*pi*theta/beta));
    f(3) = 2*pi*(He-Hs)/beta^2*(sin(2*pi*theta/beta));
    f(4) = (2*pi)^2*(He-Hs)/beta^3*(cos(2*pi*theta/beta));
else
    f(1) = abs(He-Hs)-abs(He-Hs)*(theta/beta-1/(2*pi)*sin(2*pi*theta/beta));
    f(2) = -abs(He-Hs)/beta*(1-cos(2*pi*theta/beta));
    f(3) = -2*pi*abs(He-Hs)/beta^2*(sin(2*pi*theta/beta));
    f(4) = -(2*pi)^2*abs(He-Hs)/beta^3*(cos(2*pi*theta/beta));
end
