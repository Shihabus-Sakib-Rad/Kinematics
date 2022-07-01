function [f] = harmonic(tt,Hs,He,start,ending)

% Function harmonic(tt,Hs,He,start,ending)
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

rise = abs(He-Hs);     % degree input
%rise = rised*fact;      % convert to radian to calculate

theta=tt-start;
theta=theta*fact;
if He-Hs == 0
    f(1) = Hs;
    f(2) = 0;
    f(3) = 0;
    f(4) = 0;
% else
%     f(1) = (He+Hs)/2-(He-Hs)/2*cos(pi*theta/beta);
%     f(2) = ((He-Hs)/2)*(pi/beta)*sin(pi*theta/beta);
%     f(3) = ((He-Hs)/2)*((pi/beta)^2)*cos(pi*theta/beta);
%     f(4) = -((He-Hs)/2)*((pi/beta)^3)*sin(pi*theta/beta);

%**************************************************************************
elseif Hs < He
    f(1) = (rise/2)*(1-cos(pi*theta/beta));
    f(2) = (rise/2)*(pi/beta)*sin(pi*theta/beta);
    f(3) = (rise/2)*((pi/beta)^2)*cos(pi*theta/beta);
    f(4) = -(rise/2)*((pi/beta)^3)*sin(pi*theta/beta);
    
else
%    f(1) = ((He+Hs)/2)*(1+cos(pi*theta/beta));
    f(1) = (rise/2)*(1+cos(pi*theta/beta));
    f(2) = -(rise/2)*(pi/beta)*sin(pi*theta/beta);
    f(3) = -(rise/2)*((pi/beta)^2)*cos(pi*theta/beta);
    f(4) = (rise/2)*((pi/beta)^3)*sin(pi*theta/beta);    
    
%**************************************************************************

end