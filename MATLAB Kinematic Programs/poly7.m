function [f] = poly7(tt,Hs,He,Vs,Ve,As,Ae,Js,Je,start,ending)

% Function poly7(tt,Hs,He,Vs,Ve,As,Ae,Js,Je,start,ending)
% Determines the follower displacement, velocity, acceleration, and jerk.
% Inputs are:
%   tt  = cam angle (deg.)
%   Hs  = start height
%   He  = end height
%   Vs  = start velocity
%   Ve  = end velocity
%   As  = start acceleration
%   Ae  = end acceleration
%   Js  = start jerk
%   Je  = end jerk
%   start = start angle (deg.)
%   ending = end angle (deg.)

if isempty(Vs); Vs=0; end
if isempty(Ve); Ve=0; end
if isempty(As); As=0; end
if isempty(Ae); Ae=0; end

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
else
    c0 = Hs;
    c1 = Vs*beta;
    c2 = 1/2*As*beta^2;
    c3 = 1/6*Js*beta^3;
    c4 = -5*As*beta^2-35*Hs+35*He-20*Vs*beta-2/3*Js*beta^3-15*Ve*beta+5/2*Ae*beta^2-1/6*Je*beta^3;
    c5 = -5*As*beta^2-Js*beta^3-3*c4-6*Ve*beta-15*Vs*beta-21*Hs+21*He+1/2*Ae*beta^2;
    c6 = -6*Vs*beta-5/2*As*beta^2-2/3*Js*beta^3-3*c4-2*c5-7*Hs+7*He-Ve*beta;
    c7 = -Hs-Vs*beta-1/2*As*beta^2-1/6*Js*beta^3-c4-c5-c6+He;
    f(1) = c0+c1*theta/beta+c2*theta^2/(beta^2)+c3*theta^3/(beta^3)+...
        c4*theta^4/(beta^4)+c5*theta^5/(beta^5)+c6*theta^6/(beta^6)+c7*theta^7/(beta^7);
    f(2) = (c1+2*c2*theta/beta+3*c3*theta^2/(beta^2)+4*c4*theta^3/(beta^3)+...
        5*c5*theta^4/(beta^4)+6*c6*theta^5/(beta^5)+7*c7*theta^6/(beta^6))/beta;
    f(3) = (2*c2+6*c3*theta/beta+12*c4*theta^2/(beta^2)+20*c5*theta^3/(beta^3)+...
        30*c6*theta^4/(beta^4)+42*c7*theta^5/(beta^5))/(beta^2);
    f(4) = (6*c3+24*c4*theta/beta+60*c5*theta^2/(beta^2)+120*c6*theta^3/(beta^3)+...
        210*c7*theta^4/(beta^4))/(beta^3);
end
