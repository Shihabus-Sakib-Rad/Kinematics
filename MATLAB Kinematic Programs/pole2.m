function polecoord = pole2(a1,a2,theta1,theta2)% Using the Pole Equation from the book% a1 is the coordinate of the first position% a2 is the coordinate of the second position% theta1 is the theta angle of the first position% theta2 is the theta angle of the second position% this routine will not take care of the parallel situationd2r = pi/180;if theta1 == theta2    theta1 = theta1+0.1;endtheta12 = theta2-theta1;c12 = cos(theta12*d2r);s12 = sin(theta12*d2r);if c12 ~= 1      cost = 1/(2-2*c12);      B1 = [(1-c12)  s12   ;          -s12  (1-c12)];      B2 = [(1-c12) -s12   ;           s12  (1-c12)];      C1 = [a1(1);a1(2)];   C2 = [a2(1);a2(2)];    polecoord = cost*(B1*C1+B2*C2);   % polecord(3) is just a flag used in the RBG4barDesign program   polecoord(3) = 0;   else      printf('parallel');   return;end