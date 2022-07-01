function []=RBGSliderCrankDesign(Action, Argument2)
%MATLAB file for design of a slider crank linkage for rigid-body guidance.
%The slider is the driver, and the crank is selected first.

%Some of the important variables used are:

%ax1     = x coordinate of coupler coordinate system in position 1
%ay1     = y coordinate of coupler coordinate system in position 1
%ax2     = x coordinate of coupler coordinate system in position 2
%ay2     = y coordinate of coupler coordinate system in position 2
%ax3     = x coordinate of coupler coordinate system in position 3
%ay3     = y coordinate of coupler coordinate system in position 3
%theta1  = angle from frame x axis to coupler x axis in position 1 (degrees)
%theta2  = angle from frame x axis to coupler x axis in position 2 (degrees)
%theta3  = angle from frame x axis to coupler x axis in position 3 (degrees)
%acx(i)  = x coordinate of circle point relative to fixed system for position i
%acy(i)  = y coordinate of circle point relative to fixed system for position i
%XC1     = x coordinate of first circle point relative to coupler coordinate system
%YC1     = y coordinate of first circle point relative to coupler coordinate system
%XC2     = x coordinate of second circle point relative to coupler coordinate system
%YC2     = y coordinate of second circle point relative to coupler coordinate system
%Astar   = coordinates of center point corresponding to [XC1, YC1]
%Bstar   = coordinates of center point corresponding to [XC2, YC2]
%p12     = coordinates of pole between positions 1 and 2
%p13     = coordinates of pole between positions 1 and 3
%p23     = coordinates of pole between positions 2 and 3
%p23prime= coordinates of image pole corresponding to p23

% This program uses the following m.files.  These must be located in the same directory
% or the path to the routines must be clearly defined:

%	centerpoint.m - Routine for finding center point given circle point
%	circlepoint.m - Routine for finding circle point given center point
%	assemblymode_cs.m - Routine to find assembly mode
%	sldcrkc.m - Routine to analyze slider crank linkage when the crank is driving.
%	circle.m - Routine to determine the coordinates of a circle
%	rectangle.m - Routine to determine the coordinates of a rectangle
%	frameline.m - Routine to determine the coordinates of a frame line
%	sliderpoint.m - Routine for finding slider point given a point 
%                   near the circle of sliders.
%	ipole.m - Routine to find image pole given displacement poles
%	pole.m - Routine to find displacement poles
%	bushing.m - Routine to find coordinates of a bushing


global g_AnaRes
global nCheckCenter1

if nargin == 0 && isempty(findobj('Tag','rbg_slidercrank_dsgn'))% RBGSliderCrankDesign is being initialized
	Action = 'initialize';
   
elseif nargin == 0 && ~isempty(findobj('Tag','rbg_slidercrank_dsgn'))	  % RBGSliderCrankDesign is already open
   Action = 'None';
end
 
if ~strcmp(Action,'initialize')&& ~strcmp(Action,'Help')
   % retrieve the previous data from the userdata of some objects
   a = findobj('Tag','rbg_slidercrank_dsgn');
   UD=a.UserData;
   handles = UD.hand;
   h_astarx = handles(1);
   h_astary = handles(2);
   Astarx = get(h_astarx,'userdata');
   Astary = get(h_astary,'userdata');
   h_ax = handles(3);
   h_ay = handles(4);
   fx1 = get(h_ax,'userdata');
   fy1 = get(h_ay,'userdata');
   
% sldx and sldy are the x and y coordinates in the coupler coordinate system
% the editable variable are actually fx2 and fy2, which can only be modified through
% the mouse movement.
   h_sldx = handles(5);
   h_sldy = handles(6);
   sldx = get(h_sldx,'userdata');
   sldy = get(h_sldy,'userdata');
   fx2 = handles(31);
   fy2 = handles(32);
   
   h_posx = handles(7:9);
   h_posy = handles(10:12);
   h_theta = handles(13:15);
   for n = 1:3
      ax(n) = get(h_posx(n),'userdata');
      ay(n) = get(h_posy(n),'userdata');
      theta(n) = get(h_theta(n),'userdata');
   end
   
   ax1 = ax(1);
   ay1 = ay(1);
   theta1 = theta(1);
   ax2 = ax(2);
   ay2 = ay(2);
   theta2 = theta(2);
   ax3 = ax(3);
   ay3 = ay(3);
   theta3 = theta(3);
   
   h_r1txt =  handles(16);
   h_r2txt =  handles(17);
   h_r3txt =  handles(18);
   h_r4txt =  handles(19);

   h_status = handles(20);
   
   b = findobj('Tag','rbg_crankslider_axes');
   handles = get(b,'userdata');
   position1 = handles(1);
   position2 = handles(2);
   position3 = handles(3);
   a1 = handles(4);
   a2 = handles(5);
   a3 = handles(6);
   pole12 = handles(7);
   pole13 = handles(8);
   pole23 = handles(9);
   ipole23 = handles(10);
   scircle = handles(11);
%=============================================================================================================
   icircle23 = handles(12);
   icircle12 = handles(13);
   icircle13 = handles(14);
%=============================================================================================================
   joint1 = handles(15);
   joint2 = handles(16);
   joint3 = handles(17);
   sliderline1 = handles(18);
   crank1 = handles(19);
   block1 = handles(20);
   coupler1 = handles(21);
   coupler2 = handles(22);
   coupler3 = handles(23);
   hinge1 = handles(24);
%=============================================================================================================
   patch_1 = handles(25);
   patch_2 = handles(26);
%=============================================================================================================   


	set(h_status,'String','');   
end  

switch Action
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	Initialize the User Interface
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
case  'initialize'
   if nargin >= 2 || ~isempty(findobj('tag','menu_page'))
   nCheckCenter1 = 1;
   LocalOpenFig;
   RBGSliderCrankDesign('Draw Axes');
   else
   	show_me_logo('initialize','RBGSliderCrankDesign');
   end
   
case 'Draw Axes'   
% Identify vectors giving the three components of a and b on the coupler.
   drawfact=20;
   height=0.8;
   width=0.8;
   
   d1x=cos(theta1*pi/180);
	d1y=sin(theta1*pi/180);
	d2x=cos(theta2*pi/180);
	d2y=sin(theta2*pi/180);
	d3x=cos(theta3*pi/180);
	d3y=sin(theta3*pi/180);
	cross1=abs(d1x*d2y-d1y*d2x);
	cross2=abs(d1x*d3y-d1y*d3x);
	cross3=abs(d2x*d3y-d2y*d3x);
	if abs(cross1)<10^(-8) || abs(cross2)<10^(-8) || abs(cross3)<10^(-8)
      set(h_status,'string', ...
         ' Two or more positions approximately parallel, reinput positions');
      return
   end
   
% Identify vectors giving the three components of a and b on the coupler.
   fact=pi/180;
   theta1r=theta1*fact;
   theta2r=theta2*fact;
   theta3r=theta3*fact;
   ax=[ax1 ax2 ax3];
   ay=[ay1 ay2 ay3];
   bx=[ax1+cos(theta1r) ax2+cos(theta2r) ax3+cos(theta3r)];
   by=[ay1+sin(theta1r) ay2+sin(theta2r) ay3+sin(theta3r)];
   drawlim1=max([max(ax); max(ay); max(bx); max(by)]);
   drawlim2=min([max(ax); min(ay); min(bx); min(by)]);
   drawlim=drawfact*max([abs(drawlim1); abs(drawlim2)]);

% Identify poles using pole.m function.
   p12=pole([ax(1);ay(1)],[ax(2);ay(2)],[bx(1);by(1)],[bx(2);by(2)]);	
   p13=pole([ax(1);ay(1)],[ax(3);ay(3)],[bx(1);by(1)],[bx(3);by(3)]);
   p23=pole([ax(2);ay(2)],[ax(3);ay(3)],[bx(2);by(2)],[bx(3);by(3)]);
   pflag12=p12(3);
   pflag13=p13(3);
   pflag23=p23(3);
   pflagtot=pflag12+pflag13+pflag23;

% Identify the image pole p'23
   p23prime=ipole(p12,p13,p23);
   
% find the coordinates of those three circles for assembly feasibility  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   x23 = (p12(1) + p13(1))/2;
   y23 = (p12(2) + p13(2))/2;
   r23 = sqrt((p12(1)-x23)^2+(p12(2)-y23)^2);
   x12 = (p13(1) + p23prime(1))/2;
   y12 = (p13(2) + p23prime(2))/2;
   r12 = sqrt((p13(1)-x12)^2+(p23prime(2)-y12)^2);
   x13 = (p12(1) + p23prime(1))/2;
   y13 = (p12(2) + p23prime(2))/2;
   r13 = sqrt((p12(1)-x13)^2+(p23prime(2)-y13)^2);
   i23 = circle2(r23,x23,y23,30);
   i12 = circle2(r12,x12,y12,30);
   i13 = circle2(r13,x13,y13,30);
	prime23circle = [x23 y23 r23];
	prime12circle = [x12 y12 r12];
	prime13circle = [x13 y13 r13];  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    

% Find the circle of sliders relative to the coupler if the poles are finite.
   if pflagtot==0;
   	center=pole([p12(1);p12(2)],[p13(1);p13(2)],[p13(1);p13(2)],...
              [p23prime(1);p23prime(2)]);
   	x0=center(1);
   	y0=center(2);
   	dflag=center(3);
   	rc=sqrt((p12(1)-x0)^2+(p12(2)-y0)^2);

% Draw the circle relative to the frame when the poles are all finite.
   	dalpha=2*pi/30;
   	for j=1:1:31
   		sx(j)=ax1;
   		sy(j)=ay1;
   		alpha=(j-1)*dalpha;
   		if dflag==0
   			sx(j)=x0+rc*cos(alpha);
   			sy(j)=y0+rc*sin(alpha);
   		end
   	end
   end

% Draw straight line relative to the frame a pole lies at infinity
   if pflagtot>0
   	rc=10^10;
   	dflag=1;
   	stline=cosline(p12,p13,p23,p23prime);
   	for j=1:1:31
   		sx(j)=stline(j,1);
   		sy(j)=stline(j,2);
   	end
   	cirangle=pi/2+atan2((sy(31)-sy(1)), (sx(31)-sx(1)));
   	x0=sx(15)+rc*cos(cirangle);
   	y0=sy(15)+rc*sin(cirangle);
   end

% Draw straight line relative to the frame if pole lies at infinity
   if pflagtot>0
   	stline=cosline(p12,p13,p23,p23prime);
   	for j=1:1:31
   		sx(j)=stline(j,1);
   		sy(j)=stline(j,2);
   	end
   end

% If a pole is far from the positions, do not draw it
   p12x=p12(1);
   if abs(p12x) > drawlim; p12x=ax; end
   p12y=p12(2);
   if abs(p12y) > drawlim; p12y=ay; end
   p13x=p13(1);
   if abs(p13x) > drawlim; p13x=ax; end
   p13y=p13(2);
   if abs(p13y) > drawlim; p13y=ay; end
   p23x=p23(1);
   if abs(p23x) > drawlim; p23x=ax; end
   p23y=p23(2);
   if abs(p23y) > drawlim; p23y=ay; end

% Find axis limits
   cx=[ax, bx, p12x, p13x, p23x, sx];
   cy=[ay, by, p12y, p13y, p23y, sy];
   rangex=max(cx)-min(cx);
   rangey=max(cy)-min(cy);
   xmin=min(cx)-0.2*rangex;
   xmax=max(cx)+0.2*rangex;
   ymin=min(cy)-0.2*rangey;
   ymax=max(cy)+0.2*rangey;
   values=axisadjust(xmin, xmax, ymin, ymax, width, height);
   xmin=values(1);
   xmax=values(2);
   ymin=values(3);
   ymax=values(4);
   axis([xmin xmax ymin ymax]);

% Plot the positions
% To scale the length of the position line base on the axis size
% make it one tenth of the axis x width.
   f = (xmax-xmin)/10;
   bx=[ax1+f*cos(theta1r) ax2+f*cos(theta2r) ax3+f*cos(theta3r)];
   by=[ay1+f*sin(theta1r) ay2+f*sin(theta2r) ay3+f*sin(theta3r)];

   set(position1, 'xdata', [ax(1) bx(1)], 'ydata', [ay(1) by(1)]);
   set(position2, 'xdata', [ax(2) bx(2)], 'ydata', [ay(2) by(2)]);
   set(position3, 'xdata', [ax(3) bx(3)], 'ydata', [ay(3) by(3)]);
   set(a1, 'xdata',ax(1), 'ydata', ay(1));
   set(a2, 'xdata',ax(2), 'ydata', ay(2));
   set(a3, 'xdata',ax(3), 'ydata', ay(3));
   
   a = findobj('tag','rbg_slidercrank_dsgn');
   UD=a.UserData;
   hdls = UD.hand;
   lim = [xmin xmax ymin ymax];
   set(a,'userdata',struct('hand',hdls,'wlim',lim,'Bx',bx,'By',by));
   
% Plot the poles
   set(pole12, 'xdata',p12(1), 'ydata', p12(2));
   set(pole13, 'xdata',p13(1), 'ydata', p13(2));
   set(pole23, 'xdata',p23(1), 'ydata', p23(2));
   set(ipole23, 'xdata',p23prime(1), 'ydata', p23prime(2));

% Plot the slider circle and image pole circles %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

   set(scircle, 'xdata',sx, 'ydata', sy);
   set(icircle23, 'xdata',i23(:,1), 'ydata', i23(:,2));
   set(icircle12, 'xdata',i12(:,1), 'ydata', i12(:,2));
   set(icircle13, 'xdata',i13(:,1), 'ydata', i13(:,2));
   scale=sqrt((max(ax)-min(ax))^2+(max(ay)-min(ay))^2);
   rpivot=0.02*scale;
   delta=2*rpivot;	

%=============================================================
% Part of program where joints are selected.
%=============================================================
% The point selected is in the frame coordinate system.  Transform
% the point to the coupler coordinate system. 
%
   st1=sin(theta1r);
   ct1=cos(theta1r);
   if nCheckCenter1 == 1 %center point input
      Astar(1)=Astarx;
      Astar(2)=Astary;
		Values = circlepoint (ax1,ay1,theta1,ax2,ay2,theta2,ax3,ay3,...
               theta3, Astarx,Astary);
		acx(1)=Values(1);
		acy(1)=Values(2);	
      set(h_ax,'string',num2str(acx(1)),'userdata',acx(1));
      set(h_ay,'string',num2str(acy(1)),'userdata',acy(1));

   else % circle input
		XC1= (fx1-ax1)*ct1+(fy1-ay1)*st1;
		YC1= -(fx1-ax1)*st1+(fy1-ay1)*ct1;
	
		% Compute first center point.
		Values=centerpoint(ax1,ay1,theta1,ax2,ay2,theta2, ax3,ay3,theta3,XC1,YC1);
		Astar(1) = Values(1);
		Astar(2) = Values(2);
		for ii=1:1:3
			acx(ii)=Values(2*ii+1);
			acy(ii)=Values(2*ii+2);
		end
      set(h_astarx,'string',num2str(Astar(1)),'userdata',Astar(1));
      set(h_astary,'string',num2str(Astar(2)),'userdata',Astar(2));

   end
   


% Find coordinates of first bushing and pin joint.
				
   ninc=20;  
   npoints=ninc+1;
   centerptx=Astar(1);
   centerpty=Astar(2);
   coord=circle2(rpivot,centerptx,centerpty,ninc);

% find coordinates of first pin

   for i=1:1:npoints
      xcoord1(i)=coord(i,1);
      ycoord1(i)=coord(i,2);
   end

% find coordinates of first bushing

   binc=ninc/2;
   coord1=bushing(rpivot,centerptx,centerpty,binc,0);
   bpoints=ninc/2+16;
   for i=1:1:bpoints
		xbush1(i)=coord1(i,1);
		ybush1(i)=coord1(i,2);
	end

% draw the first bushing

	set(joint1,'xdata', acx(1), 'ydata',acy(1));
	set(joint2,'xdata', xcoord1, 'ydata',ycoord1);
	set(hinge1,'xdata', xbush1,'ydata', ybush1);
	set(crank1,'xdata', [acx(1) Astar(1)], 'ydata',[acy(1) Astar(2)]);

% **********************************************************************
% Slider point input part
% **********************************************************************
% Rectify the point to the slider point circle.  First find the line from 
% the point chosen to be the center of the circle.  Then find the point on the
% circle that is on the line.  The center of the circle is at x0, y0, and 
% the radius of the circle is rc.  

	angle=atan2(sldy-y0, sldx-x0);
	xn=x0+rc*cos(angle);
	yn=y0+rc*sin(angle);

% The point selected is in the frame coordinate system.  Transform
% the point to the coupler coordinate system. 
	st1=sin(theta1r);
	ct1=cos(theta1r);
				
% Compute the three positions of the slider point and slider line coordinates
	Xn1= (xn-ax1)*ct1+(yn-ay1)*st1;
	Yn1= -(xn-ax1)*st1+(yn-ay1)*ct1;
					
% Determine coordinates of the slider point for three positions
	Values=sliderpoint(ax1,ay1,theta1,ax2,ay2,theta2, ax3,ay3,...
		 theta3,Xn1,Yn1);

	for ii=1:1:3
		asx(ii)=Values(2*ii-1);
		asy(ii)=Values(2*ii);
	end
    
   set(h_sldx,'string',num2str(sldx),'userdata',sldx);
   set(h_sldy,'string',num2str(sldy),'userdata',sldy);
   
   set(fx2,'string',num2str(asx(1)));
   set(fy2,'string',num2str(asy(1)));
		
% Determine the coordinates of the slider line	
	lineangle=Values(7)*fact;
	length1=Values(8);
	if length1 < 2*delta; length1=2*delta; end
	ndash=20;
	csang=cos(lineangle);
	ssang=sin(lineangle);					
	xe = asx(1)-0.5*length1*csang;
	ye = asy(1)-0.5*length1*ssang;
	flag=1;
	lineangled=Values(7);
	coord=frameline(2*length1,xe,ye,ndash,lineangled,flag);					
	npoints1=3*ndash;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Calculation of the coordinates of the Bstar  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Values=centerpoint(ax1,ay1,theta1,ax2,ay2,theta2, ax3,ay3,...
		 theta3,Xn1+0.001,Yn1+0.001);
     Bstar(1)=Values(1);
     Bstar(2)=Values(2);
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
% Offset slider line by half the height of the slider block.
	for i=1:1:npoints1
		xsl1(i)=coord(i,1)+delta*ssang;
		ysl1(i)=coord(i,2)-delta*csang;
	end					
	set(sliderline1, 'xdata', xsl1, 'ydata', ysl1);
   
% draw the first slider block
	xpin=asx(1);
	ypin=asy(1);
	coords = rect(4*delta,2*delta,xpin,ypin,lineangled,0);
	for j=1:1:5
		xblock(j)=coords(j,1);
		yblock(j)=coords(j,2);
	end
	set(joint2,'xdata', xcoord1, 'ydata',ycoord1);
	set(block1,'xdata', xblock, 'ydata',yblock);
	set(joint3,'xdata', xpin, 'ydata',ypin);	
		
% Draw the coupler
	set(coupler1,'xdata', [acx(1) asx(1)], 'ydata',[acy(1) asy(1)]);
  
% **********************************************************************
% Slider-crank linkage designed.  Proceed with analysis
% **********************************************************************

% Define angles and distances to the original coupler line.

   Q2=atan2((acy(1)-Astar(2)), (acx(1)-Astar(1)))/fact;
   temp=atan2((ay(1)-acy(1)), (ax(1)-acx(1)));
   phi=atan2((asy(1)-acy(1)), (asx(1)-acx(1)));
   beta1=temp-phi;
   temp=atan2((by(1)-acy(1)), (bx(1)-acx(1)));
   beta2=temp-phi;
   rc2=sqrt((by(1)-acy(1))^2+ (bx(1)-acx(1))^2);
   rc1=sqrt((ay(1)-acy(1))^2+ (ax(1)-acx(1))^2);

% Define link lengths and offset distance (r4)

   r2=sqrt((acx(1)-Astar(1))^2+ (acy(1)-Astar(2))^2);
   r3=sqrt((asx(1)-acx(1))^2+ (asy(1)-acy(1))^2);
   r4=(asy(1)-Astar(2))*cos(lineangle)-(asx(1)-Astar(1))*sin(lineangle);	
   
% display the link lengths on the screen
   set(h_r1txt,'string',num2str(r2));
   set(h_r2txt,'string',num2str(r3));
   set(h_r3txt,'string',num2str(r4));
   set(h_r4txt,'string',num2str(lineangled));
   
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Filemon Construction Part 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    Bstar(1) = Astar(1);    % <----------------------------------------------------------------------------------------
    Bstar(2) = Astar(2);    % <----------------------------------------------------------------------------------------
    asx(1) = acx(1);    % <--------------------------------------------------------------------------------------------
    asy(1) = acy(1);    % <--------------------------------------------------------------------------------------------   

    delta = atan((Bstar(2)-asy(1))/(Bstar(1)-asx(1)));      % angle is in radian

    if ((Bstar(1) > asx(1)) && (Bstar(2) > asy(1)))            % alpha is in the 1st quarter area
        alpha = delta;
    end
    if ((Bstar(1) < asx(1)) && (Bstar(2) > asy(1)))            % alpha is in the 2nd quarter area
        alpha = pi+delta;
    end
    if ((Bstar(1) < asx(1)) && (Bstar(2) < asy(1)))            % alpha is in the 3rd quarter area
        alpha = pi+delta;
    end
    if ((Bstar(1) > asx(1)) && (Bstar(2) < asy(1)))            % alpha is in the 4th quarter area
        alpha = 2*pi+delta;
    end    

    % Calculation of -phi_12 and -phi_13
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Coordinate of the three positions
    a_xy=[ax1 ax2 ax3; ay1 ay2 ay3];
    theta = (pi/180)*[theta1 theta2 theta3];        % angles are in radian
    
    % Coordinate of the driven crank pivot
    x1 = asx(1);
    y1 = asy(1);
    
    x_star=Bstar(1);
    y_star=Bstar(2);
    
    % Transformation Matrix
    for i = 1;
    R = [cos(theta(i)) -sin(theta(i)) a_xy(1,i); ...
     sin(theta(i))  cos(theta(i)) a_xy(2,i); 0 0 1];
    end

    % Location of the driven crank pivot in the coupler coordi.
    B = [x1 y1 1]';         % (x1, y1)
    A = inv(R)*B;            % (X, Y)
    
    % Calculation of x2, x3 and y2, y3
    for i = 2:3
    R = [cos(theta(i)) -sin(theta(i)) a_xy(1,i); ...
     sin(theta(i))  cos(theta(i)) a_xy(2,i); 0 0 1];
    C(:,i-1) = R*A;
    i = i+1;
    end
    
    x2=C(1,1);
    y2=C(2,1);
    x3=C(1,2);
    y3=C(2,2);
    
    xy_i=[x1 x2 x3; y1 y2 y3];
    pq_i=a_xy;
    
    for i = 1:3
    cos_phi = ((xy_i(1,i)-x_star)*(pq_i(1,i)-xy_i(1,i)) + (xy_i(2,i)-y_star)*(pq_i(2,i)-xy_i(2,i)))/ ...
             sqrt(((xy_i(1,i)-x_star)^2+(xy_i(2,i)-y_star)^2)*((pq_i(1,i)-xy_i(1,i))^2+(pq_i(2,i)-xy_i(2,i))^2));
    sin_phi = ((xy_i(1,i)-x_star)*(pq_i(2,i)-xy_i(2,i)) - (xy_i(2,i)-y_star)*(pq_i(1,i)-xy_i(1,i)))/ ...
             sqrt(((xy_i(1,i)-x_star)^2+(xy_i(2,i)-y_star)^2)*((pq_i(1,i)-xy_i(1,i))^2+(pq_i(2,i)-xy_i(2,i))^2));
        
    phi_i(1,i) = 2*atan(sin_phi/(1+cos_phi));

    i=i+1;
    end
    
    phi_12 = (phi_i(2)-phi_i(1));
    phi_13 = (phi_i(3)-phi_i(1));

    minus_phi12 = -phi_12;       % the angle is in radian
    minus_phi13 = -phi_13;       % the angle is in radian
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    x = xmin:0.01:xmax;

% Calculation of the length(L) of Filemon lines
   L = sqrt(xmax^2 + ymax^2)*12;
   
% coordinates for the patch_1(for minus_phi_12) and patch_2(for minus_phi_13)

    % define assistant point for patch_1

   if abs(L*cos(alpha)) > abs(L*cos(alpha+minus_phi12))
       P_patch_1(1,1) = asx(1)+L*cos(alpha);
       P_patch_1(2,1) = asx(1)-L*cos(alpha);
   else
       P_patch_1(1,1) = asx(1)+L*cos(alpha+minus_phi12);
       P_patch_1(2,1) = asx(1)-L*cos(alpha+minus_phi12);
   end
   
   if abs(L*sin(alpha)) > abs(L*sin(alpha+minus_phi12))
       P_patch_1(1,2) = asy(1)+L*sin(alpha);
       P_patch_1(2,2) = asy(1)-L*sin(alpha);
   else
       P_patch_1(1,2) = asy(1)+L*sin(alpha+minus_phi12);
       P_patch_1(2,2) = asy(1)-L*sin(alpha+minus_phi12);
   end
        
    
   x_patch_1 = [asx(1)+L*cos(alpha) P_patch_1(1,1) asx(1)+L*cos(alpha+minus_phi12) asx(1) ...
                 asx(1)-L*cos(alpha+minus_phi12) P_patch_1(2,1) asx(1)-L*cos(alpha)];
   y_patch_1 = [asy(1)+L*sin(alpha) P_patch_1(1,2) asy(1)+L*sin(alpha+minus_phi12) asy(1) ...
                 asy(1)-L*sin(alpha+minus_phi12) P_patch_1(2,2) asy(1)-L*sin(alpha)];
    
    % define assistant point for patch_2
 
   if abs(L*cos(alpha)) > abs(L*cos(alpha+minus_phi13))
       P_patch_2(1,1) = asx(1)+L*cos(alpha);
       P_patch_2(2,1) = asx(1)-L*cos(alpha);
   else
       P_patch_2(1,1) = asx(1)+L*cos(alpha+minus_phi13);
       P_patch_2(2,1) = asx(1)-L*cos(alpha+minus_phi13);
   end
   
   if abs(L*sin(alpha)) > abs(L*sin(alpha+minus_phi13))
       P_patch_2(1,2) = asy(1)+L*sin(alpha);
       P_patch_2(2,2) = asy(1)-L*sin(alpha);
   else
       P_patch_2(1,2) = asy(1)+L*sin(alpha+minus_phi13);
       P_patch_2(2,2) = asy(1)-L*sin(alpha+minus_phi13);
   end

   x_patch_2 = [asx(1)+L*cos(alpha) P_patch_2(1,1) asx(1)+L*cos(alpha+minus_phi13) asx(1) ...
                 asx(1)-L*cos(alpha+minus_phi13) P_patch_2(2,1) asx(1)-L*cos(alpha)];
   y_patch_2 = [asy(1)+L*sin(alpha) P_patch_2(1,2) asy(1)+L*sin(alpha+minus_phi13) asy(1) ...
                 asy(1)-L*sin(alpha+minus_phi13) P_patch_2(2,2) asy(1)-L*sin(alpha)];

    set(patch_1,'xdata',x_patch_1,'ydata',y_patch_1);
    set(patch_2,'xdata',x_patch_2,'ydata',y_patch_2);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
      
% Set argument2 = 1 in 'Set Calls' and 'Move' for moving the circle point (A)around
% Set argument2 = 2 in 'Set Calls' and 'Move' for moving the center point (Astar)around
% Set argument2 = 3 in 'Set Calls' and 'Move' for moving the slider point around
   set([joint1],...
     'buttondownfcn','RBGSliderCrankDesign(''Set Calls'',1);RBGSliderCrankDesign(''Move'',1)');
   set([joint2 hinge1],...
     'buttondownfcn','RBGSliderCrankDesign(''Set Calls'',2);RBGSliderCrankDesign(''Move'',2)');
   set([joint3 block1],...
     'buttondownfcn','RBGSliderCrankDesign(''Set Calls'',3);RBGSliderCrankDesign(''Move'',3)');
	set([a1],...
	  'buttondownfcn','RBGSliderCrankDesign(''Set Calls'',5);RBGSliderCrankDesign(''Move Pos'',1)');
	set([a2],...
	  'buttondownfcn','RBGSliderCrankDesign(''Set Calls'',6);RBGSliderCrankDesign(''Move Pos'',2)');
	set([a3],...
	  'buttondownfcn','RBGSliderCrankDesign(''Set Calls'',7);RBGSliderCrankDesign(''Move Pos'',3)');
	set([position1],...
	  'buttondownfcn','RBGSliderCrankDesign(''Set Calls'',8);RBGSliderCrankDesign(''Move Ang'',1)');
	set([position2],...
	  'buttondownfcn','RBGSliderCrankDesign(''Set Calls'',9);RBGSliderCrankDesign(''Move Ang'',2)');
	set([position3],...
	  'buttondownfcn','RBGSliderCrankDesign(''Set Calls'',10);RBGSliderCrankDesign(''Move Ang'',3)');

case 'Set Calls'
 
% get the handle of the objects used to control
% the mouse input 
   a = findobj('Tag','rbg_slidercrank_dsgn');
   
% Assign different function calls to windowbuttonmotionfcn when users try to move different point   
   if Argument2 == 1
      set(a,'windowbuttonupfcn', 'a = findobj(''Tag'',''rbg_slidercrank_dsgn'');set(a,''windowbuttonmotion'','''');refresh ');
      set(a,'windowbuttonmotionfcn','RBGSliderCrankDesign(''Move'',1)');
   elseif  Argument2 == 2
      set(a,'windowbuttonupfcn', 'a = findobj(''Tag'',''rbg_slidercrank_dsgn'');set(a,''windowbuttonmotion'','''');refresh ');
      set(a,'windowbuttonmotionfcn','RBGSliderCrankDesign(''Move'',2)');
   elseif  Argument2 == 3
      set(a,'windowbuttonupfcn', 'a = findobj(''Tag'',''rbg_slidercrank_dsgn'');set(a,''windowbuttonmotion'','''');refresh ');
      set(a,'windowbuttonmotionfcn','RBGSliderCrankDesign(''Move'',3)');
   elseif  Argument2 == 5
      set(a,'windowbuttonupfcn', 'a = findobj(''Tag'',''rbg_slidercrank_dsgn'');set(a,''windowbuttonmotion'','''');refresh ');
      set(a,'windowbuttonmotionfcn','RBGSliderCrankDesign(''Move Pos'',1)');
   elseif  Argument2 == 6
      set(a,'windowbuttonupfcn', 'a = findobj(''Tag'',''rbg_slidercrank_dsgn'');set(a,''windowbuttonmotion'','''');refresh ');
      set(a,'windowbuttonmotionfcn','RBGSliderCrankDesign(''Move Pos'',2)');
   elseif  Argument2 == 7
      set(a,'windowbuttonupfcn', 'a = findobj(''Tag'',''rbg_slidercrank_dsgn'');set(a,''windowbuttonmotion'','''');refresh ');
      set(a,'windowbuttonmotionfcn','RBGSliderCrankDesign(''Move Pos'',3)');
   elseif  Argument2 == 8
      set(a,'windowbuttonupfcn', 'a = findobj(''Tag'',''rbg_slidercrank_dsgn'');set(a,''windowbuttonmotion'','''');refresh ');
      set(a,'windowbuttonmotionfcn','RBGSliderCrankDesign(''Move Ang'',1)');
   elseif  Argument2 == 9
      set(a,'windowbuttonupfcn', 'a = findobj(''Tag'',''rbg_slidercrank_dsgn'');set(a,''windowbuttonmotion'','''');refresh ');
      set(a,'windowbuttonmotionfcn','RBGSliderCrankDesign(''Move Ang'',2)');
   elseif  Argument2 == 10
      set(a,'windowbuttonupfcn', 'a = findobj(''Tag'',''rbg_slidercrank_dsgn'');set(a,''windowbuttonmotion'','''');refresh ');
      set(a,'windowbuttonmotionfcn','RBGSliderCrankDesign(''Move Ang'',3)');
   end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The Callbacks When The User Moves the Mouse Button.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
case 'Move'
% Find out where the mouse pointer is located.
%  a = findobj('Tag','rbg_slidercrank_dsgn');
   set(h_status,'string','');
   a = findobj('Tag','rbg_slidercrank_dsgn');
   UD=a.UserData;
   bx = UD.Bx;
   by = UD.By;
   
   b = findobj('Tag','rbg_crankslider_axes');
   hdls = get(b,'userdata');
   fact=pi/180;
   theta1r=theta1*fact;
   theta2r=theta2*fact;
   theta3r=theta3*fact;
   ax=[ax1 ax2 ax3];
   ay=[ay1 ay2 ay3];
   bx=[ax1+cos(theta1r) ax2+cos(theta2r) ax3+cos(theta3r)];
   by=[ay1+sin(theta1r) ay2+sin(theta2r) ay3+sin(theta3r)];
   scale=sqrt((max(ax)-min(ax))^2+(max(ay)-min(ay))^2);
   rpivot=0.02*scale;
   delta=2*rpivot;	
   
% Identify poles using pole.m function.
   p12=pole([ax(1);ay(1)],[ax(2);ay(2)],[bx(1);by(1)],[bx(2);by(2)]);	
   p13=pole([ax(1);ay(1)],[ax(3);ay(3)],[bx(1);by(1)],[bx(3);by(3)]);
   p23=pole([ax(2);ay(2)],[ax(3);ay(3)],[bx(2);by(2)],[bx(3);by(3)]);
   pflag12=p12(3);
   pflag13=p13(3);
   pflag23=p23(3);
   pflagtot=pflag12+pflag13+pflag23;

% Identify the image pole p'23
   p23prime=ipole(p12,p13,p23);

% Find the circle of sliders relative to the coupler if the poles are finite.
   if pflagtot==0;
   	center=pole([p12(1);p12(2)],[p13(1);p13(2)],[p13(1);p13(2)],...
              [p23prime(1);p23prime(2)]);
   	x0=center(1);
   	y0=center(2);
   	dflag=center(3);
   	rc=sqrt((p12(1)-x0)^2+(p12(2)-y0)^2);
   end
   if pflagtot>0
   	rc=10^10;
   	dflag=1;
   	stline=cosline(p12,p13,p23,p23prime);
   	for j=1:1:31
   		sx(j)=stline(j,1);
   		sy(j)=stline(j,2);
   	end
   	cirangle=pi/2+atan2((sy(31)-sy(1)), (sx(31)-sx(1)));
   	x0=sx(15)+rc*cos(cirangle);
   	y0=sy(15)+rc*sin(cirangle);
   end
   
 	pt = get(b,'currentpoint');
   pt = pt(1,1:2);
   
% First set both nCheckCenter1 and nCheckCenter2 1, so that
% this program will read in two center points as default
% While Argument2 = 1 or 2 ( users try to move circle points
% around), nCheckCenter1 or nCheckCenter2 will become false (0) 
% and circle points are received as inputs and are calculated.
   
   nCheckCenter1 = 1;
   
   if Argument2 == 1
      nCheckCenter1 = 0;
   end
   
   st1=sin(theta1r);
   ct1=cos(theta1r);
       
   if nCheckCenter1 == 1 % center point input
      if Argument2 == 2
% Astar read in the mouse current cursor
         Astar(1)=pt(1); 
         Astar(2)=pt(2);
      else   
         Astar(1)=Astarx;
         Astar(2)=Astary;
      end
% Compute the first circle point.
      Values = circlepoint (ax1,ay1,theta1,ax2,ay2,theta2,ax3,ay3,...
            theta3, Astarx,Astary);
      acx(1)=Values(1);
      acy(1)=Values(2);

   else % circle input
% fx1, fy1 read in the mouse current cursor
      if Argument2 == 1
         fx1=pt(1); 
         fy1=pt(2);
      end
	   XC1= (fx1-ax1)*ct1+(fy1-ay1)*st1;
	   YC1= -(fx1-ax1)*st1+(fy1-ay1)*ct1;
	
% Compute first center point.
      Values=centerpoint(ax1,ay1,theta1,ax2,ay2,theta2,...
	       ax3,ay3,theta3,XC1,YC1);
      Astar(1) = Values(1);
      Astar(2) = Values(2);
      for ii=1:1:3
         acx(ii)=Values(2*ii+1);
		   acy(ii)=Values(2*ii+2);
      end
   end               
% Find coordinates of first bushing and pin joint.
				
   ninc=20;  
   npoints=ninc+1;
   centerptx=Astar(1);
   centerpty=Astar(2);
   coord=circle2(rpivot,centerptx,centerpty,ninc);

% find coordinates of first pin

   for i=1:1:npoints
      xcoord1(i)=coord(i,1);
      ycoord1(i)=coord(i,2);
   end

% find coordinates of first bushing

   binc=ninc/2;
   coord1=bushing(rpivot,centerptx,centerpty,binc,0);
   bpoints=ninc/2+16;
   for i=1:1:bpoints
		xbush1(i)=coord1(i,1);
		ybush1(i)=coord1(i,2);
	end

% draw the first bushing

	set(joint1,'xdata', acx(1), 'ydata',acy(1));
	set(joint2,'xdata', xcoord1, 'ydata',ycoord1);
	set(hinge1,'xdata', xbush1,'ydata', ybush1);
	set(crank1,'xdata', [acx(1) Astar(1)], 'ydata',[acy(1) Astar(2)]);

   if Argument2 == 3 % slider point input
      sldx = pt(1);
      sldy = pt(2);
   end
      a = findobj('tag','rbg_slidercrank_dsgn');
      UD=a.UserData;
      set(a,'userdata',UD);
 
        st1=sin(theta1r);
    	ct1=cos(theta1r);
      
		angle=atan2(sldy-y0, sldx-x0);
		xn=x0+rc*cos(angle);
		yn=y0+rc*sin(angle);
   
% Compute the three positions of the slider point and slider line coordinates
		Xn1= (xn-ax1)*ct1+(yn-ay1)*st1;
		Yn1= -(xn-ax1)*st1+(yn-ay1)*ct1;
					
% Determine coordinates of the slider point for three positions
		Values=sliderpoint(ax1,ay1,theta1,ax2,ay2,theta2, ax3,ay3,...
			 theta3,Xn1,Yn1);

		for ii=1:1:3
	   	asx(ii)=Values(2*ii-1);
			asy(ii)=Values(2*ii);
		end
        
 %  else
 %     asx(1) = sldx;
 %     asy(1) = sldy;
 %	end
 
   set(h_sldx,'string',num2str(sldx),'userdata',sldx);
   set(h_sldy,'string',num2str(sldy),'userdata',sldy);
   
   set(fx2,'string',num2str(asx(1)));
   set(fy2,'string',num2str(asy(1))); 
 
% Determine the coordinates of the slider line	
	lineangle=Values(7)*fact;
	length1=Values(8);
	if length1 < 2*delta; length1=2*delta; end
	ndash=20;
	csang=cos(lineangle);
	ssang=sin(lineangle);					
	xe = asx(1)-0.5*length1*csang;
	ye = asy(1)-0.5*length1*ssang;
	flag=1;
	lineangled=Values(7);
	coord=frameline(2*length1,xe,ye,ndash,lineangled,flag);					
	npoints1=3*ndash;

    
% Offset slider line by half the height of the slider block.
	for i=1:1:npoints1
		xsl1(i)=coord(i,1)+delta*ssang;
		ysl1(i)=coord(i,2)-delta*csang;
	end					
	set(sliderline1, 'xdata', xsl1, 'ydata', ysl1);
   
% draw the first slider block
	xpin=asx(1);
	ypin=asy(1);
	coords = rect(4*delta,2*delta,xpin,ypin,lineangled,0);
	for j=1:1:5
		xblock(j)=coords(j,1);
		yblock(j)=coords(j,2);
	end
	set(block1,'xdata', xblock, 'ydata',yblock);
	set(joint3,'xdata', xpin, 'ydata',ypin);	
		
% Draw the coupler
	set(coupler1,'xdata', [acx(1) asx(1)], 'ydata',[acy(1) asy(1)]);
   set(h_ax,'string',num2str(acx(1)),'userdata',acx(1));
   set(h_ay,'string',num2str(acy(1)),'userdata',acy(1));
   set(h_astarx,'string',num2str(Astar(1)),'userdata',Astar(1));
   set(h_astary,'string',num2str(Astar(2)),'userdata',Astar(2));
   set(h_sldx,'string',num2str(asx(1)),'userdata',asx(1));
   set(h_sldy,'string',num2str(asy(2)),'userdata',asy(2));

   
% ***************************************************************
% The linkage is defined. Now proceed with the analysis
% ***************************************************************

   temp=atan2((ay(1)-acy(1)), (ax(1)-acx(1)));
   phi=atan2((asy(1)-acy(1)), (asx(1)-acx(1)));
   beta1=temp-phi;
   temp=atan2((by(1)-acy(1)), (bx(1)-acx(1)));
   beta2=temp-phi;
   rc2=sqrt((by(1)-acy(1))^2+ (bx(1)-acx(1))^2);
   rc1=sqrt((ay(1)-acy(1))^2+ (ax(1)-acx(1))^2);

% Define link lengths and offset distance (r4)
   r2=sqrt((acx(1)-Astar(1))^2+ (acy(1)-Astar(2))^2);
   r3=sqrt((asx(1)-acx(1))^2+ (asy(1)-acy(1))^2);
   r4=(asy(1)-Astar(2))*cos(lineangle)-(asx(1)-Astar(1))*sin(lineangle);	
   
% display the link lengths on the screen
   set(h_r1txt,'string',num2str(r2));
   set(h_r2txt,'string',num2str(r3));
   set(h_r3txt,'string',num2str(r4));
   set(h_r4txt,'string',num2str(lineangled));
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Filemon Construction Part 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    Bstar(1) = Astar(1);    % <----------------------------------------------------------------------------------------
    Bstar(2) = Astar(2);    % <----------------------------------------------------------------------------------------
    asx(1) = acx(1);    % <--------------------------------------------------------------------------------------------
    asy(1) = acy(1);    % <--------------------------------------------------------------------------------------------
    
    delta = atan((Bstar(2)-asy(1))/(Bstar(1)-asx(1)));      % angle is in radian

    if ((Bstar(1) > asx(1)) && (Bstar(2) > asy(1)))            % alpha is in the 1st quarter area
        alpha = delta;
    end
    if ((Bstar(1) < asx(1)) && (Bstar(2) > asy(1)))            % alpha is in the 2nd quarter area
        alpha = pi+delta;
    end
    if ((Bstar(1) < asx(1)) && (Bstar(2) < asy(1)))            % alpha is in the 3rd quarter area
        alpha = pi+delta;
    end
    if ((Bstar(1) > asx(1)) && (Bstar(2) < asy(1)))            % alpha is in the 4th quarter area
        alpha = 2*pi+delta;
    end    

    % Calculation of -phi_12 and -phi_13
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Coordinate of the three positions
    a_xy=[ax1 ax2 ax3; ay1 ay2 ay3];
    theta = (pi/180)*[theta1 theta2 theta3];        % angles are in radian
    
    % Coordinate of the driven crank pivot
    x1 = asx(1);
    y1 = asy(1);
    
    x_star=Bstar(1);
    y_star=Bstar(2);
    
    % Transformation Matrix
    for i = 1;
    R = [cos(theta(i)) -sin(theta(i)) a_xy(1,i); ...
     sin(theta(i))  cos(theta(i)) a_xy(2,i); 0 0 1];
    end

    % Location of the driven crank pivot in the coupler coordi.
    B = [x1 y1 1]';         % (x1, y1)
    A = inv(R)*B;            % (X, Y)
    
    % Calculation of x2, x3 and y2, y3
    for i = 2:3
    R = [cos(theta(i)) -sin(theta(i)) a_xy(1,i); ...
     sin(theta(i))  cos(theta(i)) a_xy(2,i); 0 0 1];
    C(:,i-1) = R*A;
    i = i+1;
    end
    
    x2=C(1,1);
    y2=C(2,1);
    x3=C(1,2);
    y3=C(2,2);
    
    xy_i=[x1 x2 x3; y1 y2 y3];
    pq_i=a_xy;
    
    for i = 1:3
    cos_phi = ((xy_i(1,i)-x_star)*(pq_i(1,i)-xy_i(1,i)) + (xy_i(2,i)-y_star)*(pq_i(2,i)-xy_i(2,i)))/ ...
             sqrt(((xy_i(1,i)-x_star)^2+(xy_i(2,i)-y_star)^2)*((pq_i(1,i)-xy_i(1,i))^2+(pq_i(2,i)-xy_i(2,i))^2));
    sin_phi = ((xy_i(1,i)-x_star)*(pq_i(2,i)-xy_i(2,i)) - (xy_i(2,i)-y_star)*(pq_i(1,i)-xy_i(1,i)))/ ...
             sqrt(((xy_i(1,i)-x_star)^2+(xy_i(2,i)-y_star)^2)*((pq_i(1,i)-xy_i(1,i))^2+(pq_i(2,i)-xy_i(2,i))^2));
        
    phi_i(1,i) = 2*atan(sin_phi/(1+cos_phi));

    i=i+1;
    end
    
    phi_12 = (phi_i(2)-phi_i(1));
    phi_13 = (phi_i(3)-phi_i(1));

    minus_phi12 = -phi_12;       % the angle is in radian
    minus_phi13 = -phi_13;       % the angle is in radian
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   a = findobj('Tag','rbg_slidercrank_dsgn');
   UD=a.UserData;
   hdls = get(a,'userdata');
%###########################################################################################################
   xmin = UD.wlim(1);
   xmax = UD.wlim(2);
   ymin = UD.wlim(3);
   ymax = UD.wlim(4);
%########################################################################################################### 
    
    x = xmin:0.01:xmax;

% Calculation of the length(L) of Filemon lines
   L = sqrt(xmax^2 + ymax^2)*12;
   
% coordinates for the patch_1(for minus_phi_12) and patch_2(for minus_phi_13)

    % define assistant point for patch_1

   if abs(L*cos(alpha)) > abs(L*cos(alpha+minus_phi12))
       P_patch_1(1,1) = asx(1)+L*cos(alpha);
       P_patch_1(2,1) = asx(1)-L*cos(alpha);
   else
       P_patch_1(1,1) = asx(1)+L*cos(alpha+minus_phi12);
       P_patch_1(2,1) = asx(1)-L*cos(alpha+minus_phi12);
   end
   
   if abs(L*sin(alpha)) > abs(L*sin(alpha+minus_phi12))
       P_patch_1(1,2) = asy(1)+L*sin(alpha);
       P_patch_1(2,2) = asy(1)-L*sin(alpha);
   else
       P_patch_1(1,2) = asy(1)+L*sin(alpha+minus_phi12);
       P_patch_1(2,2) = asy(1)-L*sin(alpha+minus_phi12);
   end
        
    
   x_patch_1 = [asx(1)+L*cos(alpha) P_patch_1(1,1) asx(1)+L*cos(alpha+minus_phi12) asx(1) ...
                 asx(1)-L*cos(alpha+minus_phi12) P_patch_1(2,1) asx(1)-L*cos(alpha)];
   y_patch_1 = [asy(1)+L*sin(alpha) P_patch_1(1,2) asy(1)+L*sin(alpha+minus_phi12) asy(1) ...
                 asy(1)-L*sin(alpha+minus_phi12) P_patch_1(2,2) asy(1)-L*sin(alpha)];
    
    % define assistant point for patch_2
 
   if abs(L*cos(alpha)) > abs(L*cos(alpha+minus_phi13))
       P_patch_2(1,1) = asx(1)+L*cos(alpha);
       P_patch_2(2,1) = asx(1)-L*cos(alpha);
   else
       P_patch_2(1,1) = asx(1)+L*cos(alpha+minus_phi13);
       P_patch_2(2,1) = asx(1)-L*cos(alpha+minus_phi13);
   end
   
   if abs(L*sin(alpha)) > abs(L*sin(alpha+minus_phi13))
       P_patch_2(1,2) = asy(1)+L*sin(alpha);
       P_patch_2(2,2) = asy(1)-L*sin(alpha);
   else
       P_patch_2(1,2) = asy(1)+L*sin(alpha+minus_phi13);
       P_patch_2(2,2) = asy(1)-L*sin(alpha+minus_phi13);
   end

   x_patch_2 = [asx(1)+L*cos(alpha) P_patch_2(1,1) asx(1)+L*cos(alpha+minus_phi13) asx(1) ...
                 asx(1)-L*cos(alpha+minus_phi13) P_patch_2(2,1) asx(1)-L*cos(alpha)];
   y_patch_2 = [asy(1)+L*sin(alpha) P_patch_2(1,2) asy(1)+L*sin(alpha+minus_phi13) asy(1) ...
                 asy(1)-L*sin(alpha+minus_phi13) P_patch_2(2,2) asy(1)-L*sin(alpha)];

    set(patch_1,'xdata',x_patch_1,'ydata',y_patch_1);
    set(patch_2,'xdata',x_patch_2,'ydata',y_patch_2);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
   
%flush the draw buffer
   drawnow; 
      
case 'Move Pos'
   nCheckCenter1 = 1;
   a = findobj('Tag','rbg_slidercrank_dsgn');
   UD=a.UserData;
   axislimit = UD.wlim;
   
   b = findobj('Tag','rbg_crankslider_axes');
   pt = get(b,'currentpoint');
   pt = pt(1,1:2);
   n = Argument2;
   ax(n)=pt(1);
   ay(n)=pt(2);
   set(h_posx(n),'UserData',ax(n),'string',num2str(ax(n)));
   set(h_posy(n),'UserData',ay(n),'string',num2str(ay(n)));
      
   RBGSliderCrankDesign('Draw Axes'); 
   set(a,'userdata',UD);
   axis(axislimit);

case 'Move Ang'
   nCheckCenter1 = 1;
   a = findobj('Tag','rbg_slidercrank_dsgn');
   UD=a.UserData;
   axislimit = UD.wlim;
   
   b = findobj('Tag','rbg_crankslider_axes');
   pt = get(b,'currentpoint');
   pt = pt(1,1:2);
   n = Argument2;
   a1=pt(1);
   a2=pt(2);
   ang = atan2(pt(2)-ay(n),pt(1)-ax(n))*180/pi;
   set(h_theta(n),'UserData',ang,'string',num2str(ang));
      
   RBGSliderCrankDesign('Draw Axes'); 
   set(a,'userdata',UD);
   axis(axislimit);

case 'Change Pos'
% Set nCheckCenter1 and nCheckCenter2 1 so that the program
% will read in Astar and Bstar in the userdata as default.  
   nCheckCenter1 = 1;
%   nCheckCenter2 = 1;
% Update a set of three values posx, posy and theta every time
   n = Argument2;
   newvalx=get(h_posx(n),'string');
   newvaly=get(h_posy(n),'string');
   newval=get(h_theta(n),'string');
   ax(n)=check_val(ax(n),newvalx);
   ay(n)=check_val(ax(n),newvaly);
   theta(n)=check_val(theta(n),newval);
   set(h_posx(n),'UserData',ax(n),'string',num2str(ax(n)));
   set(h_posy(n),'UserData',ay(n),'string',num2str(ay(n)));
   set(h_theta(n),'UserData',theta(n),'string',num2str(theta(n)));
   
   RBGSliderCrankDesign('Draw Axes'); 

case 'Change Center'
% Change the center point using the keyboard input
   nCheckCenter1 = 1;
   newvalx = get(h_astarx,'string');
   newvaly = get(h_astary,'string');
   Astarx = check_val(Astarx,newvalx);
   Astary = check_val(Astary,newvaly);
   set(h_astarx,'userdata',Astarx,'string',num2str(Astarx));
   set(h_astary,'userdata',Astary,'string',num2str(Astary));
   RBGSliderCrankDesign('Draw Axes');  
  
case 'Change Circle'
% Change the circle point using the keyboard input
   nCheckCenter1 = 0;
   newvalx = get(h_ax,'string');
   newvaly = get(h_ay,'string');
   fx1 = check_val(fx1,newvalx);
   fy1 = check_val(fy1,newvaly);
   set(h_ax,'userdata',fx1,'string',num2str(fx1));
   set(h_ay,'userdata',fy1,'string',num2str(fy1));
   RBGSliderCrankDesign('Draw Axes');  
  
case 'Change Slider'
% Change the center point using the keyboard input
   nCheckCenter1 = 1;
   newvalx = get(h_sldx,'string');
   newvaly = get(h_sldy,'string');
   sldx = check_val(sldx,newvalx);
   sldy = check_val(sldy,newvaly);
   set(h_sldx,'userdata',sldx,'string',num2str(sldx));
   set(h_sldy,'userdata',sldy,'string',num2str(sldy));

   RBGSliderCrankDesign('Draw Axes');  
  
case 'Resize'
   n = Argument2;
% To retrieve current axis limits   
   a = findobj('Tag','rbg_slidercrank_dsgn');
   UD=a.UserData;
   hdls = get(a,'userdata');
% If 'Zoom out' button is pressed, downsize the figure by increasing x any y axis limits
% If 'Zoom In' button is pressed, enlarge the figure by decreasing x any y axis limits
   if n ==1
      factor = 1.05;
   else
      factor = 0.95;
   end   
   UD.wlim=factor*UD.wlim;
   a.UserData=UD;
   axis(UD.wlim);
   
case 'None'      
% the following command will bring the existing figure 
% to foreground
   a = findobj('Tag','rbg_slidercrank_dsgn');
   figure(a);
   
case 'Show Definitions'
   Show_definition('initialize','RBGCranksliderDesign.jpg'); 

case 'Close'   
% if the target window exist then close it
   if ~isempty(findobj('Tag','RBG SliderCrank Analysis Window'))
      rbg_slidercrank_ana('Stop');
      a = findobj('Tag','RBG SliderCrank Analysis Window');   
     	close(a);
   end
   
   a = findobj('Tag','rbg_slidercrank_dsgn');
   close(a);
   
case 'Analysis'
   Astar = [Astarx Astary];
   f2 = [sldx sldy];
   a = findobj('Tag','rbg_slidercrank_dsgn');
   UD=a.UserData;
   bx = UD.Bx;
   by = UD.By;

   %======================================================================================================
   fact=pi/180;
   theta1r=theta1*fact;
   theta2r=theta2*fact;
   theta3r=theta3*fact;
   ax=[ax1 ax2 ax3];
   ay=[ay1 ay2 ay3];
   bx=[ax1+cos(theta1r) ax2+cos(theta2r) ax3+cos(theta3r)];
   by=[ay1+sin(theta1r) ay2+sin(theta2r) ay3+sin(theta3r)];
   
   %======================================================================================================
   rbg_slidercrank_ana('initialize',Astar,f2,ax,ay,bx,by,theta);

case 'Help'
%-------On-line help  
   HelpStr={'SliderCrank Design for Rigid Body Guidance';
    '';
    '    "RBGSliderCrankDesign" is  a  program  to  design of slider-crank';
    'linkages by the center point, circle point, slider point, and coupler';
    'positions.   The program contains two windows: a design window and an';
    'analysis window.   The nomenclature used by the program is that given';
    'in the  textbook, Kinematics, Dynamics, and Design  of  Mechanisms by';
    'Kenneth Waldron and Gary Kinzel.';
    '';
    '    In the design window, users can  specify the center point, circle';
    'point, slider point, and three  coupler positions either by  keyboard';
    'input or the mouse input.   Users can  even drag  mouse to change the';
    'angles of coupler positions.	The convenience of the mouse usage is';
    'one of the important of this program, which creating a fairly frendly';
    'user interface.   In addition, to verify the coupler position and its';
    'associated data, three colors, red, green  and  blue,  are  utilized.';   
    'Moreover, "Zoom Out" and "Zoom In" buttons are available to scale the';
    'graphs. ';
    '';
    '    In the analysis  window, the animation  of two assembly modes are';
    'provided.   Furthermore, the animation  speed can be easily  adjusted';
    'by clicking ';
    'on the speed buttons.';
    '';
    '    Two buttons connect the two windows.   The "Return" button in the';
    'analysis window closes the analysis window and brings back the design';
    'window, while the "Analysis"  button in  the  design  window open the';
    'analysis  window  and  start the  linkage  animation  and  associated';
    'shaking force animation.  Every time the "Analysis"button is pressed,';
    'all of the design parameters are updated to start a new animation. ';
    ' '};
   helpwin(HelpStr,'Four Bar Design for Rigid Body Guidance');

case 'Get File'
% Get the file name using the UIGETFILE
   [sFileName sFilePath] = uigetfile('*.dat','Load File');
  
   if sFileName == 0 && sFilePath == 0
      return;
   end   
   
% Combine the path and file name together
   sTemp = strcat( sFilePath, sFileName );
% get the size to erase the '.dat' extension   
   sz  = size(sFileName);
% load the data file  
   load(sTemp);
% assign the parameter to a variable
   Parameter = eval(sFileName(1:(sz(2)-4)));
   if length(Parameter) == 9
      ax = Parameter(1:3);
      ay = Parameter(4:6);
      theta = Parameter(7:9);
   else
      set(h_status,'string','invalid file format');  
      return;
   end
   for n = 1:3
      set(h_posx(n),'UserData',ax(n),'string',num2str(ax(n)));
      set(h_posy(n),'UserData',ay(n),'string',num2str(ay(n)));
      set(h_theta(n),'UserData',theta(n),'string',num2str(theta(n)));
   end   
   RBGSliderCrankDesign('Draw Axes');
   refresh;
   
case 'Put File'
   [sFileName sFilePath] = uiputfile('*.dat','Save As');
	ParameterData = [ax1;ax2;ax3;ay1;ay2;ay3;theta1;theta2;theta3];
%sTemp = strcat( sFilePath, sFileName);
   [sTemp , errMsg] = sprintf('%s%s', sFilePath, sFileName);
   [strCmd, errMsg] = sprintf('%s %s %s', 'save', sTemp, ' -ascii ParameterData');
   eval(strCmd);     


   
   
end


function LocalOpenFig()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control the color of the UI
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
StdColor = get(0,'DefaultUIcontrolBackgroundColor');
%PointsPerPixel = 72/get(0,'ScreenPixelsPerInch');
bgframe = StdColor;
bgedit = [1 1 1];
bgtext = bgframe;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The Main Figure of Crank Rocker Design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ScreenSize = get(0,'ScreenSize');
sx = 480*2;   %  <----------------------------------------------------------------------------------------
sy = 400*2;
FigPos = [ 10 ScreenSize(4)-sy-95 sx sy ] ;

a = figure('Units','pixels', ...
	'Color',[0.8 0.8 0.8], ...
	'Name','SliderCrank Design Window for Rigid Body Guidance'' ', ...
	'NumberTitle','off', ...
	'Position',FigPos, ...
  	'Tag','rbg_slidercrank_dsgn');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Draw Frame First
% This will prevent the frames from been drawn on top of
% other UICONTROLS. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgframe, ...
	'Position',[0.00625 0.01 0.9833 0.1025], ...
	'Style','frame'	, ...
  	'Tag','Frame1');
   
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgframe, ...
	'Position',[0.6 0.8075 0.3896 0.18], ...
	'Style','frame', ...
	'Tag','Frame_link');
   
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgframe, ...
	'Position',[0.00625 0.12 0.58125 0.08], ...
	'Style','frame', ...
	'Tag','Frame_angle');
   
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgframe, ...
	'Position',[0.6 0.12 0.3896 0.08], ...
	'Style','frame', ...
	'Tag','Frame_pushbutton');
   
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgframe, ...
	'Position',[0.6 0.7025 0.3896 0.0975], ...
	'Style','frame', ...
	'Tag','Frame_R');
   
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgframe, ...
	'Position',[0.6 0.5975 0.3896 0.1], ...
	'Style','frame', ...
  	'Tag','Frame6');
   
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgframe, ...
	'Position',[0.6 0.4925 0.3896 0.1], ...
	'Style','frame', ...
  	'Tag','Frame6');
   
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgframe, ...
	'Position',[0.6 0.2125 0.3896 0.2725], ...
	'Style','frame', ...
	'Tag','Frame_assemode');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% menu uicontrols
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
b = uimenu('Parent',a, ...
	'Label','Parameter', ...
	'Tag','menu_parameter');
  
c = uimenu('Parent',b, ...
	'Callback','RBGSliderCrankDesign(''Get File'')', ...
	'Label','Load Parameters', ...
	'Tag','submenu_load');
   
c = uimenu('Parent',b, ...
	'Callback','RBGSliderCrankDesign(''Put File'')', ...
	'Label','Save Parameters', ...
	'Tag','submenu_save1');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create graphics window for design input
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-------Reference axis
b = axes('Parent',a, ...
  	'Box','on', ...
  	'Color',[1 1 1], ...
	'Position',[0.00625 0.2125 0.58125 0.77], ...
  	'XColor',[0 0 0], ...
  	'XGrid','off', ...
  	'XTick',[],...
  	'YColor',[0 0 0], ...
  	'YGrid','off', ...
  	'YTick',[], ...
  	'ZColor',[0 0 0], ...
  	'ZGrid','off', ...
  	'ZTick',[]);
  
b = axes('Parent',a, ...
   'Box','off', ...
  	'DataAspectRatio',[1 1 1],...
	'CameraUpVector',[0 1 0], ...
	'CameraUpVectorMode','manual', ...
	'Position',[0.025 0.2 0.56 0.8], ...  % <----------------------------------------------------------------------
	'Tag','rbg_crankslider_axes');
%axis off;
grid;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% edit related uicontrols
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
h_posx(1) = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgedit, ...
   'ForegroundColor',[1 0 0], ...
	'Callback','RBGSliderCrankDesign(''Change Pos'',1)', ...
	'Position',[0.6706 0.365 0.105 0.0425], ...
	'String','0', ...
	'Style','edit', ...
	'UserData',0);
  
h_posx(2) = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgedit, ...
   'ForegroundColor',[0 0 1], ...
	'Callback','RBGSliderCrankDesign(''Change Pos'',2)', ...
	'Position',[0.6706 0.2925 0.105 0.0425], ...
	'String','3', ...
	'Style','edit', ...
	'UserData',3);
   
h_posx(3) = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgedit, ...
   'ForegroundColor',[0 1 0], ...
	'Callback','RBGSliderCrankDesign(''Change Pos'',3)', ...
	'Position',[0.6706 0.2275 0.105 0.0425], ...
	'String','2', ...
	'Style','edit', ...
	'Tag','R_edit(3)', ...
	'UserData',2);
   
h_posy(1) = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgedit, ...
   'ForegroundColor',[1 0 0], ...
	'Callback','RBGSliderCrankDesign(''Change Pos'',1)', ...
	'Position',[0.7730 0.365 0.105 0.0425], ...
	'String','0', ...
	'Style','edit', ...
	'UserData',0);
   
h_posy(2) = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgedit, ...
   'ForegroundColor',[0 0 1], ...
	'Callback','RBGSliderCrankDesign(''Change Pos'',2)', ...
	'Position',[0.7730 0.2925 0.105 0.0425], ...
	'String','0', ...
	'Style','edit', ...
	'UserData',0);
   
h_posy(3) = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgedit, ...
   'ForegroundColor',[0 1 0], ...
	'Callback','RBGSliderCrankDesign(''Change Pos'',3)', ...
	'Position',[0.7730 0.2275 0.105 0.0425], ...
	'String','2', ...
	'Style','edit', ...
	'UserData',2);
   
h_theta(1) = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgedit, ...
   'ForegroundColor',[1 0 0], ...
	'Callback','RBGSliderCrankDesign(''Change Pos'',1)', ...
	'Position',[0.8754 0.365 0.105 0.0425], ...
	'String','45', ...
	'Style','edit', ...
	'UserData',45);
   
h_theta(2) = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgedit, ...
   'ForegroundColor',[0 0 1], ...
	'Callback','RBGSliderCrankDesign(''Change Pos'',2)', ...
	'Position',[0.8754 0.2925 0.105 0.0425], ...
	'String','135', ...
	'Style','edit', ...
	'UserData',135);
   
h_theta(3) = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgedit, ...
   'ForegroundColor',[0 1 0], ...
	'Callback','RBGSliderCrankDesign(''Change Pos'',3)', ...
	'Position',[0.8754 0.2275 0.105 0.0425], ...
	'String','0', ...
	'Style','edit', ...
	'UserData',0);
   
h_astarx = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgedit, ...
	'Callback','RBGSliderCrankDesign(''Change Center'')', ...
	'Position',[0.7308 0.71 0.105 0.0425], ...
	'String','3', ...
	'Style','edit', ...
	'UserData',3);   

h_astary = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgedit, ...
	'Callback','RBGSliderCrankDesign(''Change Center'')', ...
	'Position',[0.8725 0.71 0.105 0.0425], ...
	'String','1.09', ...
	'Style','edit', ...
	'UserData',1.09);   

h_sldx = uicontrol('Parent',a, ...      % entered coordinate of the slider
	'Units','normalized', ...
	'BackgroundColor',bgedit, ...
	'Callback','RBGSliderCrankDesign(''Change Slider'')', ...
	'Position',[0.7308 0.53 0.105 0.023], ...
	'String','1.78', ...
	'Style','edit', ...
	'UserData',1.78);   

h_sldy = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgedit, ...
	'Callback','RBGSliderCrankDesign(''Change Slider'')', ...
	'Position',[0.8725 0.53 0.105 0.023], ...
	'String','1.82', ...
	'Style','edit', ...
	'UserData',1.82);   

h_ax = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgedit, ...
	'Callback','RBGSliderCrankDesign(''Change Circle'')', ...
	'Position',[0.7308 0.605 0.105 0.0425], ...
	'String','0.38', ...
	'Style','edit', ...
	'UserData',0.38);   

h_ay = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgedit, ...
	'Callback','RBGSliderCrankDesign(''Change Circle'')', ...
	'Position',[0.8725 0.605 0.105 0.0425], ...
	'String','3.46', ...
	'Style','edit', ...
	'UserData',3.46);   

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% push button controls
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Callback','RBGSliderCrankDesign(''Analysis'')', ...
	'Position',[0.6125 0.13 0.11875 0.06], ...
	'String','Analysis', ...
	'Tag','AnalysisButton');
   
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Callback','RBGSliderCrankDesign(''Close'')', ...
	'Position',[0.8583 0.13 0.11875 0.06], ...
	'String','Close', ...
	'Tag','CloseButon');
   
b = uicontrol('Parent',a, ...
  	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Callback','RBGSliderCrankDesign(''Help'')', ...
	'Position',[0.7354 0.13 0.11875 0.06], ...
	'String','Info', ...
	'Tag','InfoButton');
    
b = uicontrol('Parent',a, ...
  	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Callback','RBGSliderCrankDesign(''Resize'',1)', ...
   'Position',[0.209 0.1325 0.178 0.055], ...
   'userdata',1, ...
	'String','Zoom Out');
  
b = uicontrol('Parent',a, ...
  	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Callback','RBGSliderCrankDesign(''Resize'',2)', ...
	'Position',[0.3875 0.1325 0.178 0.055], ...
   'String','Zoom In');
   
b = uicontrol('Parent',a, ...
  	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Callback','RBGSliderCrankDesign(''Show Definitions'')', ...
	'Position',[0.02917 0.1325 0.178 0.055], ...
   'String','Definitions');


  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% text controls
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
h_status = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'HorizontalAlignment','left', ...
	'Position',[0.05625 0.0275 0.9083 0.0475], ...
	'String',' ', ...
	'Style','text', ...
	'Tag','status_txt');
    
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.88458 0.4125 0.0875 0.0375], ...
	'String','angle', ...
	'Style','text', ...
	'Tag','StaticText1');
   
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.61875 0.9325 0.1729 0.0425], ...
	'String','crank length', ...
	'Style','text', ...
	'Tag','StaticText4');
   
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.6208 0.8925 0.1708 0.0425], ...
	'String','coupler length', ...
	'Style','text', ...
	'Tag','StaticText2');
   
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.61875 0.8525 0.1833 0.0425], ...
	'String','offset', ...
	'Style','text', ...
	'Tag','StaticText2');
   
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.6208 0.8175 0.18125 0.0425], ...
	'String','slider angle', ...
	'Style','text', ...
	'Tag','StaticText3');
   
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.61042 0.365 0.05808 0.04], ...
	'String','pos1', ...
	'Style','text', ...
	'Tag','StaticText4');

b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.61042 0.2925 0.05808 0.04], ...
	'String','pos2', ...
	'Style','text', ...
	'Tag','StaticText4');
   
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.61042 0.2275 0.05808 0.04], ...
	'String','pos3', ...
	'Style','text', ...
	'Tag','StaticText4');
   
h_r1txt = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.8083 0.935 0.17292 0.0425], ...
	'String','3.53', ...
	'Style','text');
   
h_r2txt = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.8083 0.89 0.17292 0.0425], ...
	'String','2.15', ...
	'Style','text');
   
h_r3txt = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.8083 0.855 0.17292 0.0425], ...
	'String','0.82', ...
	'Style','text');
   
h_r4txt = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.8083 0.815 0.17292 0.0425], ...
	'String','4.34', ...
	'Style','text');
   
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.6417 0.61 0.04792 0.04], ...
	'String','A', ...
	'Style','text');
   
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.61042 0.65 0.1333 0.04], ...
	'String','circle point', ...
	'Style','text');
   
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
    'ForegroundColor',[0.1 0.1 0.8], ... 
	'BackgroundColor',bgtext, ...
	'Position',[0.61042 0.7575 0.1333 0.04], ...
	'String','center point', ...
	'Style','text');

b = uicontrol('Parent',a, ...
	'Units','normalized', ...
    'ForegroundColor',[0.1 0.1 0.8], ... 
	'BackgroundColor',bgtext, ...
	'Position',[0.61042 0.4425 0.1333 0.0325], ...
	'String','coupler point', ...
	'Style','text');

b = uicontrol('Parent',a, ...
	'Units','normalized', ...
    'ForegroundColor',[0.1 0.1 0.8], ... 
	'BackgroundColor',bgtext, ...
	'Position',[0.61042 0.5525 0.1333 0.0325], ...
	'String','slider point', ...
	'Style','text');

b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.64375 0.71 0.065 0.04], ...
	'String','Astar', ...
	'Style','text');

b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.7708 0.7575 0.04167 0.0275], ...
	'String','x', ...
	'Style','text');

b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.9125 0.7575 0.02917 0.0325], ...
	'String','y', ...
	'Style','text');

b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.7708 0.6525 0.02708 0.03], ...
	'String','x', ...
	'Style','text');

b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.9125 0.6525 0.025 0.035], ...
	'String','y', ...
	'Style','text');

b = uicontrol('Parent',a, ...           % x of the slider
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.7708 0.555 0.025 0.028], ...
	'String','x', ...
	'Style','text');

b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.9125 0.555 0.025 0.028], ...
	'String','y', ...
	'Style','text');

b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.7146 0.4125 0.02708 0.035], ...
	'String','x', ...
	'Style','text');

b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.80417 0.4125 0.02917 0.035], ...
	'String','y', ...
	'Style','text');

b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.63 0.53 0.1 0.02], ...
	'String','entered coord.', ...
	'Style','text');

b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.637 0.5 0.07 0.02], ...
	'String','slider coord.', ...
	'Style','text');

fx2 = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.7308 0.5 0.105 0.02], ...
	'String','1.26', ...
	'Style','text');

fy2 = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'BackgroundColor',bgtext, ...
	'Position',[0.8725 0.5 0.105 0.02], ...
	'String','1.99', ...
	'Style','text');

drawnow;  
% fx2 = 1.26;
% fy2 = 1.99;


hdls=[ h_astarx h_astary h_ax h_ay ...
      h_sldx h_sldy h_posx h_posy h_theta ...
	   h_r1txt h_r2txt h_r3txt h_r4txt h_status ...
		0 0 0 0 0 0 0 0 0 0 fx2 fy2];
a = findobj('Tag','rbg_slidercrank_dsgn');
set(a,'userdata',struct('hand',hdls));	
     
% Set up the vectors needed to draw the linkage

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Filemon Construction Part 2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Area color %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % patch_1(-phi_12) : light green pastel tone [.9 1 .9]
     % patch_2(-phi_13) : light red pastel tone [1 .9 .9]
     % overlap(-phi_12 & -phi_13) : light blue pastel tone
     
patch_1=patch('xdata', [], 'ydata', [],'FaceColor', [.8 1 .8], 'EdgeColor', [.9 1 .9]);
patch_2=patch('xdata', [], 'ydata', [],'FaceColor', [1 .8 .8], 'EdgeColor', [1 .9 .9]);  

% paint all 3 circles with patch function to express the forbidden regions     
     
icircle23=patch('xdata', [], 'ydata',[], 'linewidth',1.2,'FaceColor', 'y','EdgeColor','r');
icircle12=patch('xdata', [], 'ydata',[], 'linewidth',1.2,'FaceColor', 'y','EdgeColor','r');
icircle13=patch('xdata', [], 'ydata',[], 'linewidth',1.2,'FaceColor', 'y','EdgeColor','r');     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

position1=line('xdata', [], 'ydata' ,[], 'linewidth',2,'color', 'r');
position2=line('xdata', [], 'ydata' ,[], 'linewidth',2,'color', 'b');
position3=line('xdata', [], 'ydata' ,[], 'linewidth',2,'color', 'g');

sliderline1=line('xdata', [], 'ydata' ,[], 'linewidth',1,'color', 'r');

a1=line('xdata', [], 'ydata', [], 'marker', '+','markersize',10, 'color', 'r');
a2=line('xdata', [], 'ydata', [], 'marker', '+','markersize',10, 'color', 'b');
a3=line('xdata', [], 'ydata', [], 'marker', '+','markersize',10, 'color', 'g');

pole12=line('xdata', [], 'ydata',[], 'marker', '+', 'markersize', 8,'color', 'k');
pole13=line('xdata', [], 'ydata',[], 'marker', '+', 'markersize', 8,'color', 'k');
pole23=line('xdata', [], 'ydata',[], 'marker', '+', 'markersize', 8,'color', 'k');
ipole23=line('xdata', [], 'ydata',[], 'marker', '+', 'markersize', 8,'color', 'r');
		
scircle=line('xdata', [], 'ydata',[], 'linewidth',1,'color', 'k');

hinge1=line('xdata', [], 'ydata',[],'color', 'r');
crank1=line('xdata', [], 'ydata', [],'linewidth',2,'color', 'k');

coupler1=line('xdata', [], 'ydata', [],'linewidth',2,'color', 'k');
coupler2=line('xdata', [], 'ydata' ,[], 'linewidth' ,2,'color', 'k');
coupler3=line('xdata', [], 'ydata' ,[], 'linewidth' ,2,'color', 'b');

block1=line('xdata', [], 'ydata',[],'linewidth' ,2.5, 'color', 'g');
joint1=line('xdata', [], 'ydata',[], 'marker', 'o', 'markersize', 7,'color', 'k');
joint2=line('xdata', [], 'ydata', [], 'color', 'r');
joint3=line('xdata', [], 'ydata',[], 'marker', 'o', 'markersize', 7,'color', 'k');

     
hdls=[ position1 position2 position3 a1 a2 a3 ...
    pole12 pole13 pole23 ipole23 ...
    scircle icircle23 icircle12 icircle13 ...
    joint1 joint2 joint3 sliderline1 crank1 block1 ...
    coupler1 coupler2 coupler3 hinge1 patch_1 patch_2];
a = findobj('Tag','rbg_crankslider_axes');
set(a,'userdata',hdls );	

function val=check_val(oldval,newval)
% Check if the user input is number input
% if not, retrieve the previous value in userdata
val=zeros(1);
if ~isequal(length(oldval),length(str2num(newval))),
  	val=oldval;
%	set(findobj(gcf,'Tag','status_txt'),'String', ...
%   ['Warning: An invalid property value has been entered.']);
else
  	val=str2num(newval);
end
