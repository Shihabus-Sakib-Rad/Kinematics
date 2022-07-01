function []=HrCrankRockerAnalysis(Action, Argument2)% MATLAB program to produce an atlas of coupler curves for crank% rocker mechanisms.%Variables%   r1=frame length (cm)%   r2=crank length (cm)%   r3=coupler length (cm)%   r4=rocker length (cm)%   rg=grid row to animate%   r33g=horizontal grid spacing%   xg,yg=x,y coordinates of the grid points%   rr,ppheta=polar coordinates of the grid points%   cr1=coupler radius (cm) (to point D1 from the crank pin)%   cr2=coupler radius (cm) (to point D1 from the crank pin)%   beta1=coupler angle (deg) (to point D2 from the rocker pin)%   beta2=coupler angle (deg) (to point D2 from the rocker pin)%   cpad=coupler angle (rad) (to point D2 from the rocker pin)%   Q1=frame angle (deg)%   Q11=frame angle (rad)%   Q2=crank angle (rad)%   Q21=crank angle (deg)%   Q22=crank angle (deg) (vector)%   Q3,Q33=coupler angle (deg)%   Q4,Q44=rocker angle (deg)%   w2=angular velocity of the crank (rad/sec)%   w22=w2 (vector)%   w33=angular velocity of the coupler (rad/sec)%   w44=angular velocity of the rocker (rad/sec)%   cang1=angle from the line connecting the coupler point one and the crank pin (rad)%   cang2=angle from the line connecting the coupler point two and the rocker pin (rad)%   Tout=torque on output link (rocker) (N-cm)%   Tin=torque on crank pin (N-cm)%   mode,assem=assembly mode%   tt=number of cycles to animate%   dt=number of animation positions per revolution%   itotal=number of animation positions%   cycle=number of cycles as an integer%   bush1,bush2=x,y coordinates of the bushings%   cir1,cir2=x,y coordinates of the circles inside the bushings%   Bx,By=x,y coordinates of the crank pin%   Cx,Cy=x,y coordinates of the rocker pin%   Bx2,By2=x,y coordinates of the crank pointer%   Cx2,Cy2=x,y coordinates of the rocker pointer%   mx,my=x,y coordinates of the grid points%   d2r,r2d=conversion factors between radians and degrees%   i=counting variable%   ans=repitition variable%	vbx=crank pin velocity in the x direction%	vby=crank pin velocity in the y direction%	vcx=rocker pin velocity in the x direction%	vcy=rocker pin velocity in the y direction%	nhgp=# of grid points in the x direction%	nvgp=# of grid points in the y direction%	gp=starting index of grid points to animate%	hh=counter%	timesink=delay variable for animation speed%	vm=velocity magnitude of selected grid point%	nhgpoc=# of gridpoints along r3%	tl=length along gridpoints horizontal%	cxrc=distance to center of gridpoints%	spacx=spacing between gridpoints%	cxmin,cxmax,cymin,cymax=window limits based on grid points%	xmin1,xmax1,ymin1=window limits based on grid points%	xmin2,xmax2,ymin2=window limits based on linkage%	xmin,xmax,ymin,ymax=window limits%	a1=frame/crank joint x,y coordinates%	b1=crank pin x,y coordinates%	c1=rocker pin x,y coordinates%	d1=frame/rocker joint x,y coordinates%	xg=x coordinates of grid points%	ygy=y coordinates of grid points%	yg=y coordinates of selected row of grid points%	xp,yp=x,y coordinates of the mouse selected point%	rr,ppheta=polar coordinates of the selected row of grid points%	Qs=starting crank angle%	mx,my=x,y coordinates of the selected grid points throughout animation%	rep=string variable dictating a change in the spee of animation%Graphics variables%   xmin,xmax,ymin,ymax=axes limits%   h1=handle on the first axes in the window (crank angle vs. rocker angle)%   h2=handle on the second axes in the window (crank angle vs. rocker velocity)%   h3=handle on the third axes in the window (crank angle vs. input torque)%   h4=handle on the fourth axes in the window (linkage animation)%   displ=line representing the crank angle vs. rocker angle%   vel=line representing the crank angle vs. rocker velocity%   torque=line representing the crank angle vs. input torque%   dbead=bead which follows the crank angle/rocker angle throughout animation%   vbead=bead which follows the crank angle/rocker velocity throughout animation%   tbead=bead which follows the crank angle/input torque throughout animation%   bushy1=line representing the bushing at the crank hinge%   bushy2=line representing the bushing at the rocker hinge%   circ1=line representing the circle inside the bushing at the crank hinge %   circ2=line representing the circle inside the bushing at the rocker hinge%   joint1=line representing the joint at the crank pin %   joint2=line representing the joint at the rocker pin%   crank=line representing the crank %   coupler=line representing the coupler%   rocker=line representing the rocker%   couplerptr=line representing the coupler point%   couplerpt1=line from the crank pin to the coupler point one%   couplerpt2=line from the rocker pin to the coupler point two%   gpvel=line representing gridpoint velocity%   gridpt=line representing gridpoints%   gridline=line representing grid point paths%   gridpoint=line representing gridpoints throughout animation%   gline=line representing selected grid points%   gpoint=line representing selected grid point%Programglobal nCheckLengthglobal g_AnaResd2r=pi/180;r2d=180/pi;if nargin == 0 && isempty(findobj('Tag','hr_crankrocker_dsgn'))% hr_crankrocker_dsgn is being initialized	Action = 'initialize';   elseif nargin == 0 && ~isempty(findobj('Tag','hr_crankrocker_dsgn'))	  % hr_crankrocker_dsgn is already open   Action = 'None';endif ~strcmp(Action,'initialize') && ~strcmp(Action,'Help')					% retrieve the previous data from the userdata of some objects   a = findobj('Tag','hr_crankrocker_dsgn');   UD=a.UserData;   handles = UD.hand;     h_R = handles(1:3);   if nCheckLength == 0	   for i = 1:3		   r(i) = get(h_R(i),'userdata');       end   else      for i = 1:3         r(i) = str2num(get(h_R(i),'string'));      end   end   r1 = r(1);   r2 = 1;   r3 = r(2);   r4 = r(3);   h_row = handles(4:5);   xp(1) = get(h_row(1),'userdata');   xp(2) = get(h_row(2),'userdata');   if xp(1) > xp(2)      temp = xp(1);      xp(1) = xp(2);      xp(2) = temp;   end      h_col = handles(6:7);   yp(1) = get(h_col(1),'userdata');   yp(2) = get(h_col(2),'userdata');   if yp(1) > yp(2)      temp = yp(1);      yp(1) = yp(2);      yp(2) = temp;   end      h_gpl = handles(8);   tl0 = get(h_gpl,'userdata');   h_gph = handles(9);   tv0 = get(h_gph,'userdata');   h_nhgp = handles(10);   nhgp0 = get(h_nhgp,'userdata');   h_nvgp = handles(11);   nvgp0 = get(h_nvgp,'userdata');        h_vel = handles(12);   w2 = get(h_vel,'userdata');      h_r2txt = handles(14);   h_status = handles(16);   set(h_status,'string','');   b = findobj('Tag','hr_crankrocker_axe');   UD=b.UserData;   handles = UD.hand;   joint1 = handles(1);   joint2 = handles(2);   crank = handles(3);   coupler = handles(4);   rocker = handles(5);   coupline1 = handles(6);   coupline2 = handles(7);   couplerptr = handles(8);   couplerpt = handles(9);   busho1 = handles(10);   busho2 = handles(11);   circo1 = handles(12);   circo2 = handles(13);end  switch Action   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%	Initialize the User Interface%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%case  'initialize'	nCheckLength = 0;   if nargin >= 2 || ~isempty(findobj('tag','menu_page'))	   LocalOpenFig;      HrCrankRockerAnalysis('Draw Axes');   else   	show_me_logo('initialize','HrCrankRockerAnalysis');   end   case 'Draw Axes'   %Set up the Animation Grid   if nCheckLength == 1      	   x = [r1; r2; r3; r4];      y = sort(x);      if(y(1)+y(4)>y(2)+y(3))         set(h_status,'string','Linkage is non-Grashoff. Please re-enter link lengths.');         for j = 1:3            r_old = get(h_R(j),'userdata');            set(h_R(j),'string',num2str(r_old));         end         nCheckLength = 0;	      return;   	else         set(h_status,'string','');   	   for j = 1:3            r_new = get(h_R(j),'string');            set(h_R(j),'userdata',str2num(r_new));         end   	   nCheckLength = 0;      end     end			nvgp=nvgp0;	nhgp=nhgp0;	nvgpoc=nvgp+1;	tl=tl0;	tv=tv0;	cxrc=(1/2)*r3;	spacx=(1/(nhgp-1))*tl;	spacy=(1/(nvgp-1))*tv;	cxmin=cxrc-(tl/2);	cxmax=cxrc+(tl/2);	cymin=-(nvgpoc/2)*spacy;	cymax=(nvgpoc/2)*spacy;		% Locate the first position of the linkage.  In this position, the crank % and frame are aligned so that r1, r2, r3 and r4 form a triangle.  Angle% AA is between r3 and r4 and BB is between r4 and the r1.	AA=acos(((r3.^2)+(r4.^2)-((r1+r2).^2))/(2*r4*r3));	BB=asin((r3/(r1+r2))*sin(AA));	CC=AA+BB;	ax=r2*cos(CC);	ay=r2*sin(CC);		% Set the first position so that the coupler is horizontal.  The left end % is at b1 and the right end is at c1.  The pivot for r4 is at d1.  The % pivot at the base of r2 is at a1 (0,0).	b1=[ax ay];	c1=[(r3+ax) ay];	d1x=-r1*cos(CC);	d1y=-r1*sin(CC);	d1=[d1x d1y];	a1=[0 0];		% Locate the coordinates of the bushing and revolute joints.	bush1=bushing(r2/10,0,0,20,0);	cir1=circle2(r2/10,0,0,20);	bush2=bushing(r2/10,(-r1*cos(CC)),(-r1*sin(CC)),20,0);	cir2=circle2(r2/10,(-r1*cos(CC)),(-r1*sin(CC)),20);		% Find the limits for the plots	xmin1=cxmin+ax;	xmin2=d1x*.95;	xmin=min([xmin1 xmin2]);	xmax1=cxmax+ax;	xmax2=d1x*1.05;	xmax=max([xmax1 xmax2]);	ymin1=cymin+ay;	ymin2=d1y*.95;	ymin=min([ymin1 ymin2]);	ymax=cymax+ay;	xmina=min([xmin d1(1) b1(1) c1(1) min(bush1(:,1))  min(bush2(:,1))]);	xmaxa=max([xmax d1(1) b1(1) c1(1) max(bush1(:,1)) max(bush2(:,1))]);	ymina=min([ymin d1(2) b1(2) c1(2) min(bush1(:,2))  min(bush2(:,2))]);	ymaxa=max([ymax d1(2) b1(2) c1(2) max(bush1(:,2)) max(bush2(:,2))]);		% Define the axes for the first plot.	width=1.0;	height=1.0;	rangex=xmaxa-xmina;	rangey=ymaxa-ymina;	xmin=xmina-0.05*rangex;	xmax=xmaxa+0.05*rangex;	ymin=ymina-0.05*rangey;	ymax=ymaxa+0.05*rangey;	values=axisadjust(xmin, xmax, ymin, ymax, width, height);	xmin=values(1);	xmax=values(2);	ymin=values(3);	ymax=values(4);	axis equal	axis([xmin xmax ymin ymax]);    a = findobj('tag','hr_crankrocker_dsgn');    UD=a.UserData;    hdls = UD.hand;    lim = [xmin xmax ymin ymax];    set(a,'UserData',struct('hand',hdls,'wlim',lim));% Compute the grid coordinates	gxstart=cxmin-spacx;    for i=1:nhgp		xg(i)=b1(1)+gxstart+(i*spacx);    end    for i=1:nvgp		ygy(i)=b1(2)+cymin+(i*spacy);    end      % delete previous created gridpt objects   a = findobj('tag','hr_crankrocker_axe');   UD=a.UserData;   Hhdls=UD.hand;   if isfield(UD,'wlim')       Whdls = UD.wlim;       k = UD.knum;      for i = 1:k,         if Whdls(i) ~= 0      		delete(Whdls(i));         	Whdls(i) = 0;         end      end   end      for k=1:(nhgp*nvgp)		gridpt(k)=line('xdata',[],'ydata',[],'marker','.','markersize',8,'color','k');   end   set(a,'userdata',struct('hand',Hhdls,'knum',k,'wlim',gridpt));		% Analyze the linkage for each grid point   k=0;		% Draw each of the grid points on the coupler.    for i=1:nhgp        for j=1:nvgp             k=k+1;             set(gridpt(k),'xdata',xg(i),'ydata',ygy(j));             if i>=xp(1) && i<=xp(2) && j>=yp(1) && j<=yp(2)                set(gridpt(k),'marker','o','markersize',6 );                refresh;             end        end    end	drawnow;    % Draw the linkage	set(crank,'xdata',[0 b1(1)],'ydata',[0 b1(2)]);	set(coupler,'xdata',[b1(1) c1(1)],'ydata',[b1(2) c1(2)]);	set(rocker,'xdata',[c1(1) d1(1)],'ydata',[c1(2) d1(2)]);	set(joint1,'xdata', b1(1), 'ydata',b1(2));	set(joint2,'xdata', c1(1), 'ydata',c1(2));	set(busho1,'xdata',bush1(:,1),'ydata',bush1(:,2));	set(circo1,'xdata',cir1(:,1),'ydata',cir1(:,2));	set(busho2,'xdata',(bush2(:,1)),'ydata',(bush2(:,2)));	set(circo2,'xdata',(cir2(:,1)),'ydata',(cir2(:,2)));   case 'Change R'	n = Argument2;	newval=get(h_R(n),'string');	r(n)=check_val(r(n),newval);   nCheckLength = 1;	HrCrankRockerAnalysis('Draw Axes');    case 'Change Row'	n = Argument2;   	newval=get(h_row(n),'string');    xp(n)=check_val(xp(n),newval);    nhgp0 = get(h_nhgp,'userdata');    if xp(n) > nhgp0        set(h_status,'string','Exceed the maximum horizontal range. Please reenter the parameter');        xp_old = get(h_row(n),'userdata');        set(h_row(n),'string',num2str(xp_old));        return    end    set(h_row(n),'UserData',xp(n),'string',num2str(xp(n)));    for i = 1:2        xp(i) = get(h_row(i),'userdata');    end    a = findobj('tag','hr_crankrocker_axe');    UD=a.UserData;    gridpt = UD.wlim;     x1 = min(xp);    x2 = max(xp);    y1 = min(yp);    y2 = max(yp);    k = 0;    for i=1:nhgp0        for j=1:nvgp0            k=k+1;            set(gridpt(k),'marker','.','markersize',8);            if i>=x1 && i<=x2 && j>=y1 && j<=y2                set(gridpt(k),'marker','o','markersize',6 );                refresh;            end        end    end   case 'Change Column'	n = Argument2;   	newval=get(h_col(n),'string');	yp(n)=check_val(yp(n),newval);   nvgp0 = get(h_nvgp,'userdata');   if yp(n) > nvgp0      set(h_status,'string','Exceed the maximum vertical range. Please reenter the parameter');      yp_old = get(h_col(n),'userdata');      set(h_col(n),'string',num2str(yp_old));      return   end	set(h_col(n),'UserData',yp(n),'string',num2str(yp(n)));   for i = 1:2      yp(i) = get(h_col(i),'userdata');   end   a = findobj('tag','hr_crankrocker_axe');   UD=a.UserData;   k = UD.knum;   gridpt = UD.wlim;    x1 = min(xp);   x2 = max(xp);   y1 = min(yp);   y2 = max(yp);   k = 0;	for i=1:nhgp0		for j=1:nvgp0			k=k+1;                  set(gridpt(k),'marker','.','markersize',8);         if i>=x1 && i<=x2 && j>=y1 && j<=y2            set(gridpt(k),'marker','o','markersize',6 );            refresh;         end		end   end   case 'Change Grid Length'	newval=get(h_gpl,'string');	tl0=check_val(tl0,newval);	set(h_gpl,'UserData',tl0,'string',num2str(tl0));	HrCrankRockerAnalysis('Draw Axes');     case 'Change Grid Height'	newval=get(h_gph,'string');	tv0=check_val(tv0,newval);	set(h_gph,'UserData',tv0,'string',num2str(tv0));	HrCrankRockerAnalysis('Draw Axes');     case 'Change HGrid No'	newval=get(h_nhgp,'string');   nhgp0=check_val(nhgp0,newval);   xval = max(xp);   if nhgp0 < xval      set(h_status,'string','The animation range exceed the total grid number. Please reenter this parameter or change the animation range');      oldval = get(h_nhgp,'userdata');      set(h_nhgp,'string',num2str(oldval));      return   end	set(h_nhgp,'UserData',nhgp0,'string',num2str(nhgp0));	HrCrankRockerAnalysis('Draw Axes');     case 'Change VGrid No'	newval=get(h_nvgp,'string');   nvgp0=check_val(nvgp0,newval);   yval = max(yp);   if nvgp0 < yval      set(h_status,'string','The animation range exceed the total grid number. Please reenter this parameter or change the animation range');      oldval = get(h_nvgp,'userdata');      set(h_nvgp,'string',num2str(oldval));      return   end	set(h_nvgp,'UserData',nvgp0,'string',num2str(nvgp0));	HrCrankRockerAnalysis('Draw Axes');     case 'Change Velocity'   	newval=get(h_vel,'string');	w2=check_val(w2,newval);	set(h_vel,'UserData',w2);	set(h_vel,'string',num2str(w2));%	HrCrankRockerAnalysis('Draw Axes');  case 'None'      	% the following command will bring the existing figure 	% to foreground	a = findobj('Tag','hr_crankrocker_dsgn');	figure(a);   case 'Close'   	% if the target window exist then close it	if ~isempty(findobj('Tag','Fourbar Analysis Window'))	   fourbar_ana('Stop');   	a = findobj('Tag','Fourbar Analysis Window');   	  	close(a);	end      if ~isempty(findobj('Tag','CR Coupler Curve Animation Window'))	   hr_cr_animation('Stop');   	a = findobj('Tag','CR Coupler Curve Animation Window');   	  	close(a);	end	a = findobj('Tag','hr_crankrocker_dsgn');	close(a);   case 'Show Definitions'   Show_definition('initialize','HrCrankRocker.jpg');   case 'Resize'   n = Argument2;% To retrieve current axis limits      a = findobj('Tag','hr_crankrocker_dsgn');   UD=a.UserData;% If 'Zoom out' button is pressed, downsize the figure by increasing x any y axis limits% If 'Zoom In' button is pressed, enlarge the figure by decreasing x any y axis limits   if n ==1      factor = 1.05;   else      factor = 0.95;   end   UD.wlim=factor*UD.wlim;   a.UserData=UD;   axis(UD.wlim);   case 'Animation'   if xp(1)>nhgp0 || xp(2)>nhgp0 || yp(1)>nvgp0 || yp(2)>nvgp0      set(h_status,'string','Please check the range for coupler curve animation');      return   end   hr_cr_animation('initialize',r,tl0,tv0,nhgp0,nvgp0,w2,xp,yp);case 'Help'%-------On-line help     HelpStr={'Crank Rocker Coupler Curve Analysis';    '';    '    "HrCrankRockerAnalysis"  is a  program  to produce  an atlas of coupler';    'curves for crank rocker mechanism and to analyze the results.   The program';    'contains three windows:  a  design  window,  an  animation  window  and  an';    'analysis window.  The nomenclature used by the program is that given in the';    'textbook,  Kinematics, Dynamics, and Design of Mechinery by Kenneth Waldron';    'and Gary Kinzel.';    '';    '    In the design window, the  variables are the  three link length (frame,';    'coupler, rocker) and the angular  velocity of the driver.  More options are';    'given to present a uniform grid of coupler points, incuding the grid length';    'and  grid height, and the total number of grids in both directions.  One of';    'the program''s  features is that  users can specify the  animation range in';    'both directions.  The selected coupler points will change their marker into';     'hollow circles. In addition, "zoom in" and "zoom out" buttons are available';    'to downsize and upsize the mechanism plot.';    '';    '    In the  animation  window, selected  coupler  points are animated.  The';    '"Analysis" button is not available  during the animation.  Once users click';    'on the  desired coupler point in the "Stop" mode, the "Analysis" button can';    'be activated to open the analysis window.  Furthermore, the animation speed';    'can be easily adjusted by clicking on the speed buttons.';    '';    '    In the  analysis window, users can  control the number  of plots (up to';    'four) and the contents of each plot  (four  options  are  provided).    The';    '"Speed" buttons are also available for users to control the animation speed';    '';    '    Several  buttons connect the three windows.  The "Return" button in the';    'animation  window will close  both the  animation and  analysis windows and';    'brings  back the  design window, while  the "Return" button in the analysis';    'window only close  the analysis window and bring back the animation window.';    'The "Animation"  button in the  design window opens the  animation  window.';    'Then the  analysis window can be  open through the "Analysis" button in the';    'animation window.';    '';    '    There  is a  status bar at  the  bottom of the  design  window  and the';    'animation  windows.  Information  about an instruction or an error  message';    'will be shown in the status box. ';    ' '};   helpwin(HelpStr,'Crank Rocker Coupler Curve Analysis');case 'Get File'	% Get the file name using the UIGETFILE	[sFileName sFilePath] = uigetfile('*.dat','Load File');  	if sFileName == 0 && sFilePath == 0	   return;	end      	% Combine the path and file name together	sTemp = strcat( sFilePath, sFileName );	% get the size to erase the '.dat' extension   	sz  = size(sFileName);	% load the data file  	load(sTemp);	% assign the parameter to a variable	Parameter = eval(sFileName(1:(sz(2)-4)));	if length(Parameter) == 8   	r(1:3) = Parameter(1:3);	   w2 = Parameter(4);   	tl0 = Parameter(5);	   tv0 = Parameter(6);	   nhgp0 = Parameter(7);   	nvgp0 = Parameter(8);	for i=1:3 		set(h_R(i),'UserData',r(i),'string',num2str(r(i)));   end    	set(h_vel,'Userdata',w2,'string',num2str(w2));	set(h_gpl,'Userdata',tl0,'string',num2str(tl0));	set(h_gph,'Userdata',tv0,'string',num2str(tv0));	set(h_nhgp,'Userdata',nhgp0,'string',num2str(nhgp0));	set(h_nvgp,'Userdata',nvgp0,'string',num2str(nvgp0));	HrCrankRockerAnalysis('Draw Axes');         refresh;else   set(h_status,'string','invalid file format');     return;end             case 'Put File'	[sFileName sFilePath] = uiputfile('*.dat','Save As');	if Argument2 == 1   		ParameterData = [r1; r3; r4; w2; tl0; tv0; nhgp0; nvgp0];	else   	ParameterData = g_AnaRes;	end	%sTemp = strcat( sFilePath, sFileName);	[sTemp , errMsg] = sprintf('%s%s', sFilePath, sFileName);	[strCmd, errMsg] = sprintf('%s %s %s', 'save', sTemp, ' -ascii ParameterData');	eval(strCmd);   end   function LocalOpenFig()%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Control the color of the UI%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%StdColor = get(0,'DefaultUIcontrolBackgroundColor');%PointsPerPixel = 72/get(0,'ScreenPixelsPerInch');bgframe = StdColor;bgedit = [1 1 1];bgtext = bgframe;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% The Main Figure of Crank Rocker Design%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%ScreenSize = get(0,'ScreenSize');sx = 480;sy = 400;FigPos = [ 10 ScreenSize(4)-sy-95 sx sy ] ; a = figure('Units','pixels', ...	'Color',[0.8 0.8 0.8], ...	'Name','Crank Rocker Coupler Curve Window', ...	'NumberTitle','off', ...	'Position',FigPos, ...  	'Tag','hr_crankrocker_dsgn');  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Draw Frame First% This will prevent the frames from been drawn on top of% other UICONTROLS. %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgframe, ...	'Position',[0.00625 0.01 0.9833 0.11], ...	'Style','frame'	, ...  	'Tag','Frame_status');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgframe, ...	'Position',[0.6 0.715 0.3896 0.275], ...	'Style','frame', ...	'Tag','Frame_link');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgframe, ...	'Position',[0.6 0.4025 0.3896 0.2275], ...	'Style','frame', ...	'Tag','Frame_R');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgframe, ...	'Position',[0.00625 0.1275 0.58125 0.0775], ...	'Style','frame', ...	'Tag','Frame_angle');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgframe, ...	'Position',[0.6 0.1275 0.3896 0.0775], ...	'Style','frame', ...	'Tag','Frame_pushbutton');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgframe, ...	'Position',[0.6 0.215 0.3896 0.175], ...	'Style','frame', ...	'Tag','Frame_showanimation');b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgframe, ...	'Position',[0.6 0.64 0.3896 0.065], ...	'Style','frame', ...	'Tag','Frame_vel');      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% menu uicontrols%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%b = uimenu('Parent',a, ...	'Label','Parameter', ...	'Tag','menu_parameter');  c = uimenu('Parent',b, ...	'Callback','HrCrankRockerAnalysis(''Get File'')', ...	'Label','Load Parameters', ...	'Tag','submenu_load');   c = uimenu('Parent',b, ...	'Callback','HrCrankRockerAnalysis(''Put File'',1)', ...	'Label','Save Parameters', ...	'Tag','submenu_save1');%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Create graphics window for design input%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%-------Reference axisb = axes('Parent',a, ...  	'Box','on', ...  	'Color',[1 1 1], ...	'Position',[0.00625 0.215 0.58125 0.775], ...  	'XColor',[0 0 0], ...  	'XGrid','off', ...  	'XTick',[],...  	'YColor',[0 0 0], ...  	'YGrid','off', ...  	'YTick',[], ...  	'ZColor',[0 0 0], ...  	'ZGrid','off', ...  	'ZTick',[]);  b = axes('Parent',a, ...   'Box','off', ...  	'DataAspectRatio',[1 1 1],...	'CameraUpVector',[0 1 0], ...	'CameraUpVectorMode','manual', ...	'Position',[0.00625 0.215 0.58125 0.775], ...	'Tag','hr_crankrocker_axe');axis off;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% edit related uicontrols%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%h_R(1) = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgedit, ...	'Callback','HrCrankRockerAnalysis(''Change R'',1)', ...	'Position',[0.8354 0.915 0.1375 0.05], ...	'String','3.304', ...	'Style','edit', ...	'Tag','R_edit(1)', ...	'UserData',3.304);  h_R(2) = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgedit, ...	'Callback','HrCrankRockerAnalysis(''Change R'',2)', ...	'Position',[0.8354 0.855 0.1375 0.05], ...	'String','2.913', ...	'Style','edit', ...	'Tag','R_edit(2)', ...	'UserData',2.913);   h_R(3) = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgedit, ...	'Callback','HrCrankRockerAnalysis(''Change R'',3)', ...	'Position',[0.8354 0.795 0.1375 0.05], ...	'String','1.565', ...	'Style','edit', ...	'Tag','R_edit(3)', ...	'UserData',1.565);   h_row(1) = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgedit, ...	'Callback','HrCrankRockerAnalysis(''Change Row'',1)', ...	'Position',[0.7333 0.2875 0.06875 0.05], ...	'String','1', ...	'Style','edit', ...	'Tag','start_row_edit', ...	'UserData',1);   h_row(2) = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgedit, ...	'Callback','HrCrankRockerAnalysis(''Change Row'',2)', ...	'Position',[0.9041 0.2875 0.06875 0.05], ...	'String','5', ...	'Style','edit', ...	'Tag','stop_row_edit', ...	'UserData',5);   h_col(1) = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgedit, ...	'Callback','HrCrankRockerAnalysis(''Change Column'',1)', ...	'Position',[0.7333 0.2325 0.06875 0.05], ...	'String','5', ...	'Style','edit', ...	'Tag','start_col_edit', ...	'UserData',5);   h_col(2) = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgedit, ...	'Callback','HrCrankRockerAnalysis(''Change Column'',2)', ...	'Position',[0.9041 0.2325 0.06875 0.05], ...	'String','5', ...	'Style','edit', ...	'Tag','stop_col_edit', ...	'UserData',5);   h_vel = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgedit, ...	'Callback','HrCrankRockerAnalysis(''Change Velocity'')', ...	'Position',[0.8354 0.6475 0.1375 0.05], ...	'String','5', ...	'Style','edit', ...	'Tag','vel_edit', ...	'UserData',5);      h_gpl = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgedit, ...	'Callback','HrCrankRockerAnalysis(''Change Grid Length'')', ...	'Position',[0.8354 0.5725 0.1375 0.05], ...	'String','11.652', ...	'Style','edit', ...	'Tag','gth_edit', ...	'UserData',11.652);      h_gph = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgedit, ...	'Callback','HrCrankRockerAnalysis(''Change Grid Height'')', ...	'Position',[0.8354 0.5175 0.1375 0.05], ...	'String','5.826', ...	'Style','edit', ...	'Tag','gtl_edit', ...	'UserData',5.826);      h_nhgp = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgedit, ...	'Callback','HrCrankRockerAnalysis(''Change HGrid No'')', ...	'Position',[0.8354 0.4625 0.1375 0.05], ...	'String','5', ...	'Style','edit', ...	'Tag','ngtr_edit', ...	'UserData',5);      h_nvgp = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgedit, ...	'Callback','HrCrankRockerAnalysis(''Change VGrid No'')', ...	'Position',[0.8354 0.41 0.1375 0.05], ...	'String','10', ...	'Style','edit', ...	'Tag','ngtl_edit', ...	'UserData',10);      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% push button controls%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Callback','HrCrankRockerAnalysis(''Animation'')', ...	'Position',[0.6083 0.1415 0.1333 0.05], ...	'String','Animation', ...	'Tag','AnalysisButton');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Callback','HrCrankRockerAnalysis(''Close'')', ...	'Position',[0.86 0.1415 0.12 0.05], ...	'String','Close', ...	'Tag','CloseButon');   b = uicontrol('Parent',a, ...  	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Callback','HrCrankRockerAnalysis(''Help'')', ...	'Position',[0.7416 0.1415 0.11875 0.05], ...	'String','Info', ...	'Tag','InfoButton');    b = uicontrol('Parent',a, ...  	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Callback','HrCrankRockerAnalysis(''Resize'',1)', ...   'Position',[0.209 0.1415 0.178 0.05], ...   'userdata',1, ...	'String','Zoom Out');  b = uicontrol('Parent',a, ...  	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Callback','HrCrankRockerAnalysis(''Resize'',2)', ...	'Position',[0.3875 0.1415 0.178 0.05], ...   'String','Zoom In');   b = uicontrol('Parent',a, ...  	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Callback','HrCrankRockerAnalysis(''Show Definitions'')', ...	'Position',[0.02917 0.1415 0.178 0.05], ...   'String','Definitions');   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% text controls%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%h_status = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'HorizontalAlignment','left', ...	'Position',[0.04121 0.015 0.9086 0.09], ...	'String',' ', ...	'Style','text', ...	'Tag','status_txt');    b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.6396 0.9125 0.1708 0.0425], ...	'String','frame length', ...	'Style','text', ...	'Tag','StaticText4');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.6396 0.74 0.1708 0.0425], ...	'String','crank length', ...	'Style','text', ...	'Tag','StaticText2');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.6396 0.8575 0.1708 0.0425], ...	'String','coupler length', ...	'Style','text', ...	'Tag','StaticText2');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.6396 0.7975 0.1708 0.0425], ...	'String','rocker length', ...	'Style','text', ...	'Tag','StaticText3');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.6083 0.6475 0.2033 0.0425], ...	'String','angular velocity', ...	'Style','text', ...   	'Tag','StaticText4');b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.6083 0.5725 0.1708 0.0425], ...	'String','grid length', ...	'Style','text', ...   	'Tag','StaticText4');b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.6083 0.5175 0.1708 0.0425], ...	'String','grid height', ...	'Style','text', ...   	'Tag','StaticText4');b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.6021 0.4625 0.2333 0.0425], ...	'String','no. of points in row', ...	'Style','text', ...   	'Tag','StaticText4');b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.6021 0.4175 0.2333 0.0425], ...	'String','no. of points in col.', ...	'Style','text', ...   	'Tag','StaticText4');b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.6125 0.2875 0.1167 0.0425], ...	'String',' from col.', ...	'Style','text');b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.8125 0.2875 0.0815 0.0425], ...	'String',' to col.', ...	'Style','text');b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.6125 0.2325 0.1167 0.0425], ...	'String','from row', ...	'Style','text');b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.8125 0.2325 0.0815 0.0425], ...	'String','to row', ...	'Style','text');h_r2txt = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...   'Position',[0.83542 0.7375 0.1333 0.0425], ...	'String','1', ...	'Style','text', ...   	'Tag','r2_txt');b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.69 0.34 0.2271 0.0425], ...	'String','animation range', ...	'Style','text', ...   	'Tag','StaticText4');hdls = [h_R h_row h_col h_gpl h_gph ...		h_nhgp h_nvgp h_vel 0 h_r2txt 0 h_status];a = findobj('tag','hr_crankrocker_dsgn');set(a,'userdata',struct('hand',hdls));joint1=line('xdata', [], 'ydata', [], 'marker', 'o','markersize',6, 'color', 'k');joint2=line('xdata', [], 'ydata', [], 'marker', 'o','markersize',6, 'color', 'k');crank=line('xdata', [], 'ydata' ,[], 'linewidth',2,'color', 'k');rocker=line('xdata', [], 'ydata' ,[], 'linewidth',2,'color', 'k');coupler=line('xdata', [], 'ydata' ,[], 'linewidth',2,'color', 'k');coupline1=line('xdata', [], 'ydata' ,[], 'linewidth' ,1,'color', 'k');coupline2=line('xdata', [], 'ydata' ,[], 'linewidth' ,1,'color','k');couplerptr=line('xdata', [], 'ydata' ,[], 'linewidth' ,1,'color', 'k','linestyle',':');	     couplerpt=line('xdata', [], 'ydata' ,[], 'linewidth' ,1,'color', 'k');	   busho1=line('xdata',[],'ydata',[],'linewidth',1,'linestyle','-','color','r');circo1=line('xdata',[],'ydata',[],'linewidth',1,'linestyle','-','color','r');busho2=line('xdata',[],'ydata',[],'linewidth',1,'linestyle','-','color','r');circo2=line('xdata',[],'ydata',[],'linewidth',1,'linestyle','-','color','r');hdls = [joint1 joint2 crank coupler rocker coupline1 coupline2 ...      couplerptr couplerpt busho1 busho2 circo1 circo2 ];a = findobj('tag','hr_crankrocker_axe');set(a,'userdata',struct('hand',hdls));function val=check_val(oldval,newval)% Check if the user input is number input% if not, retrieve the previous value in userdataval=zeros(1);if ~isequal(length(oldval),length(str2num(newval))),  	val=oldval;%	set(findobj(gcf,'Tag','status_txt'),'String', ...%   ['Warning: An invalid property value has been entered.']);else  	val=str2num(newval);end