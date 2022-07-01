function []=SliderCrankAnalysis(Action,Argument2)% slidercrank_anal.m% MATLAB program for analyzing slider-crank mechanisms%Variables%   r1=frame length%   r2=crank length%   r3=coupler length%   r4=rocker length%   cr1=coupler radius (to point D1 from the crank pin)%   beta1=coupler angle (deg) (to point D2 from the rocker pin)%   cpad=coupler angle (rad) (to point D2 from the rocker pin)%   Q1=frame angle (deg)%   Q11=frame angle (rad)%   Q2=crank angle (rad)%   Q21=crank angle (deg)%   Q22=crank angle (deg) (vector)%   Q3,Q33=coupler angle (deg)%   Q4,Q44=rocker angle (deg)%   w2=angular velocity of the crank (rad/sec)%   w22=w2 (vector)%   w33=angular velocity of the coupler (rad/sec)%   w44=angular velocity of the rocker (rad/sec)%   cang1=angle from the line connecting the coupler point one and the crank pin (rad)%   cang2=angle from the line connecting the coupler point two and the rocker pin (rad)%   mode,assem=assembly mode%   tt=number of cycles to animate%   dt=number of animation positions per revolution%   itotal=number of animation positions%   cycle=number of cycles as an integer%   bush1,bush2=x,y coordinates of the bushings%   cir1,cir2=x,y coordinates of the circles inside the bushings%   Bx,By=x,y coordinates of the crank pin%   Cx,Cy=x,y coordinates of the rocker pin%   Bx2,By2=x,y coordinates of the crank pointer%   Cx2,Cy2=x,y coordinates of the rocker pointer%   d2r,r2d=conversion factors between radians and degrees%   i=counting variable%   ans=repitition variable%Graphics variables%   xmin,xmax,ymin,ymax=axes limits%   h1=handle on the first axes in the window (crank angle vs. slider displacement)%   h2=handle on the second axes in the window (crank angle vs. slider velocity)%   h3=handle on the third axes in the window (crank angle vs. input torque)%   h4=handle on the fourth axes in the window (linkage animation)%   displ=line representing the crank angle vs. rocker angle%   vel=line representing the crank angle vs. rocker velocity%   dbead=bead which follows the crank angle/slider displacement throughout animation%   vbead=bead which follows the crank angle/slider velocity throughout animation%   tbead=bead which follows the crank angle/input torque throughout animation%   bushy1=line representing the bushing at the crank hinge%   circ1=line representing the circle inside the bushing at the crank hinge %   joint1=line representing the joint at the crank pin %   joint2=line representing the joint at the rocker pin%   crank=line representing the crank %   coupler=line representing the coupler%   couplerptr=line representing the coupler point%   couplerpt1=line from the crank pin to the coupler point one% Program global nBetaflagglobal nCradiusflagglobal g_AnaResif nargin==0 && isempty(findobj('Tag','slidercrank_dsgn'))	Action='initialize';elseif nargin==0 && ~isempty(findobj('Tag','slidercrank_dsgn'))	Action='None';endif ~strcmp(Action,'initialize') && ~strcmp(Action,'Help')	a=findobj('Tag','slidercrank_dsgn');	%To retrieve handles for uicontrols	    UD=a.UserData;    hdls = UD.hand;    h_beta = hdls(1);	h_Q = hdls(2);	h_R = hdls(3:6);	h_vel = hdls(7);	h_Dradio = hdls(8:10);	h_Mradio = hdls(11:12);	h_ASradio = hdls(13:14);	h_status = hdls(15);	beta = get(h_beta,'userdata');   Q1 = get(h_Q,'userdata'); %slider line angle   for i = 1 : 4      r(i) = get(h_R(i),'userdata');   end   	r2 = r(1);%crank length	r3 = r(2);%coupler length	r4 = r(3); %slider-line offset	cr1 = r(4);%coupler point radius	if get(h_Dradio(1),'value') == 1   	driver = 1;	elseif get(h_Dradio(2),'value') == 1   	driver = 2;	else   	driver = 3;	end	if get(h_Mradio(1),'value') == 1	   times = 1;	else   	times = 2;	end     	if get(h_ASradio(1),'value') == 1   	mode = 1;	else   	mode = -1;	end            vel = get(h_vel,'userdata');	if driver == 1	   w2 = vel;	elseif driver == 2   	w3 = vel;	else   	r1d = vel;    end       b=findobj('Tag','slidercrank_axe');    UD=b.UserData;	hdls = UD.hand;	joint1 = hdls(1);       	joint2 = hdls(2);       	crank = hdls(3);       	coupler = hdls(4);       	couplerptr = hdls(5);       	couplerpoint1 = hdls(6);       	couplerpoint2 = hdls(7);       	gridpt = hdls(8);       	block = hdls(9);       	sliderline1 = hdls(10);       	bush1 = hdls(11);       	circ1 = hdls(12);       endswitch Actioncase'None'	findobj('Tag','slidercrank_dsgn');case 'initialize'	nBetaflag=1;	nCradiusflag=1;   if nargin >= 2 || ~isempty(findobj('tag','menu_page'))	   LocalOpenFig;      SliderCrankAnalysis('Draw Axes');   else   	show_me_logo('initialize','SliderCrankAnalysis');   end   case 'Draw Axes'		i=0;	d2r=pi/180;	r2d=180/pi;	Q11=Q1*d2r;	cpad=beta*d2r;	qstart=0;	qstop=360;	if driver==1; [qstart, qstop]=sc_angle_limits_cr(r2,r3,r4,Q1); end	if driver==2; [qstart, qstop]=sc_angle_limits_co(r2,r3,r4,Q1); end	if driver==3; [qstart, qstop]=sc_angle_limits_sl(r2,r3,r4,Q1); end	npos=100;	dq=(qstop-qstart)/(npos-1);	Astar=[0;0];	rpivot=0.15*r2;	delta=0.3*r2;	QM1=qstart;		i2=0;% Loop to analyze the linkage for tt full cycles	for mmm=1:1:times% Analyze for two cycles in different modes if driver is 2 or 3		if mmm ==2			mode = -mode;			dq=-dq;		end		for motion=1:1:npos			if motion>1; QM1=QM1+dq; end			i=i+1;			QM = QM1*d2r;% Call function to compute the position			if driver ==1; para = sldcrkc(r2,r3,r4,QM1,w2,0,mode,Q1,3); end			if driver ==2; para = sldcrkco(r2,r3,r4,QM1,w3,0,mode,Q1,3); end				if driver ==3; para = sldcrks(QM1,r2,r3,r4,r1d,0,mode,Q1,3); end					Q2=para(6);			if Q2<0; Q2=Q2+360; end			Q22(i)=Q2;			w22(i)=para(14);			Q3=para(7);			Q33(i)=Q3;			w33(i)=para(15);			r1(i)=para(1);			rd1(i)=para(9);			assem=para(37);			cang1=(Q3+beta)*d2r;% Store position information in matrices.			Bx(i)= para(25);			By(i)= para(26);			Cx(i)= para(27);			Cy(i)= para(28);			Dx1(i)=Bx(i)+cr1*cos(cang1);			Dy1(i)=By(i)+cr1*sin(cang1);			% Determine coordinates for coupler curve for first mode						if mmm==1				Dx2(i)=Dx1(i);				Dy2(i)=Dy1(i);			end			% Determine coordinates for coupler curve for second mode						if mmm==2				i2=i2+1;				Dx3(i2)=Dx1(i);				Dy3(i2)=Dy1(i);			end		% Locate the slider block coordinates			xpin=Cx(i);			ypin=Cy(i);			height=delta;			length=2*height;			coords = rect(length,height,xpin,ypin,Q1,0);			for j=1:1:5				xblocka(i,j)=coords(j,1);				yblocka(i,j)=coords(j,2);			end% Prepare the result data and store them in the global variable g_AnaRes% Those result data can be save as output data while menu 'save result data'% is choosen.		Q222 = Q22';		Q333 = Q33';		r11 = r1';		w333 = w33';		rd11 = rd1';		g_AnaRes = [Q222 Q333 r11 w333 rd11];				end	end	itotal=i;% Determine the coordinates of the slider line					smax=max(r1);	smin=min(r1);	[cmin, index]=min(Cx);	cymin=Cy(index);	length1=smax-smin+length;	ndash=20;	csang=cos(Q11);	ssang=sin(Q11);		xe = cmin-(0.5*length)*csang;	ye = cymin-(0.5*length)*ssang;	slflag=1;	coord=frameline(length1,xe,ye,ndash,Q1,slflag);						npoints1=3*ndash;					% Offset slider line by half the height of the slider block.						for i=1:1:npoints1		xsl1(i)=coord(i,1)+(0.6*height)*ssang;		ysl1(i)=coord(i,2)-(0.6*height)*csang;	end						width=0.95;	height=0.95;	   % Define the pin and bushing coordinates		bush=bushing(r2/20,0,0,20,0);	cir1=circle2(r2/20,0,0,20);% Find the limits for the plots	xmina=min([min(Dx1) min(Bx) min(Cx) min(bush(:,1)) ...      	min(xsl1) min(xblocka) min(coord(:,1))]);	xmaxa=max([max(Dx1) max(Bx) max(Cx) max(bush(:,1)) ...    		max(xsl1) max(xblocka) max(coord(:,1))]);	ymina=min([min(Dy1) min(By) min(Cy) min(bush(:,2)) ...      	min(ysl1) min(yblocka) min(coord(:,2))]);	ymaxa=max([max(Dy1) max(By) max(Cy) max(bush(:,2)) ...      	max(ysl1) max(yblocka) max(coord(:,2))]);	rangex=xmaxa-xmina;	rangey=ymaxa-ymina;	xmin=xmina-0.05*rangex;	xmax=xmaxa+0.05*rangex;	ymin=ymina-0.05*rangey;	ymax=ymaxa+0.05*rangey;	values=axisadjust(xmin, xmax, ymin, ymax, width, height);	xmin=values(1);	xmax=values(2);	ymin=values(3);	ymax=values(4);	axis([xmin xmax ymin ymax]);	a = findobj('Tag','slidercrank_dsgn');    UD=a.UserData;	hdls = UD.hand;	lim =[xmin xmax ymin ymax];    a.UserData=struct('hand',hdls,'wlim',lim);% Draw the coupler curve first.	set(bush1,'xdata',bush(:,1),'ydata',bush(:,2));	set(circ1,'xdata',cir1(:,1),'ydata',cir1(:,2));	set(couplerpoint1,'xdata', Dx2, 'ydata' ,Dy2);	set(couplerpoint2,'xdata', [], 'ydata' ,[]);	if times==2		set(couplerpoint2,'xdata', Dx3, 'ydata' ,Dy3);	end% Draw the frame line.	set(sliderline1, 'xdata', xsl1, 'ydata', ysl1);% Animate the linkage as long as ans = 'y'		i = 6;	set(joint1,'xdata', Bx(i), 'ydata',By(i));	set(joint2,'xdata', Cx(i), 'ydata',Cy(i));	set(crank,'xdata',[0 Bx(i)],'ydata', [0 By(i)]);	set(coupler,'xdata', [Bx(i) Cx(i)],'ydata', [By(i) Cy(i)]);	set(gridpt,'xdata',Dx1(i), 'ydata' ,Dy1(i));	set(block,'xdata', [xblocka(i,:)],'ydata', [yblocka(i,:)]);	set(couplerptr,'xdata', [Bx(i) Dx1(i) Cx(i)], ...		'ydata' ,[By(i) Dy1(i) Cy(i)]);   %flush the draw buffer	drawnow; 	Bx_fix = Bx(i);	By_fix = By(i);	Cx_fix = Cx(i);	Cy_fix = Cy(i);	b = findobj('Tag','slidercrank_axe');    UD=b.UserData;	hdls = UD.hand;	lim =[Bx_fix By_fix Cx_fix Cy_fix];	b.UserData=struct('hand',hdls,'wlim',lim);	set([gridpt couplerptr],...  'buttondownfcn','SliderCrankAnalysis(''Set Calls'');SliderCrankAnalysis(''Move'')');%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%The Callbacks When the User Moves or Releases%the Mouse Button.%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%case 'Set Calls'%get the handle of the objects used to control the mouse input	a=findobj('Tag','slidercrank_dsgn');	nBetaflag=0;	nCradiusflag=0;	set(a,'windowbuttonupfcn','a=findobj(''Tag'',''slidercrank_dsgn'');set(a,''windowbuttonmotion'','''');refresh');	set(a,'windowbuttonmotionfcn','SliderCrankAnalysis(''Move'')');%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%The Callbacks When the User Moves the Mouse Button.%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%case 'Move'%Find out where the mouse pointer is located.	a=findobj('Tag','slidercrank_dsgn');	b=findobj('Tag','slidercrank_axe');    UD=b.Username;	hdls = UD.wlim;   Bx = hdls(1);   By = hdls(2);    Cx = hdls(3);    Cy = hdls(4); 	set(h_status,'string','');	if nBetaflag == 1 || nCradiusflag == 1		a = get(h_beta,'userdata');   	b = get(h_beta,'string');   	beta = check_val(a,b);   	a = get(h_R(4),'userdata');   	b = get(h_R(4),'string');   	cr1 = check_val(a,b);	else   	pt=get(b,'currentpoint');   	pt=pt(1,1:2);   	beta = atan2(pt(2)-By,pt(1)-Bx)*180/pi-atan2(Cy-By,Cx-Bx)*180/pi;		cr1 = sqrt((pt(2) -By)^2 + (pt(1) -Bx)^2);	end	if beta > 180   	beta = beta - 360;	elseif beta < -180   	beta = beta+360;     	end   % Store the value in the current axes UserData% where it can be retrieved by an application.	set(h_beta,'userdata',beta);	set(h_beta,'string',num2str(beta));	set(h_R(4),'userdata',cr1);	set(h_R(4),'string',num2str(cr1));	i=0;	d2r=pi/180;	r2d=180/pi;	Q11=Q1*d2r;	cpad=beta*d2r;	qstart=0;	qstop=360;	if driver==1; [qstart, qstop]=sc_angle_limits_cr(r2,r3,r4,Q1); end	if driver==2; [qstart, qstop]=sc_angle_limits_co(r2,r3,r4,Q1); end	if driver==3; [qstart, qstop]=sc_angle_limits_sl(r2,r3,r4,Q1); end	npos=100;	dq=(qstop-qstart)/(npos-1);	Astar=[0;0];	rpivot=0.15*r2;	delta=0.3*r2;	QM1=qstart;		i2=0;% Loop to analyze the linkage for tt full cycles	for mmm=1:1:times% Analyze for two cycles in different modes if driver is 2 or 3		if mmm ==2			mode = -mode;			dq=-dq;		end		for motion=1:1:npos			if motion>1; QM1=QM1+dq; end			i=i+1;			QM = QM1*d2r;% Call function to compute the position			if driver ==1; para = sldcrkc(r2,r3,r4,QM1,w2,0,mode,Q1,3); end			if driver ==2; para = sldcrkco(r2,r3,r4,QM1,w3,0,mode,Q1,3); end				if driver ==3; para = sldcrks(QM1,r2,r3,r4,r1d,0,mode,Q1,3); end					Q2=para(6);			if Q2<0; Q2=Q2+360; end			Q22(i)=Q2;			w22(i)=para(14);			Q3=para(7);			Q33(i)=Q3;			w33(i)=para(15);			r1(i)=para(1);			rd1(i)=para(9);			assem=para(37);			cang1=(Q3+beta)*d2r;% Store position information in matrices.			Bx(i)= para(25);			By(i)= para(26);			Cx(i)= para(27);			Cy(i)= para(28);			Dx1(i)=Bx(i)+cr1*cos(cang1);			Dy1(i)=By(i)+cr1*sin(cang1);			% Determine coordinates for coupler curve for first mode						if mmm==1				Dx2(i)=Dx1(i);				Dy2(i)=Dy1(i);			end			% Determine coordinates for coupler curve for second mode						if mmm==2				i2=i2+1;				Dx3(i2)=Dx1(i);				Dy3(i2)=Dy1(i);			end		% Locate the slider block coordinates			xpin=Cx(i);			ypin=Cy(i);			height=delta;			length=2*height;         coords = rect(length,height,xpin,ypin,Q1,0);         % Prepare the result data and store them in the global variable g_AnaRes% Those result data can be save as output data while menu 'save result data'% is choosen.		Q222 = Q22';		Q333 = Q33';		r11 = r1';		w333 = w33';		rd11 = rd1';		g_AnaRes = [Q222 Q333 r11 w333 rd11];				end	end	itotal=i;% Determine the coordinates of the slider line					smax=max(r1);	smin=min(r1);	[cmin, index]=min(Cx);	cymin=Cy(index);	length1=smax-smin+length;	ndash=20;	csang=cos(Q11);	ssang=sin(Q11);		xe = cmin-(0.5*length)*csang;	ye = cymin-(0.5*length)*ssang;	slflag=1;	coord=frameline(length1,xe,ye,ndash,Q1,slflag);						npoints1=3*ndash;					% Offset slider line by half the height of the slider block.					% Draw the coupler curve first.	set(couplerpoint1,'xdata', Dx2, 'ydata' ,Dy2);	set(couplerpoint2,'xdata', [], 'ydata' ,[]);	if times==2		set(couplerpoint2,'xdata', Dx3, 'ydata' ,Dy3);	end		i = 6;	set(gridpt,'xdata',Dx1(i), 'ydata' ,Dy1(i));	set(couplerptr,'xdata', [Bx(i) Dx1(i) Cx(i)], ...		'ydata' ,[By(i) Dy1(i) Cy(i)]);   %flush the draw buffer	drawnow;    case 'Change R'	n=Argument2;	newval=get(h_R(n),'string');	r(n)=check_val(r(n),newval);	set(h_R(n),'UserData',r(n));	set(h_R(n),'string',num2str(r(n)));   SliderCrankAnalysis('Draw Axes'); case 'Change Cradius'	nCradiusflag=1;	SliderCrankAnalysis('Move');case 'Change Beta'	nBetaflag=1;	SliderCrankAnalysis('Move');      case 'Change Q'   newval=get(h_Q,'string');   Q1=check_val(Q1,newval);   set(h_Q,'UserData',Q1);  	set(h_Q,'string',num2str(Q1));   SliderCrankAnalysis('Draw Axes');       case 'D_radio'      num_buttons = 3;   button = Argument2;	if get(h_Dradio(button),'value') == 0		set(h_Dradio(button),'value',1);  	end  	set(h_Dradio([1:(button-1), (button+1):num_buttons]),'value',0);   SliderCrankAnalysis('Draw Axes');      case 'M_radio'      num_buttons = 2;   button = Argument2;	if get(h_Mradio(button),'value') == 0		set(h_Mradio(button),'value',1);  	end  	set(h_Mradio([1:(button-1), (button+1):num_buttons]),'value',0);	SliderCrankAnalysis('Draw Axes');      case 'AS_radio'      nBetaflag = 1;   num_buttons = 2;   button = Argument2;	if get(h_ASradio(button),'value') == 0		set(h_ASradio(button),'value',1);  	end  	set(h_ASradio([1:(button-1), (button+1):num_buttons]),'value',0);	SliderCrankAnalysis('Draw Axes');      case 'Change Velocity'      newval=get(h_vel,'string');   vel=check_val(vel,newval);   set(h_vel,'UserData',vel);  	set(h_vel,'string',num2str(vel));     case 'Resize'	n = Argument2;%To retrieve current axis limits	a=findobj('Tag','slidercrank_dsgn');    UD=a.UserData;%If 'Zoomout' button is pressed, downsize the figure by increasing x and y axis limits%If 'ZoomIn' button is pressed, enlarge the figure by decreasing x and y axis limits	if n==1		factor=1.05;	else		factor=0.95;    end    UD.wlim=factor*UD.wlim;    a.UserData=UD;	axis(UD.wlim);case 'None'         % the following command will bring the existing figure    % to foreground   a = findobj('Tag','slidercrank_dsgn');   figure(a);   case 'Close'         % if the target window exist then close it   if ~isempty(findobj('Tag','Slider Crank Analysis Window'))      slidercrank_ana('Stop');      a = findobj('Tag','Slider Crank Analysis Window');      	close(a);	end      a = findobj('Tag','slidercrank_dsgn');   close(a);   case 'Show Definitions'   Show_definition('initialize','SliderCrankAnalysis.jpg');   case 'Analysis'	slidercrank_ana('initialize',r,beta,Q1,vel,driver,times,mode);   case 'Help'   HelpStr={'Slider Crank Analysis';    '';    '    "SliderCrankAnalysis"  is  a  program  to  design  a  slider crank';    'mechanism and to analyze the result. The program contains two windows:';    'a design window and an analysis window.   The nomenclature used by the';    'program  is  that  given  in  the  textbook, Kinematics, Dynamics, and';    'Design of Mechanisms by Kenneth Waldron and Gary Kinzel.';    '';    '    In the design  window, the  variables  are the  four  link lengths';    '(crank, coupler,slider-line offset and coupler radius)  and two angles';    '(beta and slider angle) and the angular velocity of the driver.   More';    'options are given  to present  the  mechanisms in  different modes, to';    'allow the mechanism to be driven by the crank,  coupler or slider, and';    'to show the coupler curve for one or both modes of mechanism.   One of';    'the program features is that users can  drag the mouse  to  change the';    'coupler point in a continuous fashion.   This  changes  both  the beta';    'angle and coupler radius.   In addition, five more link lengths of the';    'cognates mechanism  are  shown  and  updated  dynamically.     Because';    'changing the  coupler point  can  move  the  mechanism plot out of the';    'figure window, "Zoom Out"  and  "Zoom In"  buttons  are  available  to';    'downsize and upsize the mechanism plot.' ;    '';    '    In the analysis window,  users can control the number of plots (up';    'to four) and the  contents  of  each plot (four options) are provided.';    'Furthermore, the animation speed can be  easily  adjusted  by clicking';    'on the speed buttons.';    '';    '    Two buttons connect the two  windows.   The "Return" button in the';    'analysis window closes  the analysis window and brings back the design';    'window, while the  "Analysis"  button in  the  design  window open the';    'analysis  window  and  start  the  linkage  animation  and  associated';    'shaking force animation.   Every time the "analysis"button is pressed,';    'all of the design parameters are updated to start a new animation. ';    '';    '    There is a status bar at the bottom of the design window.   If the';    'chosen values  for the variables  cannot be  used  to create a  slider';    'crank mechanism, an error message will be shown in the status box.';     ' '};   helpwin(HelpStr,'Slider Crank Analysis');case 'Get File'% Get the file name using the UIGETFILE   [sFileName sFilePath] = uigetfile('*.dat','Load File');     if sFileName == 0 && sFilePath == 0      return;   end      % Combine the path and file name together   sTemp = strcat( sFilePath, sFileName );% get the size to erase the '.dat' extension      sz  = size(sFileName);% load the data file     load(sTemp);% assign the parameter to a variable   Parameter = eval(sFileName(1:(sz(2)-4)));   if 10 == max(size(Parameter))      r(1:4) = Parameter(1:4);      beta = Parameter(5);      Q1 = Parameter(6);      driver = Parameter(7);      mode = Parameter(8);      times = Parameter(9);      vel = Parameter(10);    	for i=1:4    		set(h_R(i),'UserData',r(i));   	end   	set(h_Q,'UserData',Q1);      set(h_beta,'Userdata',beta);      set(h_vel,'Userdata',vel);      if driver == 1        	SliderCrankAnalysis('D_radio',1);      elseif driver == 2        	SliderCrankAnalysis('D_radio',2);      else       	SliderCrankAnalysis('D_radio',3);      end         if times == 1        	SliderCrankAnalysis('M_radio',1);      else        	SliderCrankAnalysis('M_radio',2);      end         if mode == 1        	SliderCrankAnalysis('AS_radio',1);      else        	SliderCrankAnalysis('AS_radio',2);      end         for i=1:4   	  	set(h_R(i),'string',num2str(r(i)));   	end   	set(h_Q,'string',num2str(Q1));   	set(h_beta,'string',num2str(beta));      set(h_vel,'string',num2str(vel));        	refresh;   else   	set(h_status,'string','invalid file format');   	return   endcase 'Put File'   [sFileName sFilePath] = uiputfile('*.dat','Save As');   if Argument2 == 1      	ParameterData=[r2;r3;r4;cr1;beta;Q1;driver;mode;times;vel];   else      ParameterData = g_AnaRes;   end%sTemp = strcat( sFilePath, sFileName);   [sTemp , errMsg] = sprintf('%s%s', sFilePath, sFileName);   [strCmd, errMsg] = sprintf('%s %s %s', 'save', sTemp, '-ascii ParameterData');   eval(strCmd);   endfunction val=check_val(oldval,newval)%Check if the user input is a number input%if not, retrieve the previous value in userdataval=zeros(1);if~isequal(length(oldval),length(str2num(newval))),  	val=oldval;%	set(findobj(gcf,'Tag','status_txt'),'String',...%['Warning:Aninvalidpropertyvaluehasbeenentered.']);else 	val=str2num(newval);end   function LocalOpenFig()hdls = zeros(1,20);%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Control the color of the UI%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%StdColor = get(0,'DefaultUIcontrolBackgroundColor');%PointsPerPixel = 72/get(0,'ScreenPixelsPerInch');bgframe = StdColor;bgedit = [1 1 1];bgtext = bgframe;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% The Main Figure of Crank Rocker Design%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%ScreenSize = get(0,'ScreenSize');sx = 480;sy = 400;FigPos = [ 10 ScreenSize(4)-sy-40 sx sy ] ;a = figure('Units','pixels', ...	'Color',[0.8 0.8 0.8], ...	'Name','Slider Crank Analysis', ...	'NumberTitle','off', ...	'Position',FigPos, ...  	'Tag','slidercrank_dsgn');%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Draw Frame First% This will prevent the frames from been drawn on top of% other UICONTROLS. %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgframe, ...	'Position',[0.0083 0.01 0.9833 0.11], ...	'Style','frame', ...  	'Tag','Frame1');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgframe, ...	'Position',[0.6 0.7175 0.3896 0.2575], ...	'Style','frame', ...	'Tag','Frame_link');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgframe, ...	'Position',[0.01 0.1275 0.577 0.0850], ...	'Style','frame', ...	'Tag','Frame_angle');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgframe, ...	'Position',[0.6 0.1275 0.3896 0.0850], ...	'Style','frame', ...	'Tag','Frame_pushbutton');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgframe, ...	'Position',[0.6 0.645 0.3896 0.065], ...	'Style','frame', ...	'Tag','Frame_R');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgframe, ...	'Position',[0.6 0.312 0.3896 0.055], ...	'Style','frame', ...  	'Tag','Frame6');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgframe, ...	'Position',[0.6 0.474 0.3896 0.1645], ...	'Style','frame', ...	'Tag','Frame7');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgframe, ...	'Position',[0.6 0.375 0.3896 0.0925], ...	'Style','frame', ...	'Tag','Frame_assemode');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgframe, ...	'Position',[0.6 0.2188 0.3896 0.085], ...	'Style','frame', ...	'Tag','Frame_zoombutton');      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% menu uicontrols%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%b = uimenu('Parent',a, ...	'Label','Parameter', ...	'Tag','menu_parameter');  c = uimenu('Parent',b, ...	'Callback','SliderCrankAnalysis(''Get File'')', ...	'Label','Load Parameters', ...	'Tag','submenu_load');   c = uimenu('Parent',b, ...	'Callback','SliderCrankAnalysis(''Put File'',1)', ...	'Label','Save Parameters', ...	'Tag','submenu_save1');c = uimenu('Parent',b, ...	'Callback','SliderCrankAnalysis(''Put File'',2)', ...	'Label','Save Result Data', ...	'Tag','submenu_save2');%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Create graphics window for design input%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%-------Reference axisb = axes('Parent',a, ...  	'Box','on', ...  	'Color',[1 1 1], ...	'Position',[0.0083 0.2275 0.5771 0.7475], ...  	'XColor',[0 0 0], ...  	'XGrid','off', ...  	'XTick',[],...  	'YColor',[0 0 0], ...  	'YGrid','off', ...  	'YTick',[], ...  	'ZColor',[0 0 0], ...  	'ZGrid','off', ...  	'ZTick',[]);  b = axes('Parent',a, ...   'Box','off', ...  	'DataAspectRatio',[1 1 1],...	'CameraUpVector',[0 1 0], ...	'CameraUpVectorMode','manual', ...	'Position',[0.0083 0.2275 0.5771 0.7475], ...	'Tag','slidercrank_axe');axis off;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% edit related uicontrols%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%h_beta = uicontrol('Parent',a, ...  	'Units','normalized', ...  	'BackgroundColor',bgedit, ...	'Callback','SliderCrankAnalysis(''Change Beta'')', ...	'Position',[0.1625 0.1425 0.1125 0.0525], ...	'String','20', ...  	'Horiz','left', ...	'Style','edit', ...	'Tag','beta_edit', ...  	'UserData',20);   h_Q  = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgedit, ...	'Callback','SliderCrankAnalysis(''Change Q'')', ...	'Position',[0.4625 0.1425 0.1125 0.0525], ...	'String','30', ...  	'Horiz','left', ...	'Style','edit', ...	'Tag','Q_edit', ...  	'UserData',30);   h_R(1) = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgedit, ...	'Callback','SliderCrankAnalysis(''Change R'',1)', ...	'Position',[0.823 0.9 0.1375 0.05], ...	'String','3', ...	'Style','edit', ...	'Tag','crank length', ...	'UserData',3);   h_R(2) = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgedit, ...	'Callback','SliderCrankAnalysis(''Change R'',2)', ...	'Position',[0.823 0.845 0.1375 0.05], ...	'String','8', ...	'Style','edit', ...	'Tag','coupler length', ...	'UserData',8);   h_R(3) = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgedit, ...	'Callback','SliderCrankAnalysis(''Change R'',3)', ...	'Position',[0.823 0.7925 0.1375 0.05], ...	'String','1', ...	'Style','edit', ...	'Tag','slider offset', ...	'UserData',1);   h_R(4) = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgedit, ...	'Callback','SliderCrankAnalysis(''Change Cradius'')', ...	'Position',[0.823 0.7375 0.1375 0.05], ...	'String','6', ...	'Style','edit', ...	'Tag','coupler radius)', ...	'UserData',6);      h_vel = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgedit, ...	'Callback','SliderCrankAnalysis(''Change velocity'')', ...	'Position',[0.823 0.6525 0.1375 0.05], ...	'String','5', ...	'Style','edit', ...	'Tag','vel_edit', ...	'UserData',5);      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% radio button controls%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         h_Dradio(1) = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Callback','SliderCrankAnalysis(''D_radio'',1)', ...	'Position',[0.8 0.575  0.1292 0.0475], ...	'String','crank', ...	'Style','radiobutton', ...	'Tag','driver_radio1', ...	'Value',0);           h_Dradio(2) = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Callback','SliderCrankAnalysis(''D_radio'',2)', ...	'Position',[0.8 0.5325  0.14 0.0475], ...	'String','coupler', ...	'Style','radiobutton', ...	'Tag','driver_radio2', ...	'Value',1);   h_Dradio(3) = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Callback','SliderCrankAnalysis(''D_radio'',3)', ...	'Position',[0.8 0.4875  0.1292 0.0475], ...	'String','slider', ...	'Style','radiobutton', ...	'Tag','driver_radio3', ...	'Value',0);   h_Mradio(1) = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Callback','SliderCrankAnalysis(''M_radio'',1)', ...	'Position',[0.6125 0.315 0.177 0.0425], ...	'String','one mode', ...	'Style','radiobutton', ...	'Tag','mode_radio1', ...	'Value',0);   h_Mradio(2) = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Callback','SliderCrankAnalysis(''M_radio'',2)', ...	'Position',[0.8 0.315 0.177 0.0425], ...	'String','both modes', ...	'Style','radiobutton', ...	'Tag','mode_radio2', ...   'Value',1);   h_ASradio(1) = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Callback','SliderCrankAnalysis(''AS_radio'',1)', ...	'Position',[0.8 0.4125 0.177 0.0425], ...	'String','', ...	'Style','radiobutton', ...	'Tag','assmode_radio1', ...	'Value',1);   h_ASradio(2) = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Callback','SliderCrankAnalysis(''AS_radio'',2)', ...	'Position',[0.8 0.3775 0.177 0.0425], ...	'String','-1', ...	'Style','radiobutton', ...	'Tag','assmode_radio2', ...	'Value',0);   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% push button controls%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Callback','SliderCrankAnalysis(''Analysis'')', ...	'Position',[0.6042 0.14 0.1333 0.06], ...	'String','Analysis', ...	'Tag','AnalysisButton');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Callback','SliderCrankAnalysis(''Close'')', ...	'Position',[0.8542 0.14 0.1324 0.06], ...	'String','Close', ...	'Tag','CloseButon');   b = uicontrol('Parent',a, ...  	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Callback','SliderCrankAnalysis(''Help'')', ...	'Position',[0.7367 0.14 0.1166 0.06], ...	'String','Info', ...	'Tag','InfoButton');   b = uicontrol('Parent',a, ...  	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Callback','SliderCrankAnalysis(''Resize'',1)', ...	'Position',[0.6042 0.2325 0.1333 0.06], ...	'String','Zoom Out', ...	'Tag','ZoomButton1', ...	'UserData',1);   b = uicontrol('Parent',a, ...  	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Callback','SliderCrankAnalysis(''Resize'',2)', ...	'Position',[0.7367 0.2325 0.1166 0.06], ...	'String','Zoom In', ...   'Tag','ZoomButton2');b = uicontrol('Parent',a, ...  	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Callback','SliderCrankAnalysis(''Show Definitions'')', ...	'Position',[0.8542 0.2325 0.1324 0.06], ...   'String','Definitions', ...	'Tag','ZoomButton2');       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% text controls%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%h_status = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'HorizontalAlignment','left', ...	'Position',[0.05625 0.0275 0.9083 0.0475], ...	'String',' ', ...	'Style','text', ...	'Tag','status_txt');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.02708 0.145 0.1375 0.0425], ...	'String','Beta angle', ...	'Style','text', ...	'Tag','StaticText1');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.2938 0.145 0.1521 0.0425], ...	'String','slider angle', ...	'Style','text', ...	'Tag','StaticText1');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.6167 0.8975 0.1708 0.0425], ...	'String','crank length', ...	'Style','text', ...	'Tag','StaticText2');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.6167 0.8475 0.1833 0.0425], ...	'String','coupler length', ...	'Style','text', ...	'Tag','StaticText2');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.6167 0.787 0.1955 0.0425], ...	'String','slider-line offset', ...	'Style','text', ...	'Tag','StaticText3');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.6167 0.74 0.18125 0.0425], ...	'String','coupler radius', ...	'Style','text', ...	'Tag','StaticText4');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.6167 0.6575 0.20625 0.0425], ...	'String','angular velocity', ...	'Style','text', ...	'Tag','StaticText4');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.6167 0.415 0.1375 0.0425], ...	'String','assembly mode', ...	'Style','text', ...	'Tag','StaticText4');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.8354 0.4125 0.033 0.0425], ...	'String','1', ...	'Style','text', ...	'Tag','StaticText4');   b = uicontrol('Parent',a, ...	'Units','normalized', ...	'BackgroundColor',bgtext, ...	'Position',[0.6167 0.5725 0.0854 0.0425], ...	'String','driver', ...	'Style','text', ...	'Tag','StaticText4');hdls=[ h_beta h_Q h_R h_vel h_Dradio h_Mradio h_ASradio h_status ];a = findobj('Tag','slidercrank_dsgn');set(a,'userdata',struct('hand',hdls));	joint1=line('xdata', [], 'ydata', [], 'marker', 'o','markersize',8,'color', 'k');joint2=line('xdata', [], 'ydata', [], 'marker', 'o','markersize', ...       8,'color', 'k');crank=line('xdata', [], 'ydata' ,[], 'linewidth' ,1,'color', 'k');coupler=line('xdata', [], 'ydata' ,[], 'linewidth' ,1,'color', 'k');couplerptr=line('xdata', [], 'ydata' ,[], 'linewidth' ,1,'color', 'k');	   couplerpoint1=line('xdata', [], 'ydata' ,[], 'linewidth' ,1,'color', 'r', 'linestyle', '-');	couplerpoint2=line('xdata', [], 'ydata' ,[], 'linewidth' ,1,'color', 'r', 'linestyle', '-');	gridpt=line('xdata', [], 'ydata' ,[], 'linewidth' ,1,'markersize',6,'color', 'g', 'marker', 'o');	block=line('xdata', [], 'ydata', [], 'linewidth' ,1,'color', 'k');bush1=line('xdata', [], 'ydata',[], 'linestyle', '-', 'markersize', 8,'color', 'r');circ1=line('xdata', [], 'ydata',[], 'linestyle', '-', 'markersize', 8,'color', 'r');sliderline1=line('xdata', [], 'ydata',[], 'linestyle', '-','color', 'r');hdls=[ joint1 joint2 crank coupler couplerptr ...       couplerpoint1 couplerpoint2 gridpt ...		 block sliderline1 bush1 circ1 ];a = findobj('Tag','slidercrank_axe');set(a,'userdata',struct('hand',hdls));