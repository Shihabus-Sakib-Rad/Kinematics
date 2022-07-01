function rbg_slidercrank_ana(Action,Astar,sliderpt,ax,ay,bx,by,theta)
% the contents of the userdata of Analysis Window 
% handles(1) = handle of fig number
% handles(2) = handle of fig1 popupmenu
% handles(3) = handle of fig2 popupmenu
% handles(4) = handle of fig3 popupmenu
% handles(5) = handle of fig4 popupmenu
% handles(6) = handle of axe1
% handles(7) = handle of axe2
% handles(8) = handle of axe3
% handles(9) = handle of axe4
% handles(10) = flag for deciding to start or stop animation
% handles(11) = handle of bead(1)
% handles(12) = handle of bead(2)
% handles(13) = handle of bead(3)
% handles(14) = handle of bead(4)
% handles(15) = Previous Start Index
% handles(16) = timesink
% handles(17) = handle of reference axe for axe1
% handles(18) = handle of reference axe for axe2
% handles(19) = handle of reference axe for axe3
% handles(20) = handle of reference axe for axe4
% handles(21) = handle of submenu 'Load'
% handles(22) = handle of submenu 'Save'
% handles(23) = handle of return button;
% handles(24) = handle of speed plus button;
% each mechanism axes has the its own userdata storing its 
% unique graphic handle [joint1 joint2 crank coupler rocker];      

global strFigType 
global nAnimationFlag
global g_AnaRes
global nChangeSpeed
% Define the axes for the first graph (input/output angle graph)
if nargin==0 && isempty(findobj('Tag','RBG SliderCrank Analysis Window')),  % rbg_slidercrank_ana is being initialized
   Action='initialize';
elseif nargin== 0 && ~isempty(findobj('Tag','RBG SliderCrank Analysis Window'))      % rbg_slidercrank_ana is already open
   Action='None';
elseif nargin== 1 && ~isempty(findobj('Tag','RBG SliderCrank Analysis Window')) && strcmp( Action, 'initialize'), % rbg_slidercrank_ana is already open
	% Recalculate the value, and then redraw
	Action='None';
end
switch Action
case 'initialize'
   if  isempty(findobj('Tag','RBG SliderCrank Analysis Window'))   
      LocalOpenFig;
      nAnimationFlag = 0;
   else
  		rbg_slidercrank_ana('None');
   end
   
  if nargin < 6 
      Astar = [ 3.00 1.09 ];
      sliderpt = [ 1.78 1.82 ];
      ax = [ 0 3 2 ];
      ay = [ 0 0 2 ];
      theta = [ 45 135 0 ];
   end
   
   if exist('nAnimationFlag') 
      if nAnimationFlag == 1
      	rbg_slidercrank_ana('Stop');
         return;
      end
   end
   
   a = findobj('Tag','RBG SliderCrank Analysis Window');   
   handles = get(a,'userdata');
   set(a,'userdata',handles);
   g_AnaRes = CalcResult( Astar,sliderpt,ax,ay,bx,by,theta );
   DrawAnalysisWindow;
   rbg_slidercrank_ana('ShowFigure');
   rbg_slidercrank_ana('Start');
   
 	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%	Change Figure Call Back Function
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
case 'ChangeFigure'
   DrawAnalysisWindow;
   rbg_slidercrank_ana('ShowFigure');
   rbg_slidercrank_ana('Start');
   
case 'SetSpeed'
   nChangeSpeed = Astar(1);
   
case 'ChangeFigureNo'
   DrawAnalysisWindow;
   rbg_slidercrank_ana('ShowFigure');
   rbg_slidercrank_ana('Start');
   
case 'Animation'
   a = findobj('Tag','RBG SliderCrank Analysis Window');   
   handles = get(a,'userdata');
   
   [ m, n ] = size( g_AnaRes);
   itotal = n;
   timesink = handles(16);
   i = handles(15);
   
   nFigNum  = get( handles(1),'value');
   nType(1) = get( handles(2),'value');
   nType(2) = get( handles(3),'value');
   
   links = zeros(2,7);
   
   h_plus = handles(24);

   u = 0;
   u = u+1; Bx = g_AnaRes(u,:);
   u = u+1; By = g_AnaRes(u,:);
   u = u+1; Cx = g_AnaRes(u,:);
   u = u+1; Cy = g_AnaRes(u,:);
   u = u+1; Dx = g_AnaRes(u,:);
   u = u+1; Dy = g_AnaRes(u,:);
   u = u+1; Ex = g_AnaRes(u,:);
   u = u+1; Ey = g_AnaRes(u,:);
   u = u+1; B2x = g_AnaRes(u,:);
   u = u+1; B2y = g_AnaRes(u,:);
   u = u+1; C2x = g_AnaRes(u,:);
   u = u+1; C2y = g_AnaRes(u,:);
   u = u+1; D2x = g_AnaRes(u,:);
   u = u+1; D2y = g_AnaRes(u,:);
   u = u+1; E2x = g_AnaRes(u,:);
   u = u+1; E2y = g_AnaRes(u,:);
   
   u = 20; 
   u = u+1; xblock= g_AnaRes(u:u+4,:);
   u = u+5; yblock= g_AnaRes(u:u+4,:);
   xblocka = xblock';
   yblocka = yblock';
   u = u+5; xblock= g_AnaRes(u:u+4,:);
   u = u+5; yblock= g_AnaRes(u:u+4,:);
   xblocka2 = xblock';
   yblocka2 = yblock';
   
   Astar = g_AnaRes(45,9:10);
   
   for j = 1:nFigNum
     	links(j,:) = get(handles(j+5),'userdata');
   end   
   
% set timer
   dt = 5;
   tic;
   
   while nAnimationFlag      
          
      cycle=fix((i-0.0000001)/(360/dt)); %determine the cycle as an integer
      if i ==1
         drawnow;
      end
      
      if nFigNum > 0
         if nType(1) == 1
   			SetMechanismPosition( links(1,:),i,Bx,By,Cx,Cy,Dx,Dy,Ex,Ey,Astar,xblocka,yblocka);
         elseif nType(1) == 2   
   			SetMechanismPosition( links(1,:),i,B2x,B2y,C2x,C2y,D2x,D2y,E2x,E2y,Astar,xblocka2,yblocka2);
   		end
      end 
        
   	if nFigNum > 1
         if nType(2) == 1
   			SetMechanismPosition( links(2,:),i,Bx,By,Cx,Cy,Dx,Dy,Ex,Ey,Astar,xblocka,yblocka);
         elseif nType(2) == 2   
   			SetMechanismPosition( links(2,:),i,B2x,B2y,C2x,C2y,D2x,D2y,E2x,E2y,Astar,xblocka2,yblocka2);
   		end
      end
      
        
      drawnow;   %flush the draw buffer
      i= i+1;
      if i == itotal
         i = 1;
      end
      if nChangeSpeed ~= 0
         timesink = timesink * nChangeSpeed;
         nChangeSpeed = 0;
      end
      while toc < timesink
      end   
      tic;
   end
   nAnimationFlag = 0;
   handles(15) = i;
   handles(16) = timesink;
   set(a,'Userdata',handles);
   
case 'ShowFigure'
   a = findobj('Tag','RBG SliderCrank Analysis Window');
   handles = get( a,'userdata');
   nFigNum = get(handles(1),'value');
%[ m, n ] = size( g_AnaRes);
   npoints1 = g_AnaRes(45,23);
   u = 16;
   u = u+1; xsl1 = g_AnaRes(u,1:npoints1);
   u = u+1; ysl1 = g_AnaRes(u,1:npoints1);
   u = u+1; xsl12 = g_AnaRes(u,1:npoints1);
   u = u+1; ysl12 = g_AnaRes(u,1:npoints1);
   
   u = 40;
   u = u+1; xcoord = g_AnaRes(u,1:21);
   u = u+1; ycoord = g_AnaRes(u,1:21);
   u = u+1; xbush = g_AnaRes(u,1:26);
   u = u+1; ybush = g_AnaRes(u,1:26);

   u = 45;
   axislimit = g_AnaRes(u,1:4);
   axislimit2 = g_AnaRes(u,5:8);
   Astar = g_AnaRes(u,9:10);
   ax = g_AnaRes(u,11:13);
   ay = g_AnaRes(u,14:16);
   bx = g_AnaRes(u,17:19);
   by = g_AnaRes(u,20:22);


acx = 1;
acy = 1;
if nFigNum > 0
      nType = get(handles(2),'value');
      axes(handles(6));
      if nType == 1
         CreateMechanismHandles( handles(6),ax,ay,bx,by,xcoord,ycoord,xbush,ybush,axislimit,xsl1,ysl1,acx,acy,Astar);
      elseif nType == 2   
         CreateMechanismHandles( handles(6),ax,ay,bx,by,xcoord,ycoord,xbush,ybush,axislimit2,xsl12,ysl12,acx,acy,Astar);
      end
   end      
   if nFigNum > 1
      nType = get(handles(3),'value');
      axes(handles(7));
      if nType == 1
         CreateMechanismHandles( handles(7),ax,ay,bx,by,xcoord,ycoord,xbush,ybush,axislimit,xsl1,ysl1,acx,acy,Astar);
      elseif nType == 2   
         CreateMechanismHandles( handles(7),ax,ay,bx,by,xcoord,ycoord,xbush,ybush,axislimit2,xsl12,ysl12,acx,acy,Astar);
      end
   end      
   
   refresh;
   
   
case 'None'
   figure( findobj('Tag','RBG SliderCrank Analysis Window'));
% Bring the figure to front, if it already exists

case 'Close'
   rbg_slidercrank_ana('Stop');
   a = findobj('Tag','RBG SliderCrank Analysis Window');   
   close(a);

case 'Start'
   if exist('nAnimationFlag')
      if nAnimationFlag == 1
         return;
      end
   end
   
   a = findobj('Tag','RBG SliderCrank Analysis Window');   
   handles = get(a,'userdata');
   set(handles(1),'Enable','off');
   set(handles(2),'Enable','off');
   set(handles(3),'Enable','off');
   set(handles(21),'Enable','off');
   set(handles(22),'Enable','off');
   set(handles(23),'Enable','off');
   
   nAnimationFlag = 1;
   nChangeSpeed = 0;
   handles(10) = nAnimationFlag;
   set(a,'userdata',handles);
   rbg_slidercrank_ana('Animation');
   
case 'Stop'
   if  isempty(findobj('Tag','RBG SliderCrank Analysis Window'))
     	return;   
   end
   a = findobj('Tag','RBG SliderCrank Analysis Window');   
   handles = get(a,'userdata');
   nAnimationFlag = 0;
   handles(10) = nAnimationFlag;
   set(a,'userdata',handles);
   set(handles(1),'Enable','on');
   set(handles(21),'Enable','on');
   set(handles(22),'Enable','on');
   set(handles(23),'Enable','on');
   nFigNum = get(handles(1),'value');
   for i = 1:nFigNum,
     	set(handles(i+1),'Enable','on');
   end   
   
case 'Return'
   rbg_slidercrank_ana('Stop');
   a = findobj('Tag','RBG SliderCrank Analysis Window');   
   handles = get(a,'userdata');
   if ~isempty(findobj('Tag','rbg_slidercrank_dsgn'))   
     	b = findobj('Tag','rbg_slidercrank_dsgn');
      figure(b);
     	b = findobj('Tag','RBG SliderCrank Analysis Window');   
     	close(b);
   else
      return;      
   end        

case 'Get File'
% Get the file name using the UIGETFILE
   [sFileName sFilePath] = uigetfile('*.dat','Load File'); 

   if sFileName == 0 && sFilePath == 0
   	return;
   end

% Combine the path and file name together
   sTemp = strcat( sFilePath, sFileName );
% get the size to erase the '.dat' extension 
   sz = size(sFileName);
% load the data file
   load(sTemp);
% assign the parameter to a variable
   Parameter = eval(sFileName(1:(sz(2)-4))); 

   a = findobj('Tag','Analysis Window');
   handles = get(a,'userdata');

   if length(Parameter) == 3
   	nFigNum = Parameter(1);
   	nType1 = Parameter(2);
   	nType2 = Parameter(3);

   	set(handles(1),'Value',nFigNum);
      set(handles(2),'Value',nType1);
   	set(handles(3),'Value',nType2);

   	set(a,'userdata',handles);
   	DrawAnalysisWindow;
   	rbg_slidercrank_ana('ShowFigure');
   	rbg_slidercrank_ana('Start');
   else
   	return;
   end

case 'Put File'
   [sFileName sFilePath] = uiputfile('*.dat','Save As'); 

   a = findobj('Tag','Analysis Window');
   handles = get(a,'userdata');

   nFigNum = get(handles(1),'Value');
   nType1 = get(handles(2),'Value');
   nType2 = get(handles(3),'Value');
   ParameterData = [nFigNum; nType1; nType2]; 

% sTemp = strcat( sFilePath, sFileName)
   [sTemp , errMsg] = sprintf('%s%s', sFilePath, sFileName);
   [strCmd, errMsg] = sprintf('%s %s %s', 'save', sTemp, ' -ascii ParameterData');
   eval(strCmd);

end % switch

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	Show UI
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function DrawAnalysisWindow();
a = findobj('Tag','RBG SliderCrank Analysis Window');
handles = get(a,'userdata');
h_fignum = handles(1);
h_fig1 = handles(2);
h_fig2 = handles(3);
nFigNum=get(h_fignum,'Value');
position = zeros( 4, 4, 4);

%%%%%%%%%% Change the size of windows %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
position(1,1,:) = [0.058 0.145 0.9 0.84];       % size of screen 2
position(2,1,:) = [0.22 0.57 0.58 0.42];        % size of screen 1-1
position(2,2,:) = [0.22 0.14 0.58 0.42];        % size of screen 1-2

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Clearup previous axes first
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 6:7,
	if handles(i) ~= 0
		delete(handles(i));
		handles(i) = 0;
	end
end
for i = 17:18,
	if handles(i) ~= 0
		delete(handles(i));
		handles(i) = 0;
	end
end


for i = 1:nFigNum,
	set(handles(i+1),'Enable','on');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Axis 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

if nFigNum == 1
	b = axes('Parent',a, ...
	'Box','on', ...
	'Color',[1 1 1], ...
	'Position',position(1,1,:), ...
	'XColor',[0 0 0], ...
	'XGrid','off', ...
	'XTick',[],...
	'YColor',[0 0 0], ...
	'YGrid','off', ...
	'YTick',[], ...
	'ZColor',[0 0 0], ...
	'ZGrid','off', ...
	'ZTick',[]);
handles(17) = b;

h_1 = axes('Parent',a, ...
	'CameraUpVector',[0 1 0], ...
	'CameraUpVectorMode','manual', ...
	'Color',[1 1 1], ...
	'Position',position(1,1,:), ...
	'Tag','Axes1', ...
	'XColor',[0 0 0], ...
	'YColor',[0 0 0], ...
	'ZColor',[0 0 0]);
handles(6) = h_1;

end	
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Axis 2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if nFigNum == 2

	b = axes('Parent',a, ...
	'Box','on', ...
	'Color',[1 1 1], ...
	'Position',position(2,1,:), ...
	'XColor',[0 0 0], ...
	'XGrid','off', ...
	'XTick',[],...
	'YColor',[0 0 0], ...
	'YGrid','off', ...
	'YTick',[], ...
	'ZColor',[0 0 0], ...
	'ZGrid','off', ...
	'ZTick',[]);
handles(17) = b;

h_1 = axes('Parent',a, ...
	'CameraUpVector',[0 1 0], ...
	'CameraUpVectorMode','manual', ...
	'Color',[1 1 1], ...
	'Position',position(2,1,:), ...
	'Tag','Axes1', ...
	'XColor',[0 0 0], ...
	'YColor',[0 0 0], ...
   'ZColor',[0 0 0]);

b = axes('Parent',a, ...
	'Box','on', ...
	'Color',[1 1 1], ...
	'Position',position(2,2,:), ...
	'XColor',[0 0 0], ...
	'XGrid','off', ...
	'XTick',[],...
	'YColor',[0 0 0], ...
	'YGrid','off', ...
	'YTick',[], ...
	'ZColor',[0 0 0], ...
	'ZGrid','off', ...
	'ZTick',[]);
handles(18) = b;
h_2 = axes('Parent',a, ...
	'CameraUpVector',[0 1 0], ...
	'CameraUpVectorMode','manual', ...
	'Color',[1 1 1], ...
	'Position',position(2,2,:), ...
	'Tag','Axes2', ...
	'XColor',[0 0 0], ...
	'YColor',[0 0 0], ...
	'ZColor',[0 0 0]);
handles(6) = h_1;
handles(7) = h_2;

end	

	set( a, 'Userdata', handles);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	Show UI
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function LocalOpenFig();
strFigType = [ ...
      'mode -1';...
      'mode  1'];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	Main Figure
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ScreenSize = get(0,'ScreenSize');
sx = 520;
sy = 440;
FigPos = [ ScreenSize(3)-sx-10 ScreenSize(4)-sy-95 sx sy ] ;
a = figure('Units','Pixels', ...
	'Color',[0.8 0.8 0.8], ...
   'Name','RBG SliderCrank Analysis Window', ... 
   'NumberTitle','off', ...
	'Position',FigPos, ...
 	'Tag','RBG SliderCrank Analysis Window');
	
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Draw the frame for Figure Selection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
   'BackgroundColor',[0.752941 0.752941 0.752941], ...
   'Position',[0.01731 0.020 0.77 0.1117], ...
   'Style','frame', ...
   'Tag','Frame2');
	
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
   'Position',[0.7808 0.020 0.2038 0.1117], ...
   'Style','frame', ...
	'Tag','Frame1');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Menu uicontrols
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
b = uimenu('Parent',a, ...
	'Label','Parameter', ...
	'Tag','menu_parameter');

h_load = uimenu('Parent',b, ...
	'Callback','rbg_slidercrank_ana(''Get File'')', ...
	'Label','Load', ...
	'Tag','submenu_load');

h_save = uimenu('Parent',b, ...
	'Callback','rbg_slidercrank_ana(''Put File'')', ...
	'Label','Save', ...
	'Tag','submenu_save');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Setup number of figures
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'Position',[0.03 0.07 0.095 0.03911], ... 
	'String','No. of figs ', ...
	'Style','text', ...
 	'Tag','StaticText2');

h_fignum = uicontrol('Parent',a, ...
	'Units','normalized', ...
   'Callback','rbg_slidercrank_ana(''ChangeFigureNo'');', ...
   'Position',[0.135 0.0837 0.10 0.03538], ...
   'String',['1';'2'], ...
	'Style','popupmenu', ...
	'Tag','FigNumber', ...
 	'Value',2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Speed Control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
   'Position',[0.03 0.02793 0.08 0.03910], ...
   'String','Speed ', ...
	'Style','text', ...
 	'Tag','StaticText2');
	
h_plus = uicontrol('Parent',a, ...
	'Units','normalized', ...
   'Callback','rbg_slidercrank_ana(''SetSpeed'',0.707);', ...
   'Position',[0.1327 0.02955 0.04038 0.03182], ... 
	'String','+', ...
	'Enable','on', ...
	'Tag','Plus_button');

b = uicontrol('Parent',a, ...
	'Units','normalized', ...
   'Callback','rbg_slidercrank_ana(''SetSpeed'',1.414);', ...
   'Position',[0.2 0.02955 0.04038 0.03182], ...
   'String','-', ...
	'Tag','Minus_button');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Put label in front of the popup menu item 	
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
   'Position',[0.2442 0.07821 0.06262 0.03724], ...
   'String','Fig 1', ...
	'Style','text', ...
 	'Tag','StaticText1');
 
b = uicontrol('Parent',a, ...
	'Units','normalized', ...
   'Position',[0.5019 0.08007 0.06262 0.03724], ...
   'String','Fig 2', ...
   'Style','text', ...
	'Tag','StaticText1');
	
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Setup the Popup menu contents for Figure Type Selection 	
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
h_fig1 = uicontrol('Parent',a, ...
	'Units','normalized', ...
   'Callback','rbg_slidercrank_ana(''ChangeFigure'',1);', ...
   'Position',[0.3077 0.0837 0.19256 0.03538], ...
   'String',strFigType, ...
 	'Style','popupmenu', ...
	'Tag','Fig1', ...
	'Value',1);

h_fig2 = uicontrol('Parent',a, ...
	'Units','normalized', ...
   'Callback','rbg_slidercrank_ana(''ChangeFigure'',2);', ...
   'Position',[0.5692 0.0837 0.19256 0.03538], ...
   'String',strFigType, ...
	'Style','popupmenu', ...
	'Tag','Fig2', ...
	'Value',2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Start/Stop/Return
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
h_start = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'CallBack','rbg_slidercrank_ana(''Start'')', ...
   'Position',[0.7942 0.07727 0.08577 0.04318], ...
  	'String','Start', ...
	'Tag','start_button');

h_stop = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'CallBack','rbg_slidercrank_ana(''Stop'')', ...
	'Position',[0.8854 0.07727 0.08577 0.04318], ... 
	'String','Stop', ...
	'Userdata',1,...
	'Tag','stop_button');

h_return = uicontrol('Parent',a, ...
	'Units','normalized', ...
	'CallBack','rbg_slidercrank_ana(''Return'')', ...
	'Position',[0.7942 0.02891 0.1773 0.04318], ... 
	'String','Return', ...
	'Tag','return_button');

drawnow;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Assign Initial value
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nAnimationFlag = 0;
handles = zeros( 1, 24);
handles(1) = h_fignum;
handles(2) = h_fig1;
handles(3) = h_fig2;
handles(10) = nAnimationFlag;
handles(15) = 1;
% initial animation starting index
handles(16) = 0.001;  
% initial timesink value
handles(21) = h_load;
handles(22) = h_save;
handles(23) = h_return;
handles(24) = h_plus;
set( a, 'Userdata', handles);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	Calculate the result
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [res] = CalcResult(Astar,sliderpt,ax,ay,bx,by,theta );
fact=pi/180;
theta1 = theta(1);
theta2 = theta(2);
theta3 = theta(3);
theta1r=theta(1)*fact;
theta2r=theta(2)*fact;
theta3r=theta(3)*fact;
Astarx = Astar(1);
Astary = Astar(2);
fx1 = sliderpt(1);
fy1 = sliderpt(2);
ax1 = ax(1);
ax2 = ax(2);
ax3 = ax(3);
ay1 = ay(1);
ay2 = ay(2);
ay3 = ay(3);
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

% Read in the center point input
Values = circlepoint (ax1,ay1,theta1,ax2,ay2,theta2,ax3,ay3,...
                   theta3, Astarx,Astary);
acx(1)=Values(1);
acy(1)=Values(2);	

scale=sqrt((max(ax)-min(ax))^2+(max(ay)-min(ay))^2);
rpivot=0.02*scale;
delta=2*rpivot;	

% Draw circle point and center point for first point.  First find 
% coordinates of first bushing and pin joint.					
				
ninc=20;  
npoints=ninc+1;
centerptx=Astar(1);
centerpty=Astar(2);					
coord=circle2(rpivot,centerptx,centerpty,ninc);

% find coordinates of pin
for i=1:1:npoints
	xcoord1(i)=coord(i,1);
	ycoord1(i)=coord(i,2);
end

% find coordinates of bushing
binc=ninc/2;
coord1=bushing(rpivot,centerptx,centerpty,binc,0);
bpoints=ninc/2+16;
for i=1:1:bpoints
	xbush1(i)=coord1(i,1);
	ybush1(i)=coord1(i,2);
end

% Rectify the point to the slider point circle.  First find the line from 
% the point chosen to the center of the circle.  Then find the point on the
% circle that is on the line.  The center of the circle is at x0, y0, and 
% the radius of the circle is rc.  
angle=atan2(fy1-y0, fx1-x0);
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
		
% Determine the coordinates of the slider line				
lineangle=Values(7)*fact;
lineangled=Values(7);

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

% Determine the linkage mode by checking the first position.
Bx=asx(1)-Astar(1);
By=asy(1)-Astar(2);
Q1=lineangled;
mode=assemblymode_sc(r2,r3,Bx,By,Q1,Q2);

% Find extreme values for r1.
r1max=0.9999*sqrt((r2+r3)^2-r4^2);
r1min=-r1max;

% Analyze the linkage for r1 between r1min and r1max
rd1=0;
rdd1=0;
thetas=0;
thetaf=1080;        % 3*360
A0=Astar;
theta1=Q1*180/pi;
flag=1;
Qd2=0;
Qdd2=0;
theta3=0;Bx(1)=0;By(1)=0;Cx(1)=0;Cy(1)=0;
dr=(r1max-r1min)/1000;   
%rone=r1min:dt:r1max;

% calculate two data set, one for mode 1 and the other for mode -1
% The data of mode -1 will be stored in Bx,By,Cx,Cy,Dx,Dy and Ex,Ey
% The data of mode 1 will be stored in B2x,B2y,C2x,C2y,D2x,D2y 
% and E2x,E2y
if mode == -1
   i = 0;

    for r1_increment = r1min:dr:r1max    
% Start from theta2 = 0 to 3*360 degrees

		points=sldcrks(r1_increment,r2,r3,r4,rd1,rdd1,mode,lineangled,flag);    
		if points(37)==1
			i=i+1;
			Bx(i)=points(25)+Astar(1);
			By(i)=points(26)+Astar(2);
			Cx(i)=points(27)+Astar(1);
			Cy(i)=points(28)+Astar(2);
			theta3=points(7)*pi/180;

% Locate original a and b on coupler

			Dx(i) = Bx(i)+rc1*cos(theta3+beta1);
			Dy(i) = By(i)+rc1*sin(theta3+beta1);
			Ex(i) = Bx(i)+rc2*cos(theta3+beta2);
			Ey(i) = By(i)+rc2*sin(theta3+beta2);
	
% Locate the slider block coordinates

			xpin=Cx(i);
			ypin=Cy(i);
			coords = rect(4*delta,2*delta,xpin,ypin,lineangled,0);
			for j=1:1:5
				xblocka(i,j)=coords(j,1);
				yblocka(i,j)=coords(j,2);
			end
		end
	end
   i = 0;
   mode = -mode;

    for r1_increment = r1min:dr:r1max

% Start from theta2 = 0 to 3*360 degrees

        points=sldcrks(r1_increment,r2,r3,r4,rd1,rdd1,mode,lineangled,flag); 
		if points(37)==1
			i=i+1;
			B2x(i)=points(25)+Astar(1);
			B2y(i)=points(26)+Astar(2);
			C2x(i)=points(27)+Astar(1);
			C2y(i)=points(28)+Astar(2);
			theta32=points(7)*pi/180;

% Locate original a and b on coupler

			D2x(i) = B2x(i)+rc1*cos(theta32+beta1);
			D2y(i) = B2y(i)+rc1*sin(theta32+beta1);
			E2x(i) = B2x(i)+rc2*cos(theta32+beta2);
			E2y(i) = B2y(i)+rc2*sin(theta32+beta2);
	
% Locate the slider block coordinates

			xpin=C2x(i);
			ypin=C2y(i);
			coords = rect(4*delta,2*delta,xpin,ypin,lineangled,0);
			for j=1:1:5
				xblocka2(i,j)=coords(j,1);
				yblocka2(i,j)=coords(j,2);
			end
		end
	end
elseif mode == 1
   i = 0;
    for r1_increment = r1min:dr:r1max
% Start from theta2 = 0 to 3*360 degrees

        points=sldcrks(r1_increment,r2,r3,r4,rd1,rdd1,mode,lineangled,flag); 
		if points(37)==1
			i=i+1;
			B2x(i)=points(25)+Astar(1);
			B2y(i)=points(26)+Astar(2);
			C2x(i)=points(27)+Astar(1);
			C2y(i)=points(28)+Astar(2);
			theta32=points(7)*pi/180;

% Locate original a and b on coupler

			D2x(i) = B2x(i)+rc1*cos(theta32+beta1);
			D2y(i) = B2y(i)+rc1*sin(theta32+beta1);
			E2x(i) = B2x(i)+rc2*cos(theta32+beta2);
			E2y(i) = B2y(i)+rc2*sin(theta32+beta2);
	
% Locate the slider block coordinates

			xpin=C2x(i);
			ypin=C2y(i);
			coords = rect(4*delta,2*delta,xpin,ypin,lineangled,0);
			for j=1:1:5
				xblocka2(i,j)=coords(j,1);
				yblocka2(i,j)=coords(j,2);
			end
		end
	end
   i = 0;
   mode = -mode;
    for r1_increment = r1min:dr:r1max
        
% Start from theta2 = 0 to 3*360 degrees

        points=sldcrks(r1_increment,r2,r3,r4,rd1,rdd1,mode,lineangled,flag); 
		if points(37)==1
			i=i+1;
			Bx(i)=points(25)+Astar(1);
			By(i)=points(26)+Astar(2);
			Cx(i)=points(27)+Astar(1);
			Cy(i)=points(28)+Astar(2);
			theta3=points(7)*pi/180;

% Locate original a and b on coupler

			Dx(i) = Bx(i)+rc1*cos(theta3+beta1);
			Dy(i) = By(i)+rc1*sin(theta3+beta1);
			Ex(i) = Bx(i)+rc2*cos(theta3+beta2);
			Ey(i) = By(i)+rc2*sin(theta3+beta2);
	
% Locate the slider block coordinates

			xpin=Cx(i);
			ypin=Cy(i);
			coords = rect(4*delta,2*delta,xpin,ypin,lineangled,0);
			for j=1:1:5
				xblocka(i,j)=coords(j,1);
				yblocka(i,j)=coords(j,2);
			end
		end
	end
end
itotal = i;
% Find the max and min values for Cx for the sliderline.
	[maxcx, kx]=max(Cx);
	[mincx, jx]=min(Cx);
	
% Determine the extended coordinates of the sliderline
	lineang=atan2(Cy(kx)-Cy(jx), Cx(kx)-Cx(jx));
	cosang=cos(lineang);
	sisang=sin(lineang);
	lineangd=lineang*180/pi;

	slength=sqrt((Cx(kx)-Cx(jx))^2+(Cy(kx)-Cy(jx))^2)+6*delta;
	ndash=30;
	xe = Cx(jx)-3*delta*cosang;
	ye = Cy(jx)-3*delta*sisang;
%	flag=1;
	coord=frameline(slength,xe,ye,ndash,lineangd,flag);					
	npoints1=3*ndash;
   
% Offset slider line by half the height of the slider block.
	for i=1:1:npoints1
		xsl1(i)=coord(i,1)+delta*sisang;
		ysl1(i)=coord(i,2)-delta*cosang;
	end					
   
% Find the max and min values for C2x for the sliderline.
	[maxcx, kx]=max(C2x);
	[mincx, jx]=min(C2x);
	
% Determine the extended coordinates of the sliderline
	lineang=atan2(C2y(kx)-C2y(jx), C2x(kx)-C2x(jx));
	cosang=cos(lineang);
	sisang=sin(lineang);
	lineangd=lineang*180/pi;

	slength=sqrt((C2x(kx)-C2x(jx))^2+(C2y(kx)-C2y(jx))^2)+6*delta;
	ndash=30;
	xe = C2x(jx)-3*delta*cosang;
	ye = C2y(jx)-3*delta*sisang;
	coord=frameline(slength,xe,ye,ndash,lineangd,flag);					
   
% Offset slider line by half the height of the slider block.
	for i=1:1:npoints1
		xsl12(i)=coord(i,1)+delta*sisang;
		ysl12(i)=coord(i,2)-delta*cosang;
	end					
% Calculate the axis range in different mode.  
% tempx and tempy are used for mode -1 while temp2x and temp2y are
% userd for mode 1.
	tempx=[Bx, Cx, Astar(1), ax, bx, xsl1];
	tempy=[By, Cy, Astar(2), ay, by, ysl1];
	temp2x=[B2x, C2x, Astar(1), ax, bx, xsl12];
	temp2y=[B2y, C2y, Astar(2), ay, by, ysl12];

% Calculate the axis range
width=0.8;
height=0.8;

% Define axes
xmins=min(tempx);
xmaxs=max(tempx);
xmins2=min(temp2x);
xmaxs2=max(temp2x);
ymins=min(tempy);
ymaxs=max(tempy);
ymins2=min(temp2y);
ymaxs2=max(temp2y);
rangex=xmaxs-xmins;
rangey=ymaxs-ymins;
range2x=xmaxs2-xmins2;
range2y=ymaxs2-ymins2;

% Define the plot limits
xmin=xmins-0.1*rangex;
ymin=ymins-0.1*rangey;
xmax=xmaxs+0.1*rangex;
ymax=ymaxs+0.1*rangey;
values=axisadjust(xmin, xmax, ymin, ymax, width, height);
xmin=values(1);
xmax=values(2);
ymin=values(3);
ymax=values(4);

xmin2=xmins2-0.1*range2x;
ymin2=ymins2-0.1*range2y;
xmax2=xmaxs2+0.1*range2x;
ymax2=ymaxs2+0.1*range2y;
values=axisadjust(xmin2, xmax2, ymin2, ymax2, width, height);
xmin2=values(1);
xmax2=values(2);
ymin2=values(3);
ymax2=values(4);


% Store position velocity and acceleration information in matrices.
res = zeros( 45, itotal);
i = 0;
% Bx,By,Cx,Cy,Dx,Dy and Ex,Ey are the data used to draw
% linkage in mode = -1
i = i+1; res( i,: ) = Bx;
i = i+1; res( i,: ) = By;

i = i+1; res( i,: ) = Cx;
i = i+1; res( i,: ) = Cy;

i = i+1; res( i,: ) = Dx;
i = i+1; res( i,: ) = Dy;

i = i+1; res( i,: ) = Ex;
i = i+1; res( i,: ) = Ey;

% B2x,B2y,C2x,C2y,D2x,D2y and E2x,E2y are the data used to draw
% linkage in mode = 1
i = i+1; res( i,: ) = B2x;
i = i+1; res( i,: ) = B2y;

i = i+1; res( i,: ) = C2x;
i = i+1; res( i,: ) = C2y;

i = i+1; res( i,: ) = D2x;
i = i+1; res( i,: ) = D2y;

i = i+1; res( i,: ) = E2x;
i = i+1; res( i,: ) = E2y;

% xsl1, ysl1 and xblocka, yblocka have their matrix sizes different from
% most of the other data. Therefore, they are taken care of separately.
% We just store xsl1 and ysl1 in part of the 'res' matrix. To retrieve
% only that part of matrix from the g_Anares variables later when xsl1 and
% ysl2 are needed, the variable npoints1 is stored in this global variable
% at res(41,12)
i = i+1; res( i,1:npoints1 ) = xsl1;
i = i+1; res( i,1:npoints1 ) = ysl1;

i = i+1; res( i,1:npoints1 ) = xsl12;
i = i+1; res( i,1:npoints1 ) = ysl12;

% xblocka and yblocka are matries with size (itotal,5)
% Because we need to store them into the 'res' variable, xblock and yblock are
% created to transpose these two matries. Then they use five columns of 'res' 
% to store them 
xblock = xblocka';
yblock = yblocka';
i = i+1; res( i:i+4,: ) = xblock;
i = i+5; res( i:i+4,: ) = yblock;

xblock = xblocka2';
yblock = yblocka2';
i = i+5; res( i:i+4,: ) = xblock;
i = i+5; res( i:i+4,: ) = yblock;

i = i+5; res( i,1:21 ) = xcoord1;
i = i+1; res( i,1:21 ) = ycoord1;

i = i+1; res( i,1:26 ) = xbush1;
i = i+1; res( i,1:26 ) = ybush1;

i = i+1;
res( i,1:4 ) = [xmin xmax ymin ymax];
res( i,5:8 ) = [xmin2 xmax2 ymin2 ymax2];
res( i,9:10 ) = Astar;
res( i,11:13 ) = ax;
res( i,14:16 ) = ay;
res( i,17:19 ) = bx;
res( i,20:22 ) = by;
res( i,23 ) = npoints1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fig Setting for Special Case
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = CreateMechanismHandles( handle,ax,ay,bx,by,xcoord,ycoord,xbush,ybush,axislimit,xsl1,ysl1,acx,acy,Astar) 

axes(handle);
%setup the aspect ratio of the mechanism to 1 
set(handle, 'DataAspectRatio',[1 1 1],'Color','none','box','off');
axis off;
xmin = axislimit(1);
xmax = axislimit(2);
ymin = axislimit(3);
ymax = axislimit(4);
axis([xmin xmax ymin ymax]);
position1=line('xdata', [], 'ydata' ,[], 'linewidth',2,'color', 'r');
position2=line('xdata', [], 'ydata' ,[], 'linewidth',2,'color', 'b');
position3=line('xdata', [], 'ydata' ,[], 'linewidth',2,'color', 'g');

a1=line('xdata', [], 'ydata', [], 'marker', '+','markersize',10, 'color', 'r');
a2=line('xdata', [], 'ydata', [], 'marker', '+','markersize',10, 'color', 'b');
a3=line('xdata', [], 'ydata', [], 'marker', '+','markersize',10, 'color', 'g');

sliderline1=line('xdata', [], 'ydata' ,[], 'linewidth',1,'color', 'r');

joint1=line('xdata', [], 'ydata',[], 'marker', 'o', 'markersize', 7,'color', 'k');
joint2=line('xdata', [], 'ydata', [], 'color', 'r');
joint3=line('xdata', [], 'ydata',[], 'marker', 'o', 'markersize', 7,'color','k');

hinge1=line('xdata', [], 'ydata',[],'color', 'r');
crank1=line('xdata', [], 'ydata', [],'linewidth',2,'color', 'k');
coupler1=line('xdata', [], 'ydata', [],'linewidth',2,'color', 'k');
coupler2=line('xdata', [], 'ydata' ,[], 'linewidth' ,2,'color', 'k');
coupler3=line('xdata', [], 'ydata' ,[], 'linewidth' ,2,'color', 'b');

block1=line('xdata', [], 'ydata',[],'linewidth' ,2, 'color', 'g');

% Plot fixed quantities
		 
set(position1, 'xdata', [ax(1) bx(1)], 'ydata', [ay(1) by(1)]);
set(position2, 'xdata', [ax(2) bx(2)], 'ydata', [ay(2) by(2)]);
set(position3, 'xdata', [ax(3) bx(3)], 'ydata', [ay(3) by(3)]);
set(joint2,'xdata', xcoord, 'ydata',ycoord);
set(hinge1,'xdata', xbush,'ydata', ybush);
set(sliderline1, 'xdata', xsl1, 'ydata', ysl1);

set(a1, 'xdata',ax(1), 'ydata', ay(1));
set(a2, 'xdata',ax(2), 'ydata', ay(2));
set(a3, 'xdata',ax(3), 'ydata', ay(3));

text(ax(1)+0.1,ay(1),'A1');
text(ax(2)+0.1,ay(2),'A2');
text(ax(3)+0.1,ay(3),'A3');
drawnow;
hdls=[ joint1 joint3 crank1 coupler1 coupler2 coupler3 block1];
set(handle,'userdata',hdls);

zoom on     % mouse left click / zoom box using mouse drag

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set the mechanism position
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = SetMechanismPosition( links,i,Bx,By,Cx,Cy,Dx,Dy,Ex,Ey,Astar,xblocka,yblocka)
joint1 = links(1);
joint3 = links(2);
crank1 = links(3);
coupler1 = links(4);
coupler2 = links(5);
coupler3 = links(6);
block1 = links(7);
set(joint1,'xdata', Bx(i), 'ydata',By(i));
set(joint3,'xdata', Cx(i), 'ydata',Cy(i));
set(crank1,'xdata',[Astar(1) Bx(i)],'ydata', [Astar(2) By(i)]);
set(coupler1,'xdata', [Cx(i), Bx(i)],'ydata', [Cy(i), By(i)]);
set(coupler2,'xdata', [Bx(i), Dx(i)],'ydata', [By(i), Dy(i)]);
set(coupler3,'xdata', [Dx(i), Ex(i)],'ydata', [Dy(i), Ey(i)]);		
for j=1:1:5
	xblock(j)=xblocka(i,j);
	yblock(j)=yblocka(i,j);
end
set(block1,'xdata', xblock, 'ydata',yblock);


