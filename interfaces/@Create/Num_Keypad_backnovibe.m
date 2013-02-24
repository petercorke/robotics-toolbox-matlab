function varargout = RobotGuiControl(varargin)
%varargout = Num_Keypad_backnovibe(varargin)
%Large Forward Arrow-Moves Roomba forward 10cm
%Small Forward Arrow-Moves Roomba forward 5cm
%Back Arrow-Moves Roomba backward 5cm
%Large Left Arrow-Turns Roomba  15 degrees left
%Small Left Arrow-Turns Roomba 5 degrees left
%Large Right Arrow-Turns Roomba 15 degrees right
%Small Right Arrow-Turns Roomba 5 degrees right
% Description: Keypad with No Feedback.  Also, creates
% a global variable called 'list', which contains all the numeric presses,

% NUM_KEYPAD_BACKNOVIBE M-file for Num_Keypad_backnovibe.fig
%      NUM_KEYPAD_BACKNOVIBE, by itself, creates a new NUM_KEYPAD_BACKNOVIBE or raises the existing
%      singleton*.
%      H = NUM_KEYPAD_BACKNOVIBE returns the handle to a new NUM_KEYPAD_BACKNOVIBE or the handle to
%      the existing singleton*.
%
%      NUM_KEYPAD_BACKNOVIBE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in NUM_KEYPAD_BACKNOVIBE.M with the given input arguments.
%
%      NUM_KEYPAD_BACKNOVIBE('Property','Value',...) creates a new NUM_KEYPAD_BACKNOVIBE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Num_Keypad_backnovibe_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Num_Keypad_backnovibe_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% % Begin initialization code - DO NOT EDIT
%BY; Joel Esposito, US Naval Academy, 2011
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Num_Keypad_backnovibe_OpeningFcn, ...
                   'gui_OutputFcn',  @Num_Keypad_backnovibe_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end


if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before Num_Keypad_backnovibe is made visible.
function RobotGuiControl_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Num_Keypad_backnovibe (see VARARGIN)

% Choose default command line output for Num_Keypad_backnovibe

%Global Variables and Startup Processes


% -------------------------------------------------------------------------


%                           START COPY!


%--------------------------------------------------------------------------


% Cleaning up old ports


serPort = varargin{1};
global a
a = instrfind();
if ~isempty(a)
    fclose(a);
    pause(1);
    delete(a);
    pause(1);
end

global ser
% serial port for vibration

% serial port for sending characters
ser = RoombaInit(serPort);

%logging
global keyPresses startTime elapsedTime 
keyPresses = [0 0];
startTime = [];
elapsedTime = [];

% speeds and angles
global sV sD bV bD sW sA bW bA
sV = .1; sD = 0.05; % vel in m/s angles in deg W is angluar speed but in m/s?
bV = .2; bD = 0.1; 
sW = .05; sA = 5; 
bW = .1; bA = 15; 

global moving movingTest
movingTest = 1; 
moving=0;


handles.output = hObject;



% Update handles structure
guidata(hObject, handles);



% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


%------------------------COPY TO ALL OTHERS
global ser keyPresses startTime elapsedTime 
global sV sD bV bD sW sA bW bA
keyPresses(1) = keyPresses(1) +1;
keyPresses(2) = keyPresses(2) +1;
if isempty(startTime)
    startTime = tic;
end
%--------------------------



% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% global list
% list=[list 2];
%------------------------COPY TO ALL OTHERS
global ser keyPresses startTime elapsedTime 
global sV sD bV bD sW sA bW bA
keyPresses(1) = keyPresses(1) +1;

if isempty(startTime)
    startTime = tic;
end

global moving movingTest 
if (movingTest ==0)||(moving ==0)
moving=1;
travelDist(ser, sV, -sD);
moving=0;
end
%--------------------------

% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% global list
% list=[list 3];
%------------------------COPY TO ALL OTHERS
global ser keyPresses startTime elapsedTime 
global sV sD bV bD sW sA bW bA

keyPresses(1) = keyPresses(1) +1;
keyPresses(2) = keyPresses(2) +1;
if isempty(startTime)
    startTime = tic;
end

%--------------------------

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% global list
% list=[list 4];

%------------------------COPY TO ALL OTHERS
global ser keyPresses startTime elapsedTime 
global sV sD bV bD sW sA bW bA
keyPresses(1) = keyPresses(1) +1;
if isempty(startTime)
    startTime = tic;
end


global moving movingTest
if (movingTest ==0)||(moving ==0)
moving=1;
turnAngle(ser, bW, bA);
moving=0;
end

%--------------------------


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% global list
% list=[list 5];
global ser keyPresses startTime elapsedTime 
global sV sD bV bD sW sA bW bA

keyPresses(1) = keyPresses(1) +1;
if isempty(startTime)
    startTime = tic;
end

global moving movingTest
if (movingTest ==0)||(moving ==0)
moving=1;
travelDist(ser, sV, sD);
moving=0;
end



% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% global list
% list=[list 6];
%------------------------COPY TO ALL OTHERS
global ser keyPresses startTime elapsedTime 
global sV sD bV bD sW sA bW bA
keyPresses(1) = keyPresses(1) +1;
if isempty(startTime)
    startTime = tic;
end


global moving movingTest
if (movingTest ==0)||(moving ==0)
moving=1;
turnAngle(ser, bW, -bA);
moving=0;
end

%--------------------------

% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% global list
% list=[list 7];
%------------------------COPY TO ALL OTHERS
global ser keyPresses startTime elapsedTime 
global sV sD bV bD sW sA bW bA

keyPresses(1) = keyPresses(1) +1;
if isempty(startTime)
    startTime = tic;
end

global moving movingTest
if (movingTest ==0)||(moving ==0)
moving=1;
turnAngle(ser, sW, sA);
moving=0;
end

%--------------------------

% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% global list
% list=[list 8];
%--------
%------------------------COPY TO ALL OTHERS
global ser keyPresses startTime elapsedTime 
global sV sD bV bD sW sA bW bA
keyPresses(1) = keyPresses(1) +1;
if isempty(startTime)
    startTime = tic;
end


global moving movingTest
if (movingTest ==0)||(moving ==0)
moving=1;
travelDist(ser, bV, bD);
moving=0;
end

%--------------------------


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% global list
% list=[list 9];

%------------------------COPY TO ALL OTHERS
global ser keyPresses startTime elapsedTime 
global sV sD bV bD sW sA bW bA

keyPresses(1) = keyPresses(1) +1;
if isempty(startTime)
    startTime = tic;
end


global moving movingTest
if (movingTest ==0)||(moving ==0)
moving=1;
turnAngle(ser, sW, -sA);
moving=0;
end

%--------------------------

% --- Executes on button press in pushbutton_back.
function pushbutton_back_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_back (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%--------
global ser keyPresses startTime elapsedTime 
global sV sD bV bD sW sA bW bA

keyPresses(1) = keyPresses(1) +1;
keyPresses(2) = keyPresses(2) +1;

if isempty(startTime)
    startTime = tic;
end





% --- Executes on button press in pushbutton_del.
function pushbutton_del_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_del (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%--------
global ser keyPresses startTime elapsedTime 
global sV sD bV bD sW sA bW bA
keyPresses(1) = keyPresses(1) +1;
keyPresses(2) = keyPresses(2) +1;

if isempty(startTime)
    startTime = tic;
end


% --- Executes on button press in pushbutton_enter.
function pushbutton_enter_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_enter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global ser keyPresses startTime elapsedTime 
global sV sD bV bD sW sA bW bA
global NegVibe  NegTime  PosVibe PosTime
keyPresses(1) = keyPresses(1) +1;
if isempty(startTime)
    startTime = tic;
else
   elapsedTime = toc(startTime);
   fprintf('Sec:  %f,  Keystrokes: %d, NegPresses: %d \n ', [elapsedTime keyPresses(1) keyPresses(2)]);
end

%--------------------------------------------------------------------------



%                       END COPY


%-------------------------------------------------------------------------


% --- Executes when figure1 is resized.
function figure1_ResizeFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% global list
% list=[list .];
%--------
global ser keyPresses startTime elapsedTime 
global sV sD bV bD sW sA bW bA

keyPresses = keyPresses +1;
if isempty(startTime)
    startTime = tic;
end

% UIWAIT makes Num_Keypad_backnovibe wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = RobotGuiControl_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
%global keyPresses elapsedTime
