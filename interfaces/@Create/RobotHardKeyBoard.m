function [keyPresses , elapsedTime] = RobotHardKeyBoard(CommPort)
%function [keyPresses , elapsedTime] = RobotHardKeyBoard(CommPort);
%permits control of robot via numerical key pad on a physical key board
% Numerical keypad presets:
%8-moves Roomba forward 10cm
%5-moves Roomba backward 5cm
%4-turns Roomba left 15 degrees
%7-slightly turns Roomba left 5 degrees
%6-turns Roomba right 15 degrees
%9-slightly turns Roomba right 5 degrees

%Alphabetical key presets:
%y-moves Roomba forward 10cm
%h-moves Roomba backward 5cm
%g-turns Roomba left 15 degrees
%t-slightly turns Roomba left 5 degrees
%j-turns Roomba right 15 degrees
%u-slightly turns Roomba right 5 degrees
%
% q quits 
%
% input is a number for the comm port ex.   use 1, for COM1
% if omitted, set to default to 9.
% outputs: keypresses = total number of key pressed  (q doesn't count, but
% all other keys, including inactive ones ex 'a' do
% elapsedTime time in seconds since first keypress. can be 0 if q pressed
% immediately
disp('Wait for robot beep...') 
a = instrfind();
if ~isempty(a)
    fclose(a);
    pause(1);
    delete(a);
    pause(1);
end

if nargin==0
    CommPort = 9;
end
    
ser = RoombaInit(CommPort);

im = imread('KeyBoardControl.png');


%logging variables
keyPresses = [0 0];
startTime = [];
elapsedTime = [];
%-------------------------------------------
%        SWITCH SPEEDS AND ANGLES HERE
%
% bD=distance forward sD=distance backward
% sA=small turn angle bA=big turn angle
% sV=backwardspeed bV=forwardspeed
% sW=small angle speed bW=big angle speed
%
%-------------------------------------------
%
% speeds and angles
sV = .1; sD = 0.05; % vel in m/s angles in deg W is angluar speed but in m/s?
bV = .2; bD = 0.1; 
sW = .05; sA = 5; 
bW = .1; bA = 15; 

% set up hidden key logger window
callstr = ['set(gcbf,''Userdata'',double(get(gcbf,''Currentcharacter''))) ; uiresume '];
fh = figure('keypressfcn',callstr, ...
    'windowstyle','modal',...
    'units', 'normalized',...
    'position',[.1 .1 .8 .8],...
    'Name','GETKEY', ...
    'userdata','timeout') ;
imshow(im);

disp('Starting Keyboard Mode.   Press q to quit.   ')

while(1)
  uiwait ;
  try      
      %disp('here');  why does a single keypress appear to trigger two
      %events.   First with empty, then actual character????   JME 5/26
      k = char( get(fh,'Userdata') );
    
  catch
    k = nan ;
  end

  if ~isempty(k)&& k~='q'
    keyPresses(1) = keyPresses(1) +1;
    %disp(k)
  end
if isempty(startTime)
    startTime = tic;
end
%-------------------------------------
%       SWITCH KEYBOARD KEYS HERE
%-------------------------------------
if ~isempty(k)
switch k
    %2
    case {'5','h'}
        travelDist(ser, sV, -sD);
    case {'4','g'}
        turnAngle(ser, bW, bA);
    case {'6','j'}
        turnAngle(ser, bW, -bA);
    case {'7','t'}
        turnAngle(ser, sW, sA);
    case {'8','y'}
        travelDist(ser, bV, bD);
    case {'9','u'}
        turnAngle(ser, sW, -sA);
    case '0'
        elapsedTime = toc(startTime);
        fprintf('Sec:  %.2f,  Keystrokes: %d, NegKey: %d \n ', [elapsedTime keyPresses(1) keyPresses(2)]);
    case 'q'
        if ~isempty(startTime)
        elapsedTime = toc(startTime);
        else
            elapsedTime =0;
        end
        fprintf('Sec:  %.2f,  Keystrokes: %d, NegKey: %d \n ', [elapsedTime keyPresses(1) keyPresses(2)]);
        delete(fh);
        break
    otherwise
        disp('Otherwise')
        keyPresses(2) = keyPresses(2) +1;
end % end switch k
end % if ~isempty(k)        
end % end while(1)
