function [] = SetFwdVelAngVelCreate(serPort, FwdVel, AngVel )
%[] = SetFwdVelAngVelCreate(serPort, FwdVel, AngVel )
%  Specify forward velocity in meters/ sec
%  [-0.5, 0.5].   Specify Angular Velocity in rad/sec.  Negative velocity is backward/Clockwise.  Caps overflow.
%  Note that the wheel speeds are capped at .5 meters per second.  So it is possible to 
% specify speeds that cannot be acheived.  Warning is displayed. 
% Only works with Create I think...not Roomba
% By; Joel Esposito, US Naval Academy, 2011
try
    
%Flush Buffer    
N = serPort.BytesAvailable();
while(N~=0) 
fread(serPort,N);
N = serPort.BytesAvailable();
end

global td
d = .258; % wheel baseline
wheelVel = inv([.5 .5; 1/d -1/d])*[FwdVel; AngVel];
rightWheelVel = min( max(1000* wheelVel(1), -500) , 500);
leftWheelVel = min( max(1000* wheelVel(2), -500) , 500);
if ( abs(rightWheelVel) ==500) |  ( abs(leftWheelVel) ==500)
   disp('Warning: desired velocity combination exceeds limits') 
end
fwrite(serPort, [145]);  fwrite(serPort,rightWheelVel, 'int16'); fwrite(serPort,leftWheelVel, 'int16');
pause(td)
catch
    disp('WARNING:  function did not terminate correctly.  Output may be unreliable.')
end
