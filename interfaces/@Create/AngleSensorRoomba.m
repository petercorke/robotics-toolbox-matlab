function [AngleR] = AngleSensorRoomba(serPort);
%[AngleR] = AngleSensorRoomba(serPort)
% Displays the angle in radians and degrees that Create has turned since the angle was last requested.
% Counter-clockwise angles are positive and Clockwise angles are negative.


% By; Joel Esposito, US Naval Academy, 2011

%Initialize preliminary return values
AngleR = nan;

try
   
%Flush Buffer    
N = serPort.BytesAvailable();
while(N~=0) 
fread(serPort,N);
N = serPort.BytesAvailable();
end

warning off
global td

fwrite(serPort, [142]);  fwrite(serPort,20);

AngleR = fread(serPort, 1, 'int16')*pi/180;
pause(td)
catch
    disp('WARNING:  function did not terminate correctly.  Output may be unreliable.')
end