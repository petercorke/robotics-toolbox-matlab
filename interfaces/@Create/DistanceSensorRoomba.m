function [Distance] = DistanceSensorRoomba(serPort);
%[Distance] = DistanceSensorRoomba(serPort)
% Gives the distance traveled in meters since last requested. Positive
% values indicate travel in the forward direction. Negative values indicate
% travel in the reverse direction. If not polled frequently enough, it is
% capped at its minimum or maximum of +/- 32.768 meters.


% By; Joel Esposito, US Naval Academy, 2011

%Initialize preliminary return values
Distance = nan;

try
    
%Flush Buffer    
N = serPort.BytesAvailable();
while(N~=0) 
fread(serPort,N);
N = serPort.BytesAvailable();
end

warning off
global td
fwrite(serPort, [142]);  fwrite(serPort,19);

Distance = fread(serPort, 1, 'int16')/1000;
if (Distance > 32) | (Distance <-32)
    disp('Warning:  May have overflowed')
end

pause(td)
catch
    disp('WARNING:  function did not terminate correctly.  Output may be unreliable.')
end
