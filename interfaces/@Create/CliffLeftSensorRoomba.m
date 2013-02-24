function [state] = CliffLeftSensorRoomba(serPort);
%[state] = CliffLeftSensorRoomba(serPort)
% Specifies the state of the Cliff Left sensor
% Either triggered or not triggered

% By; Joel Esposito, US Naval Academy, 2011

%Initialize preliminary return values
state = nan;

try
    
%Flush Buffer    
N = serPort.BytesAvailable();
while(N~=0) 
fread(serPort,N);
N = serPort.BytesAvailable();
end

warning off
global td

fwrite(serPort, [142]);  fwrite(serPort,9); 
CliffLft = dec2bin(fread(serPort, 1));
state = bin2dec(CliffLft(end));



pause(td)
catch
    disp('WARNING:  function did not terminate correctly.  Output may be unreliable.')
end
