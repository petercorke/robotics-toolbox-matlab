function [Current] = CurrentTesterRoomba(serPort);
%[Current] = CurrentTesterRoomba(serPort)
% Displays the current (in amps) flowing into or out of Create's battery. 
% Negative currents indicate that current is flowing out of the battery.
% Positive currents indicate that current is flowing into the battery.


% By; Joel Esposito, US Naval Academy, 2011

%Initialize preliminary return values
Current = nan;

try

%Flush Buffer    
N = serPort.BytesAvailable();
while(N~=0) 
fread(serPort,N);
N = serPort.BytesAvailable();
end

warning off
global td
fwrite(serPort, [142]);  fwrite(serPort,23);

Current = fread(serPort, 1, 'int16')/1000;

pause(td)
catch
    disp('WARNING:  function did not terminate correctly.  Output may be unreliable.')
end
