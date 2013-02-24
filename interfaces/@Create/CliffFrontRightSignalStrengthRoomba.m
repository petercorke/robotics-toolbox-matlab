function [Signal] = CliffFrontRightSignalStrengthRoomba(serPort);
%[Signal] = CliffFrontRightSignalStrengthRoomba(serPort)
%Displays the strength of the front right cliff sensor's signal.
%Ranges between 0-100 percent signal



% By; Joel Esposito, US Naval Academy, 2011

%Initialize preliminary return values
Signal = nan;

try
    
%Flush Buffer    
N = serPort.BytesAvailable();
while(N~=0) 
fread(serPort,N);
N = serPort.BytesAvailable();
end

warning off
global td
fwrite(serPort, [142]);  fwrite(serPort,30);

Strength =  fread(serPort, 1, 'uint16');
Signal=(Strength/4095)*100;

pause(td)
catch
    disp('WARNING:  function did not terminate correctly.  Output may be unreliable.')
end