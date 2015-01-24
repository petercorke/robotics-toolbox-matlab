function [ButtonAdv,ButtonPlay] = ButtonsSensorRoomba(serPort);
%[ButtonAdv,ButtonPlay] = ButtonsSensorRoomba(serPort)
%Displays the state of Create's Play and Advance buttons, either pressed or
%not pressed.

%initialize preliminary return values
ButtonAdv = nan;
ButtonPlay = nan;

% By; Joel Esposito, US Naval Academy, 2011
try
    
%Flush Buffer    
N = serPort.BytesAvailable();
while(N~=0) 
fread(serPort,N);
N = serPort.BytesAvailable();
end

warning off
global td


fwrite(serPort, [142]);  fwrite(serPort,18);

Buttons = dec2bin(fread(serPort, 1),8);
ButtonAdv = bin2dec(Buttons(end-2));
ButtonPlay = bin2dec(Buttons(end));

pause(td)
catch
    disp('WARNING:  function did not terminate correctly.  Output may be unreliable.')
end
