function [BumpRight, BumpLeft, BumpFront, Wall, virtWall, CliffLft, ...
    CliffRgt, CliffFrntLft, CliffFrntRgt, LeftCurrOver, RightCurrOver, ...
    DirtL, DirtR, ButtonPlay, ButtonAdv, Dist, Angle, ...
    Volts, Current, Temp, Charge, Capacity, pCharge]   = AllSensorsReadRoomba(serPort);
%[BumpRight, BumpLeft, BumpFront, Wall, virtWall, CliffLft, ...
%   CliffRgt, CliffFrntLft, CliffFrntRgt, LeftCurrOver, RightCurrOver, ...
%   DirtL, DirtR, ButtonPlay, ButtonAdv, Dist, Angle, ...
%   Volts, Current, Temp, Charge, Capacity, pCharge]   = AllSensorsReadRoomba(serPort)
% Reads Roomba Sensors
% [BumpRight (0/1), BumpLeft(0/1), BumpFront(0/1), Wall(0/1), virtWall(0/1), CliffLft(0/1), ...
%    CliffRgt(0/1), CliffFrntLft(0/1), CliffFrntRgt(0/1), LeftCurrOver (0/1), RightCurrOver(0/1), ...
%    DirtL(0/1), DirtR(0/1), ButtonPlay(0/1), ButtonAdv(0/1), Dist (meters since last call), Angle (rad since last call), ...
%    Volts (V), Current (Amps), Temp (celcius), Charge (milliamphours), Capacity (milliamphours), pCharge (percent)]
% Can add others if you like, see code
% Esposito 3/2008
% initialize preliminary return values
% By; Joel Esposito, US Naval Academy, 2011
BumpRight = nan;
BumpLeft = nan;
BumpFront = nan;
Wall = nan;
virtWall = nan;
CliffLft = nan;
CliffRgt = nan;
CliffFrntLft = nan;
CliffFrntRgt = nan;
LeftCurrOver = nan;
RightCurrOver = nan;
DirtL = nan;
DirtR = nan;
ButtonPlay = nan;
ButtonAdv = nan;
Dist = nan;
Angle = nan;
Volts = nan;
Current = nan;
Temp = nan;
Charge = nan;
Capacity = nan;
pCharge = nan;


try

%Flush buffer
N = serPort.BytesAvailable();
while(N~=0) 
fread(serPort,N);
N = serPort.BytesAvailable();
end

warning off
global td
sensorPacket = [];
% flushing buffer
confirmation = (fread(serPort,1));
while ~isempty(confirmation)
    confirmation = (fread(serPort,26));
end


%% Get (142) ALL(0) data fields
fwrite(serPort, [142 0]);

%% Read data fields
BmpWheDrps = dec2bin(fread(serPort, 1),8);  %

BumpRight = bin2dec(BmpWheDrps(end))  % 0 no bump, 1 bump
BumpLeft = bin2dec(BmpWheDrps(end-1))
if BumpRight*BumpLeft==1
    BumpRight =0;
    BumpLeft = 0;
    BumpFront =1;
else
    BumpFront = 0;
end
Wall = fread(serPort, 1)  %0 no wall, 1 wall

CliffLft = fread(serPort, 1) % no cliff, 1 cliff
CliffFrntLft = fread(serPort, 1)
CliffFrntRgt = fread(serPort, 1)
CliffRgt = fread(serPort, 1)

virtWall = fread(serPort, 1)%0 no wall, 1 wall

motorCurr = dec2bin( fread(serPort, 1),8 );
Low1 = motorCurr(end);  % 0 no over curr, 1 over Curr
Low0 = motorCurr(end-1);  % 0 no over curr, 1 over Curr
Low2 = motorCurr(end-2);  % 0 no over curr, 1 over Curr
LeftCurrOver = motorCurr(end-3)  % 0 no over curr, 1 over Curr
RightCurrOver = motorCurr(end-4)  % 0 no over curr, 1 over Curr


DirtL = fread(serPort, 1)
DirtR = fread(serPort, 1)

RemoteCode =  fread(serPort, 1); % coudl be used by remote or to communicate with sendIR command
Buttons = dec2bin(fread(serPort, 1),8);
ButtonPlay = Buttons(end)
ButtonAdv = Buttons(end-2)

Dist = fread(serPort, 1, 'int16')/1000 % convert to Meters, signed, average dist wheels traveled since last time called...caps at +/-32
Angle = fread(serPort, 1, 'int16')*pi/180 % convert to radians, signed,  since last time called, CCW positive

ChargeState = fread(serPort, 1);
Volts = fread(serPort, 1, 'uint16')/1000
Current = fread(serPort, 1, 'int16')/1000 % neg sourcing, pos charging
Temp  =  fread(serPort, 1, 'int8') 
Charge =  fread(serPort, 1, 'uint16') % in mAhours
Capacity =  fread(serPort, 1, 'uint16')
pCharge = Charge/Capacity *100  % May be inaccurate
%checksum =  fread(serPort, 1)

pause(td)
catch
    disp('WARNING:  function did not terminate correctly.  Output may be unreliable.')
end
