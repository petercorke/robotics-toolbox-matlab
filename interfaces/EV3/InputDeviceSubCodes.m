%InputDeviceSubCodes InputDeviceSubCodes enumeration for the brick
%
% Notes::
% - InputDeviceSubCodes can be found in the EV3 documentation and source code
% (bytecodes.h)

classdef InputDeviceSubCodes < uint8
    enumeration
         GetFormat (2)
         CalMinMax (3)
         CalDefault (4)
         GetTypeMode (5)
         GetSymbol (6)
         CalMin (7)
         CalMax (8)
         Setup (9)
         ClrAll (10)
         GetRaw (11)
         GetConnection (12)
         StopAll (13)
         GetName (21)
         GetModeName (22)
         SetRaw (23)
         GetFigures (24)
         GetChanges (25)
         ClrChanges (26)
         ReadyPct (27)
         ReadyRaw (28)
         ReadySI (29)
         GetMinMax (30)
         GetBumps (31)
    end
end
