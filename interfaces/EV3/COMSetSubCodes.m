%COMSetSubCodes COMSetSubCodes enumeration for the brick
%
% Notes::
% - COMSetSubCodes can be found in the EV3 documentation and source code
% (bytecodes.h)

classdef COMSetSubCodes < uint8
    enumeration
         SetOnOff (1)
         SetVisible (2)
         SetSearch (3)
         SetPin (5)
         SetPasskey (6)
         SetConnection (7)
         SetBrickName (8)
         SetMoveUp (9)
         SetMoveDown (10)
         SetEncrypt (11)
         SetSSID (12)
         SetMode2 (13)
    end
end