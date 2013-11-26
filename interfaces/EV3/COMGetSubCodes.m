%COMGetSubCodes COMGetSubCodes enumeration for the brick
%
% Notes::
% - COMGetSubCodes can be found in the EV3 documentation and source code
% (bytecodes.h)

classdef COMGetSubCodes < uint8
    enumeration
         GetOnOff(1)
         GetVisible (2)
         GetResult (4)
         GetPin (5)
         SearchItems (8)
         SearchItem (9)
         FavourItems (10)
         FavourItem (11)
         GetID (12)
         GetBrickName (13)
         GetNetwork (14)
         GetPresent (15)
         GetEncrypt (16)
         ConnecItems (17)
         ConnecItem (18)
         GetIncoming (19)
         GetMode2 (20)
    end
end