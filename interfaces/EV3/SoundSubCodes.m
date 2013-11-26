%SoundSubCodes SoundSubCodes enumeration for the brick
%
% Notes::
% - SoundSubCodes can be found in the EV3 documentation and source code
% (bytecodes.h)

classdef SoundSubCodes < uint8
    enumeration
         Break (0)
         Tone (1)
         Play (2)
         Repeat (3)
         Service (4)
    end
end
