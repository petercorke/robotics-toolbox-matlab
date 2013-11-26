%UIWriteSubCodes UIWriteSubCodes enumeration for the brick
%
% Notes::
% - The majority of the UIWrite commands have been replaced by UIDraw
% - UIWriteSubCodes can be found in the EV3 documentation and source code
% (bytecodes.h)

classdef UIWriteSubCodes < uint8
    enumeration
         PutString(8)
         InitRun (25)
         Led (27)
    end
end