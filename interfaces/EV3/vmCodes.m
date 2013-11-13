%vmCodes vmCodes enumeration for the brick
%
% Notes::
% - vmCodes can be found in the EV3 documentation and source code
% (bytecodes.h from line 39 onwards)

classdef vmCodes < uint8
    enumeration
        % hardware
        vmOutputs (4)
        vmInputs (4)
        vmButtons (6)
        vmLeds (4)
        
        vmLCDWidth (178)
        vmLCDHeight (128)
        vmToplineHeight (10)
        vmLCDStoreLevels (3)
        
        vmDefaultVolume (100)
        vmDefaultSleepminutes (30)
        
        vmFGColor (1)
        vmBGColor (0)
    end
end
