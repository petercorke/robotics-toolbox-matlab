%SystemCommands SystemCommands enumeration for the brick
%
% Notes::
% - SystemCommands can be found in the EV3 documentation and source code
% (c_com.c)

classdef SystemCommands < uint8
    enumeration
         BeginDownload (146)
         ContinueDownload (147)
         BeginUpload (148)
         ContinueUpload (149)
         BeginGetFile (150)
         ContinueGetFile (151)
         CloseFileHandle (152)
         ListFiles (153)
         ContinueListFiles (154)
         CreateDir (155)
         DeleteFile (156)
         ListOpenHandles (157)
         WriteMailBox (158)
         BlueToothPin (159)
         EnterFWUpdate (160)
         SetBundleID (161)
         SetBundleSeedID (162)
    end
end
