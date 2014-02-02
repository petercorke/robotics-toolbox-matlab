%ByteCodes Bytecode enumeration for the brick
%
% Notes::
% - bytecodes can be found in the EV3 documentation and source code
% (bytecodes.h)

classdef ByteCodes < uint8
    enumeration
        %VM
        Error (0) % = 0x00,
        Nop (1) % = 0x01,
        ProgramStop (2) % = 0x02,
        ProgramStart (3) % = 0x03,
        ObjectStop (4) % = 0x04,
        ObjectStart (5) % = 0x05,
        ObjectTrig (6) % = 0x06,
        ObjectWait (7) % = 0x07,
        Return (8) % = 0x08,
        Call (9) % = 0x09,
        ObjectEnd (10) % = 0x0A,
        Sleep (11) % = 0x0B,
        ProgramInfo (12) % = 0x0C,
        Label(13) % = 0x0D,
        Probe(14) % = 0x0E,
        Do(15) % = 0x0F,
        
        %Move
        InitBytes (47) %= 0x2F,
        
        %Branch
        Jr (64) %= 0x40,
        
        %VM
        Info (124) %= 0x7C,
        String (125) %= 0x7D,
%        MemoryWrite = 0x7E,
%        MemoryRead = 0x7F, 
        
        %UI
        UIFlush (128) %= 0x80,
        UIRead (129) %= 0x81,
        UIWrite (130) %= 0x82,
        UIButton (131) %= 0x83,
        UIDraw (132) %= 0x84,
        
        %Timer
        TimerWait (133) %= 0x85,
        TimerReady (134) %= 0x86,
        TimerRead (135) %= 0x87,
        
        %Sound
        Sound (148) %= 0x94,
        SoundTest (149) %= 095,
        SoundReady (150) % = 0x96,
        
%       %Input
%       InputSample = 0x97,
        InputDeviceList (152) % = 0x98,
        InputDevice (153) %= 0x99,
        InputRead (154) % = 0x9a,
%       InputTest = 0x9b,
        InputReady (156) % = 0x9c,
        InputReadSI (157) % = 0x9d,
%       InputReadExt = 0x9e,
%       InputWrite = 0x9f,
%       
%       %Output
%       OutputGetType = 0xa0,
        OutputSetType (161) % = 0xa1,         
        OutputReset (162) % = 0xa2,           
        OutputStop (163) % = 0xA3,
        OutputPower (164) % = 0xA4,
        OutputSpeed (165) % = 0xA5,
        OutputStart	(166) % = 0xA6,
        OutputPolarity (167) % = 0xA7,
        OutputRead (168) % = 0xA8,
        OutputTest (169) % = 0xA9,
        OutputReady (170) % = 0xAA,
%       OutputPosition = 0xAB, NOT IMPLEMENTED IN THE CODE
        OutputStepPower (172) % = 0xAC,
        OutputTimePower (173) % = 0xAD,
        OutputStepSpeed (174) % = 0xAE,
        OutputTimeSpeed (175) % = 0xAF,
        OutputStepSync (176) % = 0xB0,
        OutputTimeSync (177) % = 0xB1,
        OutputClrCount (178) % = 0xB2,
        OutputGetCount (179) % = 0xB3,
% 
%       %Memory
%        File = 0xC0,
%        Array = 0xc1,
%        ArrayWrite = 0xc2,
%        ArrayRead = 0xc3,
%        ArrayAppend = 0xc4,
%        MemoryUsage = 0xc5,
%        FileName = 0xc6,
%        

         %Communications
         COMGet (211) % = 0xD3,
         COMSet (212) % = 0xD4, 

%       %Mailbox
         MailboxOpen (216) % = 0xD8,
         MailboxWrite (217) % = 0xD9,
         MailboxRead (218) % = 0xDA,
         MailboxTest (219) % = 0xDB,
         MailboxReady (220) % = 0xDC,
         MailboxClose (221) % = 0xDD,
    end
end
