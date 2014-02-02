% EV3 command construction
%
% Methods::
% Command                   Constructor, creates an empty command object
% delete                    Destructor, clears the command object
%
% addHeaderSystem           Add a system header to the command object
% addHeaderSystemReply      Add a system header with reply to the command object
% addHeaderDirect           Add a direct header to the command object
% addHeaderDirectReply      Add a direct header with reply to the command object
% addLength                 Add the length of the msg to the front of the command object
% addSystemCommand          Add a system command to the command object
% addDirectCommand          Add a direct command to the command object
%
% clear                     Clear the command msg
% display                   Display the command msg (decimal)
% displayHex                Display thed command msg (hex)
%
% LC0                       Add a local constant 0 to the command object
% LC1                       Add a local constant 1 to the command object
% LC2                       Add a local constant 2 to the command object
% LC4                       Add a local constant 4 to the command object
% LV0                       Add a local variable 0 to the command object
% GV0                       Add a global variable 0 to the command object
% LCS                       Add a local constant string to the command object
%
% addValue                  Add a numerical value to the command object
% addArray                  Add a numerical array to the command object
% addString                 Add a string to the command object
% addLCSString              Add a LCS string to the command object
%
% LONGToBytes               Add a LONGToBytes to the command object
% WORDToBytes               Add a WORDToBytes to the command object
% BYTEToBytes               Add a BYTEToBytes to the command object
%
% PROGRAMHeader             Add a PROGRAMHeader to the command object
% addFileSize               Add the filesize to the command object
% VMTHREADHeader            Add a VMTHREADHeader to the command object
% SUBCALLHeader             Add a SUBCALLHeader to the command object
% BLOCKHeader               Add a BLOCKHeader to the command object
% GenerateByteCode          Print the command message to a file
%
% opNOP                     Add a opNOP opcode to the command object
% opOBJECT_END              Add a opOBJECT_END opcode to the command object
%
% opJR                      Add a opJR opcode to the command object
%
% opUI_FLUSH                Add a opUI_FLUSH opcode to the command object
%
% opUI_READ_GET_VBATT       Add a opUI_READ opcode with a GET_VBATT subcode to the command object
% opUI_READ_GET_LBATT       Add a opUI_READ opcode with a GET_LBATT subcode to the command object
%
% opUI_WRITE_PUT_STRING     Add a opUI_WRITE opcode with a PUT_STRING subcode to the command object
% opUI_WRITE_INIT_RUN       Add a opUI_WRITE opcode with a INIT_RUN subcode to the command object
% opUI_WRITE_LED            Add a opUI_WRITE opcode with a LED subcode to the command object
%
% opUI_DRAW_UPDATE          Add a opUI_DRAW opcode with a UPDATE subcode to the command object
% opUI_DRAW_CLEAN           Add a opUI_DRAW opcode with a CLEAN subcode to the command object
% opUI_DRAW_PIXEL           Add a opUI_DRAW opcode with a PIXEL subcode to the command object
% opUI_DRAW_LINE            Add a opUI_DRAW opcode with a LINE subcode to the command object
% opUI_DRAW_CIRCLE          Add a opUI_DRAW opcode with a CIRCLE subcode to the command object
% opUI_DRAW_TEXT            Add a opUI_DRAW opcode with a TEXT subcode to the command object
% opUI_DRAW_VALUE           Add a opUI_DRAW opcode with a VALUE subcode to the command object
% opUI_DRAW_FILLRECT        Add a opUI_DRAW opcode with a FILLRECT subcode to the command object
% opUI_DRAW_RECT            Add a opUI_DRAW opcode with a RECT subcode to the command object
% opUI_DRAW_INVERSERECT     Add a opUI_DRAW opcode with a INVERSERECT subcode to the command object
% opUI_DRAW_SELECT_FONT     Add a opUI_DRAW opcode with a SELECT_FONT subcode to the command object
% opUI_DRAW_TOPLINE         Add a opUI_DRAW opcode with a TOPLINE subcode to the command object
% opUI_DRAW_FILLWINDOW      Add a opUI_DRAW opcode with a FILLWINDOW subcode to the command object
% opUI_DRAW_FILLCIRCLE      Add a opUI_DRAW opcode with a FILLCIRCLE subcode to the command object
% opUI_DRAW_STORE           Add a opUI_DRAW opcode with a STORE subcode to the command object
% opUI_DRAW_RESTORE         Add a opUI_DRAW opcode with a RESTORE subcode to the command object
%
% opTIMER_WAIT              Add a opTIMER opcode with a WAIT subcode to the command object
% opTIMER_READY             Add a opTIMER opcode with a READY subcode to the command object
% opTIMER_READ              Add a opTIMER opcode with a READ subcode to the command object
%
% opSOUND_BREAK             Add a opSOUND opcode with a BREAK subcode to the command object
% opSOUND_TONE              Add a opSOUND opcode with a TONE subcode to the command object
% opSOUND_PLAY              Add a opSOUND opcode with a PLAY subcode to the command object
% opSOUND_REPEAT            Add a opSOUND opcode with a REPEAT subcode to the command object
% opSOUND_TEST              Add a opSOUND opcode with a TEST subcode to the command object
% opSOUND_READY             Add a opSOUND opcode with a READY subcode to the command object
%
% opINPUT_DEVICE_LIST       Add a opINPUT opcode with a DEVICE_LIST subcode to the command object
%
% opINPUT_DEVICE_GET_TYPEMODE Add a INPUT_DEVICE opcode with a GET_TYPEMODE subcode to the command object
% opINPUT_DEVICE_GET_SYMBOL Add a INPUT_DEVICE opcode with a GET_SYMBOL subcode to the command object
% opINPUT_DEVICE_CLR_ALL    Add a INPUT_DEVICE opcide with a CLR_ALL subcode to the command object
% opINPUT_DEVICE_GET_NAME   Add a INPUT_DEVICE opcode with a GET_NAME subcode to the command object.
%
% opINPUT_READ              Add a opINPUT_READ opcode to the command object
% opINPUT_READY             Add a opINPUT_READY opcode to the command object
% opINPUT_READSI            Add a opINPUT_READSI opcode to the command object
%
% opOUTPUT_SET_TYPE         Add a opOUTPUT_SET_TYPE opcode to the command object
% opOUTPUT_RESET            Add a opOUTPUT_RESET opcode to the command object
% opOUTPUT_STOP             Add a opOUTPUT_STOP opcode to the command object
% opOUTPUT_SPEED            Add a opOUTPUT_SPEED opcode to the command object
% opOUTPUT_POWER            Add a opOUTPUT_POWER opcode to the command object
% opOUTPUT_START            Add a opOUTPUT_START opcode to the command object
% opOUTPUT_POLARITY         Add a opOUTPUT_POLARITY opcode to the command object
% opOUTPUT_READ             Add a opOUTPUT_READ opcode to the command object
% opOUTPUT_TEST             Add a opOUTPUT_TEST opcode to the command object
% opOUTPUT_READY            Add a opOUTPUT_READY opcode to the command object
% opOUTPUT_STEP_POWER       Add a opOUTPUT_STEP_POWER opcode to the command object
% opOUTPUT_STEP_SPEED       Add a opOUTPUT_STEP_SPEED opcode to the command object
% opOUTPUT_TIME_SPEED       Add a opOUTPUT_TIME_SPEED opcode to the command object
% opOUTPUT_STEP_SYNC        Add a opOUTPUT_STEP_SYNC opcode to the command object
% opOUTPUT_TIME_SYNC        Add a opOUTPUT_TIME_SYNC opcode to the command object
% opOUTPUT_CLR_COUNT        Add a opOUTPUT_CLR_COUNT opcode to the command object
% opOUTPUT_GET_COUNT        Add a opOUTPUT_GET_COUNT opcode to the command object
%
% opCOMGET_SET_BRICKNAME    Add a opCOMGET opcode with a GET_BRICKNAME subcode to the command object
% opCOMSET_SET_BRICKNAME    Add a opCOMSET opcode with a SET_BRICKNAME subcode to the command object
%
% opMAILBOX_WRITE           Add a opMAILBOX_WRITE opcode to the command object
%
% BEGIN_DOWNLOAD            Add a BEGIN_DOWNLOAD system command to the command object
% CONTINUE_DOWNLOAD         Add a CONTINUE_DOWNLOAD system command to the command object
% BEGIN_UPLOAD              Add a BEGIN_UPLOAD system command to the command object
% CONTINUE_UPLOAD           Add a CONTINUE_UPLOAD system command to the command object
% LIST_FILES                Add a LIST_FILES system command to the command object
% CONTINUE_LIST_FILES       Add a CONTINUE_LIST_FILES system command to the command object
% CREATE_DIR                Add a CREATE_DIR system command to the command object
% DELETE_FILE               Add a DELETE_FILE system command to the command object
% WRITEMAILBOX              Add a WRITEMAILBOX system command to the command object
% 
% Notes::
% - Refer to the EV3 documentation or source code for a more detailed
% description of the commands.
%
% Example::
%                   cmd = Command();
%                   cmd.addHeaderDirect(42,0,0);
%                   cmd.opSOUND_TONE(volume,frequency,duration);
%                   cmd.addLength();

classdef Command < handle

    % Communications format (c_com.h):
    % /*
    %       System Command Bytes:
    %   ,------,------,------,------,------,------,------,------,
    %   |Byte 0|Byte 1|Byte 2|Byte 3|Byte 4|Byte 5|      |Byte n|
    %   '------'------'------'------'------'------'------'------'
    % 
    %   Byte 0 – 1: Command size, Little Endian\n
    % 
    %   Byte 2 – 3: Message counter, Little Endian\n
    % 
    %   Byte 4:     Command type. see following defines   */
    % 
    %   #define     SYSTEM_COMMAND_REPLY          0x01    //  System command, reply required
    %   #define     SYSTEM_COMMAND_NO_REPLY       0x81    //  System command, reply not required
    %                                                     /*
    %   Byte 5:     System Command. see following defines */
    % 
    %   #define     BEGIN_DOWNLOAD                0x92    //  Begin file down load
    %   #define     CONTINUE_DOWNLOAD             0x93    //  Continue file down load
    %   #define     BEGIN_UPLOAD                  0x94    //  Begin file upload
    %   #define     CONTINUE_UPLOAD               0x95    //  Continue file upload
    %   #define     BEGIN_GETFILE                 0x96    //  Begin get bytes from a file (while writing to the file)
    %   #define     CONTINUE_GETFILE              0x97    //  Continue get byte from a file (while writing to the file)
    %   #define     CLOSE_FILEHANDLE              0x98    //  Close file handle
    %   #define     LIST_FILES                    0x99    //  List files
    %   #define     CONTINUE_LIST_FILES           0x9A    //  Continue list files
    %   #define     CREATE_DIR                    0x9B    //  Create directory
    %   #define     DELETE_FILE                   0x9C    //  Delete
    %   #define     LIST_OPEN_HANDLES             0x9D    //  List handles
    %   #define     WRITEMAILBOX                  0x9E    //  Write to mailbox
    %   #define     BLUETOOTHPIN                  0x9F    //  Transfer trusted pin code to brick
    %   #define     ENTERFWUPDATE                 0xA0    //  Restart the brick in Firmware update mode
    %   #define     SETBUNDLEID                   0xA1    //  Set Bundle ID for mode 2
    %   #define     SETBUNDLESEEDID               0xA2    //  Set bundle seed ID for mode 2
    % 
    % /*
    % 
    %   Byte 6 - n: Dependent on System Command
    % 
    % 
    % 
    %   System Command Response Bytes:
    %   ,------,------,------,------,------,------,------,------,
    %   |Byte 0|Byte 1|Byte 2|Byte 3|      |      |      |Byte n|
    %   '------'------'------'------'------'------'------'------'
    % 
    %   Byte 0 – 1: Reply size, Little Endian\n
    % 
    %   Byte 2 – 3: Message counter, Little Endian\n
    % 
    %   Byte 4:     Reply type. see following defines     */
    % 
    %   #define     SYSTEM_REPLY                  0x03    //  System command reply
    %   #define     SYSTEM_REPLY_ERROR            0x05    //  System command reply error
    % /*
    %   Byte 5:     System command this is the response to
    % 
    %   Byte 6:     Reply status
    % */
    % 
    %   // SYSTEM command return codes
    %   #define     SUCCESS                       0x00
    %   #define     UNKNOWN_HANDLE                0x01
    %   #define     HANDLE_NOT_READY              0x02
    %   #define     CORRUPT_FILE                  0x03
    %   #define     NO_HANDLES_AVAILABLE          0x04
    %   #define     NO_PERMISSION                 0x05
    %   #define     ILLEGAL_PATH                  0x06
    %   #define     FILE_EXITS                    0x07
    %   #define     END_OF_FILE                   0x08
    %   #define     SIZE_ERROR                    0x09
    %   #define     UNKNOWN_ERROR                 0x0A
    %   #define     ILLEGAL_FILENAME              0x0B
    %   #define     ILLEGAL_CONNECTION            0x0C
    % 
    % /*
    %   Byte 7 - n: Response dependent on System Command
    %
    %   Direct Command Bytes:
    %   ,------,------,------,------,------,------,------,------,
    %   |Byte 0|Byte 1|Byte 2|Byte 3|Byte 4|Byte 5|      |Byte n|
    %   '------'------'------'------'------'------'------'------'
    % 
    %   Byte 0 – 1: Command size, Little Endian\n
    % 
    %   Byte 2 – 3: Message counter, Little Endian\n
    % 
    %   Byte 4:     Command type. see following defines   */
    % 
    %   #define     DIRECT_COMMAND_REPLY          0x00    //  Direct command, reply required
    %   #define     DIRECT_COMMAND_NO_REPLY       0x80    //  Direct command, reply not required
    % 
    %                                                     /*
    % 
    %   Byte 5 - 6: Number of global and local variables (compressed).
    % 
    %                Byte 6    Byte 5
    %               76543210  76543210
    %               --------  --------
    %               llllllgg  gggggggg
    % 
    %                     gg  gggggggg  Global variables [0..MAX_COMMAND_GLOBALS]
    % 
    %               llllll              Local variables  [0..MAX_COMMAND_LOCALS]
    % 
    %   Byte 7 - n: Byte codes
    %
    %                     Direct Command Response Bytes:
    %   ,------,------,------,------,------,------,------,------,
    %   |Byte 0|Byte 1|Byte 2|Byte 3|      |      |      |Byte n|
    %   '------'------'------'------'------'------'------'------'
    % 
    %   Byte 0 – 1: Reply size, Little Endian\n
    % 
    %   Byte 2 – 3: Message counter, Little Endian\n
    % 
    %   Byte 4:     Reply type. see following defines     */
    % 
    %   #define     DIRECT_REPLY                  0x02    //  Direct command reply
    %   #define     DIRECT_REPLY_ERROR            0x04    //  Direct command reply error
    % 
    %                                                     /*
    % 
    %   Byte 5 - n: Response buffer (global variable values)
    %                                                     */
    
    properties
        msg
    end

    methods
        function cmd = Command(varargin)
            % Command.cmd Create an empty command
            %
            % c = Command(OPTIONS) is an object that represents an EV3 command
            %
            % Example::
            %           c = Command();
            
            cmd.msg = uint8([]);
        end
            
        function delete(cmd)
            % Command.delete Clear command
            %
            % delete(c) clears the command
            
            cmd.msg = '';
        end
        
        function addHeaderSystem(cmd,counter)
            % Command.addHeaderSystem Add a system header with no reply
            %
            % Commad.addHeaderSystem(counter) adds a system command header
            % with no reply (0x81).
            %
            % Notes::
            % - counter is a decimal number used as a message counter
            %
            % Example::
            %            cmd.addHeaderSystem(42)
            
           cmd.msg = [cmd.msg typecast(uint16(counter), 'uint8'), uint8(129)];
        end
        
        function addHeaderSystemReply(cmd,counter)
            % Command.addHeaderSystemReply Add a system header with reply
            %
            % Command.addHeaderSystemReply(counter) adds a system command 
            % header with reply (0x01).
            %
            % Notes::
            % - counter is a decimal number used as a message counter
            %
            % Example::
            %            cmd.addHeaderSystemReply(42)
            
           cmd.msg = [cmd.msg typecast(uint16(counter), 'uint8'), uint8(1)];
        end
        
        function addHeaderDirect(cmd,counter,nGV,nLV)
            % Command.addHeaderDirect Add a direct header with no reply
            %
            % Command.addHeaderDirect(counter,GV,LV) Adds a direct command 
            % header with no reply (0x80).
            %
            % Notes::
            % - counter is a decimal number used as a message counter
            % - nGV is the number of 1 byte global variables needed.
            % - nLV is the number of 1 byte local variables needed.
            % - If you needed a global float varible then GV = 4.
            % - If you nedded a local uint16 variable then LV = 2.
            %
            % Example::
            %            cmd.addHeaderDirect(42,0,0)
            
           cmd.msg = [cmd.msg typecast(uint16(counter), 'uint8'), uint8(128), typecast(bitor(bitshift(uint16(nLV),10),uint16(nGV)), 'uint8')];
        end
        
        function addHeaderDirectReply(cmd,counter,nG,nL)
            % Command.addHeaderDirectReply Add a direct header with reply
            %
            % Command.addHeaderDirect(counter,GV,LV) Adds a direct command 
            % header with reply (0x00).
            %
            % Notes::
            % - counter is a decimal number used as a message counter
            % - nGV is the number of 1 byte global variables needed.
            % - nLV is the number of 1 byte local variables needed.
            % - If you needed a global float varible then GV = 4.
            % - If you nedded a local uint16 variable then LV = 2.
            % 
            % Example::
            %            cmd.addHeaderDirectReply(42,0,0)
            
           cmd.msg = [cmd.msg typecast( uint16(counter), 'uint8'), uint8(0), typecast( bitor(bitshift(uint16(nL),10),uint16(nG)), 'uint8')];
        end
        
        function addLength(cmd)
            % Command.addLength Add command length
            %
            % Command.addLength() adds the command message length to the 
            % start of command object message.
            % 
            % Example::
            %            cmd.addLength()
            
           cmd.msg = [typecast( uint16(length(cmd.msg)), 'uint8') cmd.msg]; 
        end
        
        function addSystemCommand(cmd,v)
            % Command.addSystemCommand Add a system command
            %
            % Command.addSystemCommand(v) adds a system command to the
            % command object.
            %
            % Notes::
            % - v is the system commands which can be found in c_com.h
            % 
            % Example::
            %            cmd.addSystemCommand(SystemCommands.BeginDownload)
            
           cmd.msg = [cmd.msg uint8(v)]; 
        end
        
        function addDirectCommand(cmd,v)
            % Command.addDirectCommand Add a direct command
            %
            % Command.addDirectCommand(v) adds a direct command to the
            % command object.
            %
            % Notes::
            % - v is the direct commands opcode can be found in ByteCodes.m
            %
            % Example::
            %            cmd.addDirectCommand(SoundSubCodes.Tone)
            
           cmd.msg = [cmd.msg uint8(v)]; 
        end
        
        function clear(cmd)
            % Command.clear Clear command
            %
            % Commad.clear clears the message
            %
            % Example::
            %           cmd.clear()

            cmd.msg = '';
        end
        
        function s = char(cmd)
            s = '';
            for i=1:length(cmd.msg)
                s = [s sprintf(' %d', cmd.msg(i))];
            end
        end
        
        function s = hex(cmd)
            s = '';
            for i=1:length(cmd.msg)
                s = [s sprintf(' %x', cmd.msg(i))];
            end
        end
        
        function display(cmd)
            % Command.display Display the command message (decimal)
            %
            % Command.display() prints the command message to the MATALB
            % command window in decimal format.
            %
            % Example::
            %           cmd.display()
            
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(cmd) );
        end 
        
        function displayHex(cmd)
            % Command.displayHex Display the command message (hex)
            %
            % Command.displayHex() prints the command message to the MATLAB
            % command window in hexadecimal format.
            %
            % Example::
            %           cmd.displayHex()
            
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( hex(cmd) );
        end 
        
        function LC0(cmd,v)
            % Command.LC0 Add a local constant 0 
            %
            % Command.LC0(v) adds a local constant 0 to the command object.
            %
            % Notes::
            % - v is the numerical value for the LC0 command
            % - Local constant 0 is defined as ((v & PRIMPAR_VALUE) | PRIMPAR_SHORT | PRIMPAR_CONST)
            % - LC0 has a range from -32 to 31 since PRIMPAR_VALUE = 3F
            % 
            % Examples::
            %           cmd.LC0(10)
            
            cmd.msg = [cmd.msg bitor(bitand(typecast(int8(v),'uint8'),uint8(Primitives.pvalue)),bitor(uint8(Primitives.pshort),uint8(Primitives.pconst)))]; 
        end
        
        function LC1(cmd,v)
            % Command.LC1 Add a local constant 1
            %
            % Command.LC1(v) adds a local constant 1 to the command object.
            %
            % Notes::
            % - v is the numerical value for the LC1 command
            % - Local constant 1 is defined as (PRIMPAR_LONG  | PRIMPAR_CONST | PRIMPAR_1_BYTE),(v & 0xFF)
            % - LC1 has a range from -127 to 128
            % 
            % Examples::
            %           cmd.LC1(100)
            
            cmd.msg = [cmd.msg bitor(bitor(uint8(Primitives.plong),uint8(Primitives.pconst)),(uint8(Primitives.p1_byte))), bitand(typecast(int8(v),'uint8'),255)];
        end
        
        function LC2(cmd,v)
            % Command.LC2 Add a local constant 2 
            %
            % Command.LC2(v) adds a local constant 2 to the command object.
            %
            % Notes::
            % - v is the numerical value for the LC2 command
            % - Local constant 2 is defined as (PRIMPAR_LONG  | PRIMPAR_CONST | PRIMPAR_2_BYTES),(v & 0xFF),((v >> 8) & 0xFF)
            % - LC2 has a range from -32768 to 32767
            % 
            % Examples::
            %           cmd.LC2(1000)
                        
            cmd.msg = [cmd.msg bitor(bitor(uint8(Primitives.plong),uint8(Primitives.pconst)),(uint8(Primitives.p2_byte))), typecast(int16(v), 'uint8')];
        end
        
        function LC4(cmd,v)
            % Command.LC4 Add a local constant 4
            %
            % Command.LC4(v) adds a local constant 4 to the command object.
            %
            % Notes::
            % - v is the numerical value for the LC4 command
            % - Local constant 4 is defined as(PRIMPAR_LONG  | PRIMPAR_CONST | PRIMPAR_4_BYTES),((ULONG)v & 0xFF),(((ULONG)v >> (ULONG)8) & 0xFF),(((ULONG)v >> (ULONG)16) & 0xFF),(((ULONG)v >> (ULONG)24) & 0xFF)
            % - LC4 has a range from –2,147,483,648 to 2,147,483,647
            % 
            % Examples::
            %           cmd.LC4(10000)
            
            cmd.msg = [cmd.msg bitor(bitor(uint8(Primitives.plong),uint8(Primitives.pconst)),(uint8(Primitives.p4_byte))), typecast(int32(v), 'uint8')];
        end
        
        function LV0(cmd,i)
            % Command.LV0 Add a local variable 0
            % 
            % Command.LV0(i) adds a local variable 0 to the command object.
            %
            % Notes::
            % - i is the number of 1 byte local variables needed.
            % - Local viarable 0 is defined as ((i & PRIMPAR_INDEX) | PRIMPAR_SHORT | PRIMPAR_VARIABEL | PRIMPAR_LOCAL)
            %
            % Example::
            %           cmd.LV0(1)
            
            cmd.msg = [cmd.msg bitor(bitor(bitor(bitand(uint8(i),Primitives.pindex),Primitives.pshort),Primitives.pvariabel),Primitives.plocal)];
        end
        
        function GV0(cmd,i)
            % Command.GV0 Add a global constant 0
            %
            % Command.GV0(i) adds a global variable 0 to the command
            % object.
            % 
            % Notes::
            % - i is the number of 1 byte global variables needed.
            % - Global constant 0 is defined as ((i & PRIMPAR_INDEX) | PRIMPAR_SHORT | PRIMPAR_VARIABEL | PRIMPAR_GLOBAL)
            %
            % Example::
            %           cmd.GV0(4)
            
            cmd.msg = [cmd.msg bitor(bitor(bitor(bitand(uint8(i),Primitives.pindex),Primitives.pshort),Primitives.pvariabel),Primitives.pglobal)];
        end
        
        function LCS(cmd)
            % Command.LCS Add a local constant string
            %
            % Command.LCS() adds a local constant string to the command
            % object.
            %
            % Notes::
            % - Local constant string is defined as (PRIMPAR_LONG | PRIMPAR_STRING)
            %
            % Example::
            %           cmd.LCS()
            
            cmd.msg = [cmd.msg bitor(Primitives.plong,Primitives.pstring)];
        end
        
        function addValue(cmd,v)
            % Command.addValue add a numerical value
            %
            % Command.addValue adds a numerical value to the command
            % object.
            %
            % Notes::
            % - v is the numerical value to be added with range -128 to 127
            %
            % Example::
            %           cmd.addValue(10)
            
            cmd.msg = [cmd.msg uint8(v)];
        end
        
        function addArray(cmd,txt)
            % Command.addArray add a numerical array
            %
            % Command.addArray(txt) adds a numerical array to the command
            % object.
            %
            % Notes::
            % - txt is the numerical array to be added
            % 
            % Example::
            %           cmd.addArray([1,2,3,4,5])
            
            for i=1:length(txt)
               cmd.addValue(txt(i)); 
            end   
        end
        
        function addString(cmd,txt)
            % Command.addString add a string
            %
            % Commad.addString(txt) adds a string to the command object.
            %
            % Notes::
            % - txt is the string to be added
            % - a null termination character is added to the end of the
            % string
            %
            % Example::
            %           cmd.addString('hello')
            
            for i=1:length(txt)
               cmd.addValue(txt(i)); 
            end
            cmd.addValue(0);    
        end
        
        function addLCSString(cmd,txt)
            % Command.addLCSString add a string with the LCS type
            %
            % Command.addLCSString(txt) adds a string to the command object
            % with the LCS command as well.
            %
            % Notes::
            % - txt is the string to be added
            % - a null termination character is added to the end of the
            % string.
            %
            % Example::
            %           cmd.addLCSString('hello')
            
            cmd.LCS;
            for i=1:length(txt)
               cmd.addValue(txt(i)); 
            end
            cmd.addValue(0);    
        end
        
        function LONGToBytes(cmd,x)
            % Command.LONGToBytes add a LONGToBytes
            %
            % Command.LONGToBytes(x) adds a LONGToBytes to the command
            % object.
            %
            % Notes::
            % - x is the value to be added
            % - LONGToBytes is defined as(UBYTE)((_x) & 0xFF),(UBYTE)((_x >> 8) & 0xFF),(UBYTE)((_x >> 16) & 0xFF),(UBYTE)((_x >> 24) & 0xFF)
            %
            % Example::
            %           cmd.LONGToBytes(1)
            
            cmd.msg = [cmd.msg typecast(uint32(x), 'uint8')];
        end
        
        function WORDToBytes(cmd,x)
            % Command.WORDToBytes add a WORDToBytes
            %
            % Command.WORDToBytes(x) adds a WORDToBytes to the command
            % object.
            %
            % Notes::
            % - x is the value to be added
            % - WORDToBytes is defined as (UBYTE)((_x) & 0xFF),(UBYTE)((_x >> 8) & 0xFF)
            %
            % Example::
            %           cmd.WORDToBytes(1)
            
            cmd.msg = [cmd.msg typecast(uint16(x), 'uint8')];
        end
        
        function BYTEToBytes(cmd,x)
            % Command.BYTEToBytes add a BYTEToBytes
            %
            % Command.BYTEToBytes(x) adds a BYTEToBytes to the command
            % object.
            %
            % Notes::
            % - x is the value to be added
            % - BYTEToBytes is defined as (UBYTE)((_x) & 0xFF)
            %
            % Example::
            %           cmd.BYTEToBytes(1)
            
            cmd.msg = [cmd.msg uint8(x)];
        end
        
        function PROGRAMHeader(cmd,VersionInfo,NumberOfObjects,GlobalBytes)
            % Command.PROGRAMHeader add a PROGRAMHeader
            %
            % Command.PROGRAMHeader(VersionInfo,NumberOfObjects,GlobalBytes) 
            % adds a PROGRAMHeader to the command object.
            %
            % Notes::
            % - VersionInfo is not used in the current byte implementation
            % (hardcoded to 1.04)
            % - NumberOfObjects is the number of objects used
            % - GlobalBytes is the number of global bytes
            % - PROGRAMHEADER is defined as 'L','E','G','O',LONGToBytes(0),WORDToBytes((UWORD)(BYTECODE_VERSION * 100.0)),WORDToBytes(NumberOfObjects),LONGToBytes(GlobalBytes)
            %
            % Example::
            %           cmd.PROGRAMHeader(0,1,0)
            
            BYTECODE_VERSION = 1.04;
            cmd.msg = [cmd.msg,uint8('L'),uint8('E'),uint8('G'),uint8('O')] ;
            cmd.LONGToBytes(0);
            cmd.WORDToBytes(BYTECODE_VERSION*100);
            cmd.WORDToBytes(NumberOfObjects);
            cmd.LONGToBytes(GlobalBytes);
        end
        
        function addFileSize(cmd)
            % Command.addFileSize Add file size
            % 
            % Command.addFileSize() adds the file size to the command
            % object.
            %
            % Notes::
            % - With bytecode compiling using "Old header", the file size is 
            % inserted at byte number 5 which corresponds to the two bytes 
            % after 'L','E','G','O'
            %
            % Example::
            %            cmd.addFileSize
            
            cmd.msg(5:6) = typecast(uint16(length(cmd.msg)),'uint8');            
        end
        
        function VMTHREADHeader(cmd,OffsetToInstructions,LocalBytes)
            % Command.VMTHREADHeader Add a VMTHREADHeader
            %
            % Command.VMTHREADHeader(OffsetToInstructions,LocalBytes)
            % adds a VMTHREADHeader to the command object.
            %
            % Notes::
            % - OffsetToInstructions is the offset to the instructions
            % - LocalBytes is the number of local bytes
            % - VMTHREADHeader is defined as LONGToBytes(OffsetToInstructions),0,0,0,0,LONGToBytes(LocalBytes)
            %
            % Example::
            %           cmd.VMTHREADHeader(0,1)
            
            cmd.LONGToBytes(OffsetToInstructions);
            cmd.msg = [cmd.msg uint8(0),uint8(0),uint8(0),uint8(0)];
            cmd.LONGToBytes(LocalBytes) ;
        end
        
        function SUBCALLHeader(cmd,OffsetToInstructions,LocalBytes)
            % Command.SUBCALLHeader Add a SUBCALLHeader
            %
            % Command.SUBCALLHeader(OffsetToInstructions,LocalBytes)
            % adds a SUBCALLHeader to the command object.
            %
            % Notes::
            % - OffsetToInstructions is the offset to the instructions
            % - LocalBytes is the number of local bytes
            % - SUBCALLHeader is defined as LONGToBytes(OffsetToInstructions),0,0,1,0,LONGToBytes(LocalBytes)
            %
            % Example::
            %           cmd.SUBCALLHeader(0,1)
            
            cmd.LONGToBytes(OffsetToInstructions);
            cmd.msg = [cmd.msg uint8(0),uint8(0),uint8(1),uint8(0)];
            cmd.LONGToBytes(LocalBytes) ;
        end
        
        function BLOCKHeader(cmd,OffsetToInstructions,OwnerObjectId,TriggerCount)
            % Command.BLOCKHeader
            %
            % Command.BLOCKHeader(OffsetToInstructions,OwnerObjectId,TriggerCount)
            % adds a BLOCKHeader to the command object.
            %
            % Notes::
            % - OffsetToInstructions is the offset to the instructions
            % - OwnerObjectId is the owner object id
            % - TriggerCount is the trigger count
            % - BLOCKHeader is defined as LONGToBytes(OffsetToInstructions),WORDToBytes(OwnerObjectId),WORDToBytes(TriggerCount),LONGToBytes(0)
            %
            % Example::
            %           cmd.BLOCKHeader(0,0,0)
            
            cmd.LONGToBytes(OffsetToInstructions);
            cmd.WORDToBytes(OwnerObjectId);
            cmd.WORDToBytes(TriggerCount)
            cmd.LONGToBytes(0);
        end  
        
        function GenerateByteCode(cmd, fileName)
            % Command.GenerateByteCode Generate byte code
            %
            % Command.GenerateByteCode(fileName) prints the byte code in
            % the command object to a file. This file can then be uplodaded
            % and executed directly on the brick.
            % 
            % Example::
            %           cmd.GenerateByteCode('tst.rbf')

            fid = fopen([fileName '.rbf'], 'w');
            fwrite(fid,cmd.msg,'uint8');
            fclose(fid);
            fprintf('Wrote %d bytes to %s.rbf\n',length(cmd.msg),fileName);
        end
        
        function opNOP(cmd)
            % Command.opNOP Add a opNOP 
            %
            % Command.opNOP() adds a opNOP opcode to the command object.
            %
            % Example::
            %           cmd.opNOP()
           
            cmd.addDirectCommand(ByteCodes.Nop);            
        end
        
        function opOBJECT_END(cmd)
            % Command.opOBJECT_END Add a opOBJECT_END
            % 
            % Command.opOBJECT_END() adds a opOBJECT_END opcode to the
            % command object. 
            %
            % Notes::
            % - opOBJECT_END
            %
            % Example::            
            %           cmd.opOBJECT_END()
           
            cmd.addDirectCommand(ByteCodes.ObjectEnd);
        end
        
        function opJR(cmd,offset)
            % Command.opJR Add a opJR
            %
            % Command.opJR(offset) adds a opJR opcode to the command
            % object.
            %
            % Notes::
            % - offset is the number of command to jump where the sign
            % indicates the jump direction
            % - opJR,LC0(offset)
            %
            % Example::
            %           cmd.opJR(-10)

            cmd.addDirectCommand(ByteCodes.Jr);
            cmd.LC0(offset);
        end
        
        function opUI_FLUSH(cmd)
            % Command.opUI_FLUSH Add a opUI_FLUSH
            %
            % Command.opUI_FLUSH() adds a opUI_FLUSH opcode to the command
            % object.
            %
            % Notes::
            % opUI_FLUSH
            %
            % Example::
            %           cmd.opUI_FLUSH()
            
            cmd.addDirectCommand(ByteCodes.UIFlush);
        end
        
        function opUI_READ_GET_VBATT(cmd,value)
            % Command.opUI_READ_GET_VBATT Add a opUI_READ_GET_VBATT
            %
            % Command.opUI_READ_GET_VBATT adds a opUI opcode with a 
            % READ_GET_VBATT subcode to the command object.
            %
            % Notes::
            % - Value is the global variable index
            % - Value is DATAF
            % - opUI_READ,LC0(1),GV0(value)
            %
            % Example::
            %           cmd.opUI_READ_GET_VBATT(0)
            
            cmd.addDirectCommand(ByteCodes.UIRead);
            cmd.LC0(UIReadSubCodes.GetVbatt);
            cmd.GV0(value);
        end
        
        function opUI_READ_GET_LBATT(cmd,value)
            % Command.opUI_READ_GET_LBATT Add a opUI_READ_GET_LBATT
            %
            % Command.opUI_READ_GET_LBATT adds a opUI opcode with a
            % READ_GET_LBATT subcode to the command object.
            %
            % Notes::
            % - value is the global variable index
            % - value is DATA8
            % - opUI_READ,LC0(18),GV0(value)
            %
            % Example::
            %           cmd.opUI_READ_GET_LBATT(0)

            cmd.addDirectCommand(ByteCodes.UIRead);
            cmd.LC0(UIReadSubCodes.GetLbatt);
            cmd.GV0(value);
        end
        
        function opUI_WRITE_PUT_STRING(cmd,txt)
            % Command.opUI_WRITE_PUT_STRING Add a opUI_WRITE_PUT_STRING
            %
            % Command.opUI_WRITE_PUT_STRING adds a opUR_WRITE opcode with a
            % WRITE_PUT_STRING subcode to the command object.
            %
            % Notes::
            % - txt is the string to be added
            % - opUI_WRITE,LC0(8),LCS,'A','B' ... ,0
            %
            % Example::
            %           cmd.opUI_WRITE_PUT_STRING('hello')
            
            cmd.addDirectCommand(ByteCodes.UIWrite);
            cmd.LC0(UIWriteSubCodes.PutString);
            cmd.addLCSString(txt);
        end
        
        function opUI_WRITE_INIT_RUN(cmd)
            % Command.opUI_WRITE_INIT_RUN Add a opUI_WRITE_INIT_RUN
            %
            % Command.opUI_WRITE_INIT_RUN adds a opUI_WRITE opcode with a
            % INIT_RUN subcode to the command object.
            %
            % Notes::
            % - opUI_WRITE,LC0(25)
            %
            % Example::
            %           cmd.opUI_WRITE_INIT_RUN()
            
            cmd.addDirectCommand(ByteCodes.UIWrite);
            cmd.LC0(UIWriteSubCodes.InitRun);
        end
        
        function opUI_WRITE_LED(cmd,pattern)
            % Command.opUI_WRITE_LED Add a opUI_WRITE_LED
            %
            % Command.opUI_WRITE_LED(pattern) adds a opUI_WRITE opcode with
            % a WRITE_LED subcode to the command object.
            %
            % Notes::
            % - pattern is the LED pattern from Device.LedBlack to
            % Device.LedOrangePulse
            % - opUI_WRITE,LC0(27),LC0(pattern)
            %
            % Example::
            %           cmd.opUI_WRITE_LED(Device.LedBlack)
            
            cmd.addDirectCommand(ByteCodes.UIWrite);
            cmd.LC0(UIWriteSubCodes.Led);
            cmd.LC0(pattern);            
        end
        
        function opUI_DRAW_UPDATE(cmd)
            % Command.opUI_DRAW_UPDATE Add a opUI_DRAW_UPDATE
            %
            % Command.opUI_DRAW_UPDATE() adds a opUI_DRAW opcode with a
            % DRAW_UPDATE subcode to the command object.
            %
            % Notes::
            % - opUI_DRAW,LC0(0)
            %
            % Examples::
            %           cmd.opUI_DRAW_UPDATE()
            
            cmd.addDirectCommand(ByteCodes.UIDraw);
            cmd.LC0(UIDrawSubCodes.Update);
        end
        
        function opUI_DRAW_CLEAN(cmd)
            % Command.opUI_DRAW_CLEAN Add a opUI_DRAW_CLEAN
            %
            % Command.opUI_DRAW_CLEAN() adds a UI_DRAW opcode with a CLEAN
            % subcode to the command object.
            %
            % Notes::
            % - opUI_DRAW,LC0(1)
            % 
            % Example::
            %           cmd.opUI_DRAW_CLEAN()
            
            cmd.addDirectCommand(ByteCodes.UIDraw);
            cmd.LC0(UIDrawSubCodes.Clean);
        end
        
        function opUI_DRAW_PIXEL(cmd,color,x,y)
            % Command.opUI_DRAW_PIXEL Add a opUI_DRAW_PIXEL
            %
            % command.opUI_DRAW_PIXEL(color,x,y) adds a UI_DRAW opcode with
            % a PIXEL subcode to the command object.
            %
            % Notes::
            % - color is the pixel color (either foregrond or background)
            % - x is the x coordinate on the LCD
            % - y is the y coordinate on the LCD
            % - opUI_DRAW,LC0(2),LC0(color),LC2(x),LC2(y)
            %
            % Example::
            %           cmd.opUI_DRAW_PIXEL(vmCodes.vmFGColor,0,0)

            cmd.addDirectCommand(ByteCodes.UIDraw);
            cmd.LC0(UIDrawSubCodes.Pixel);
            cmd.LC0(color);
            cmd.LC2(x);
            cmd.LC2(y)
        end
        
        function opUI_DRAW_LINE(cmd,color,x0,y0,x1,y1)
            % Command.opUI_DRAW_LINE Add a opUI_DRAW_LINE
            %
            % Command.opUI_DRAW_LINE(color,x0,y0,x1,y1) adds a UI_DRAW
            % opcode with a LINE subcode to the command object.
            %
            % Notes::
            % - color is the pixel color (either foregrond or background)
            % - x0,y0 is the first point coordinate of the line
            % - x1,y1 is the second point coordinate of the line
            % - opUI_DRAW,LC0(3),LC0(color),LC2(x0),LC2(y0),LC2(x1),LC2(y1)
            %
            % Example::
            %           cmd.opUI_DRAW_LINE(vmCodes.vmFGColor,0,0,1,1)
            
            cmd.addDirectCommand(ByteCodes.UIDraw);
            cmd.LC0(UIDrawSubCodes.Line);
            cmd.LC0(color);
            cmd.LC2(x0);
            cmd.LC2(y0);
            cmd.LC2(x1);
            cmd.LC2(y1);
        end
        
        function opUI_DRAW_CIRCLE(cmd,color,x0,y0,r)
            % Command.opUI_DRAW_CIRCLE Add a opUI_DRAW_CIRCLE
            %
            % Command.opUI_DRAW_CIRCLE(color,x0,y0,r) adds a UI_DRAW opcode
            % with a CIRCLE subcode to the object command.
            %
            % Notes::
            % - color is the pixel color (either foregrond or background)
            % - x is the x coordinate of the circle center
            % - y is the y coordinate of the circle center
            % - r is the radius of the circle 
            % - opUI_DRAW,LC0(4),LC0(color),LC2(x0),LC2(y0),LC2(r)
            %
            % Example::
            %           cmd.opUI_DRAW_CIRCLE(vmCodes.vmFGColor,0,0,5)
            
            cmd.addDirectCommand(ByteCodes.UIDraw);
            cmd.LC0(UIDrawSubCodes.Circle);
            cmd.LC0(color);
            cmd.LC2(x0);
            cmd.LC2(y0);
            cmd.LC2(r);
        end           
        
        function opUI_DRAW_TEXT(cmd,color,x,y,txt)
            % Command.opUI_DRAW_TEXT Add a opUI_DRAW_TEXT
            %
            % Command.opUI_DRAW_TEXT(color,x,y,txt) adds a UI_DRAW opcode
            % with a TEXT subcode to the command object.
            %
            % Notes::
            % - color is the pixel color (either foregrond or background)
            % - x is the x start coordinate of the text
            % - y is the y start coordinate of the text
            % - txt is the text to be printed
            % - opUI_DRAW,LC0(5),LC0(color),LC2(x),LC2(y),LCS,'A','B' ...,0
            %
            % Example::
            %           cmd.opUI_DRAW_TEXT(vmCodes.vmFGColor,0,0,'hello')
            
            cmd.addDirectCommand(ByteCodes.UIDraw);
            cmd.LC0(UIDrawSubCodes.Text);
            cmd.LC0(color);
            cmd.LC2(x);
            cmd.LC2(y);
            cmd.addLCSString(txt);
        end
        
        function opUI_DRAW_VALUE(cmd,color,x,y,index,figures,decimals)
            % Command.opUI_DRAW_VALUE Add a opUI_DRAW_VALUE
            %
            % Command.opUI_DRAW_VALUE(color,x,y,index,figures,decimals)
            % adds a UI_DRAW opcode with a VALUE subcode to the command
            % object.
            % Notes::
            % - color is the pixel color (either foregrond or background)
            % - x is the x start coordinate of the text
            % - y is the y start coordinate of the text
            % - index is the value to write at an index
            % - figures is the total number of figured inclusive decimal point
            % - decimals is the number of decimals
            % - opUI_DRAW,LC0(8),LC0(color),LC2(x),LC2(y),GV0(index),LC0(figures),LC0(decimals)
            %
            % Example::
            %           cmd.opUI_DRAW_VALUE(vmCodes.vmFGColor,0,0,0,3,2)
            %
            
            cmd.addDirectCommand(ByteCodes.UIDraw);
            cmd.LC0(UIDrawSubCodes.Value);
            cmd.LC0(color);
            cmd.LC2(x);
            cmd.LC2(y);
            cmd.GV0(index);
            cmd.LC0(figures);
            cmd.LC0(decimals);
        end
        
        function opUI_DRAW_FILLRECT(cmd,color,x0,y0,x1,y1)
            % Command.opUI_DRAW_FILLRECT Add a opUI_DRAW_FILLRECT
            %
            % Command.opUI_DRAW_FILLRECT(color,x0,y0,x1,y1) adds a
            % opUI_DRAW opcode with a FILLRECT subcode to the command
            % object.
            %
            % Notes::
            % - color is the pixel color (either foregrond or background)
            % - x0,y0 is the point coordinate of the top left corner
            % - x1,y1 is the point cooredinate of the bottom right corner
            % - opUI_DRAW,LC0(9),LC0(color),LC2(x0),LC2(y0),LC2(x1),LC2(y1)
            %
            % Example::
            %           cmd.opUI_DRAW_FILLRECT(vmCodes.vmFGColor,0,0,10,10)
            
            cmd.addDirectCommand(ByteCodes.UIDraw);
            cmd.LC0(UIDrawSubCodes.Fillrect);
            cmd.LC0(color);
            cmd.LC2(x0);
            cmd.LC2(y0);
            cmd.LC2(x1);
            cmd.LC2(y1);
        end
        
        function opUI_DRAW_RECT(cmd,color,x0,y0,x1,y1)
            % Command.opUI_DRAW_RECT Add a opUI_DRAW_RECT
            %
            % Command.opUI_DRAW_RECT(color,x0,y0,x1,y1) adds a
            % opUI_DRAW opcode with a RECT subcode to the command
            % object.
            %
            % Notes::
            % - color is the pixel color (either foregrond or background)
            % - x0,y0 is the point coordinate of the top left corner
            % - x1,y1 is the point cooredinate of the bottom right corner
            % - opUI_DRAW,LC0(10),LC0(color),LC2(x0),LC2(y0),LC2(x1),LC2(y1)
            %
            % Example::
            %           cmd.opUI_DRAW_RECT(vmCodes.vmFGColor,0,0,10,10)
            
            cmd.addDirectCommand(ByteCodes.UIDraw);
            cmd.LC0(UIDrawSubCodes.Rect);
            cmd.LC0(color);
            cmd.LC2(x0);
            cmd.LC2(y0);
            cmd.LC2(x1);
            cmd.LC2(y1);
        end
        
        function opUI_DRAW_INVERSERECT(cmd,x0,y0,x1,y1)
            % Command.opUI_DRAW_INVERSERECT Add a opUI_DRAW_INVERSERECT
            %
            % Command.opUI_DRAW_INVERSERECT(x0,y0,x1,y1) adds a opUI_DRAW
            % opcode with a INVERSERECT subcode to the command object.
            %
            % Notes::
            % - x0,y0 is the point coordinate of the top left corner
            % - x1,y1 is the point cooredinate of the bottom right corner
            % - opUI_DRAW,LC0(16),LC2(x0),LC2(y0),LC2(x1),LC2(y1)
            %
            % Examples::
            %           cmd.opUI_DRAW_INVERSERECT(0,0,10,10)

            cmd.addDirectCommand(ByteCodes.UIDraw);
            cmd.LC0(UIDrawSubCodes.Inverserect);
            cmd.LC2(x0);
            cmd.LC2(y0);
            cmd.LC2(x1);
            cmd.LC2(y1);
        end
        
        function opUI_DRAW_SELECT_FONT(cmd,type)
            % Command.opUI_DRAW_SELECT_FONT Add a opUI_DRAW_SELECT_FONT
            %
            % Command.opUI_DRAW_SELECT_FONT(type) adds a opUI_DRAW opcode
            % with a SELECT_FONT subcode to the command object.
            %
            % Notes::
            % - type is the font type [0..2] 
            % - opUI_DRAW,LC0(17),LC0(type)
            %
            % Examples::
            %           cmd.opUI_DRAW_SELECT_FONT(1)
            
            cmd.addDirectCommand(ByteCodes.UIDraw);
            cmd.LC0(UIDrawSubCodes.SelectFont);
            cmd.LC0(type);
        end
        
        function opUI_DRAW_TOPLINE(cmd,enable)
            % Command.opUI_DRAW_TOPLINE Add a opUI_DRAW_TOPLINE
            %
            % Command.opUI_DRAW_TOPLINE(enable) adds a opUI_DRAW opcode
            % with a TOPLINE subcode to the command object.
            %
            % Notes::
            % - enable is the top status line flag, 0 = disable, 1 = enable
            % - opUI_DRAW,LC0(18),LC0(enable)
            %
            % Example::
            %           cmd.opUI_DRAW_TOPLINE(1)
         
            cmd.addDirectCommand(ByteCodes.UIDraw);
            cmd.LC0(UIDrawSubCodes.Topline);
            cmd.LC0(enable);
        end
        
        function opUI_DRAW_FILLWINDOW(cmd, color, y0, y1)
            % Command.opUI_DRAW_FILLWINDOW Add a opUI_DRAW_FILLWINDOW
            %
            % Command.opUI_DRAW_FILLWINDOW adds a UI_DRAW opcode with a
            % FILLWINDOW subcode to the command object.
            %
            % Notes::
            % - color is the pixel color (either foregrond or background)
            % - y0 is the start Y coordinate
            % - y1 is the size of Y to fill to
            % - opUI_DRAW,LC0(19),LC0(color),LC2(y0),LC2(y1)
            %
            % Example::
            %           cmd.opUI_DRAW_FILLWINDOW(vmCodes.vmFGColor,0,10)
            
            cmd.addDirectCommand(ByteCodes.UIDraw);
            cmd.LC0(UIDrawSubCodes.Fillwindow);
            cmd.LC0(color);
            cmd.LC2(y0);
            cmd.LC2(y1);            
        end
        
        function opUI_DRAW_FILLCIRCLE(cmd,color,x0,y0,r)
            % Command.opUI_DRAW_FILLCIRCLE Add a opUI_DRAW_FILLCIRCLE
            %
            % Command.opUI_DRAW_FILLCIRCLE(color,x0,y0,r) adds a UI_DRAW
            % opcode with a FILLCIRCLE subcode to the command object.
            %
            % Notes::
            % - color is the pixel color (either foreground or background)
            % - x0 is the x coordinate of the circle center
            % - y0 is the y coordinate of the circl center
            % - r is the radius of the circle
            % - opUI_DRAW,LC0(24),LC0(color),LC2(x0),LC2(y0),LC2(r)
            %
            % Example::
            %           cmd.opUI_DRAW_FILLCIRCLE(vmCodes.vmFGColor,10,10,5)
            
            cmd.addDirectCommand(ByteCodes.UIDraw);
            cmd.LC0(UIDrawSubCodes.Fillcircle);
            cmd.LC0(color);
            cmd.LC2(x0);
            cmd.LC2(y0);
            cmd.LC2(r);
        end
        
        function opUI_DRAW_STORE(cmd,no)
            % Command.opUI_DRAW_STORE Add a opUI_DRAW_STORE
            %
            % Command.opUI_DRAW_STORE(no) adds a UI_DRAW opcode with a
            % STORE subcode to the command object. 
            %
            % Notes::
            % - no is the level number to store the UI screen
            % - opUI_DRAW,LC0(25),LC0(no)
            %
            % Example::
            %           cmd.opUI_DRAW_STORE(1)
            
            cmd.addDirectCommand(ByteCodes.UIDraw);
            cmd.LC0(UIDrawSubCodes.Store);
            cmd.LC0(no);
        end
        
        function opUI_DRAW_RESTORE(cmd,no)
            % Command.opUI_DRAW_RESTORE Add a opUI_DRAW_RESTORE
            %
            % Command.opUI_DRAW_RESTORE(no) adds a UI_DRAW opcode with a
            % RESTORE subcode to the command object. 
            %
            % Notes::
            % - no is the level number to store the UI screen (0 is saved screen before run)
            % - opUI_DRAW,LC0(26),LC0(no)
            %
            % Example::
            %           cmd.opUI_DRAW_RESTORE(1)
            
            cmd.addDirectCommand(ByteCodes.UIDraw);
            cmd.LC0(UIDrawSubCodes.Restore);
            cmd.LC0(no);
        end
        
        function opTIMER_WAIT(cmd,time,timer)
            % Command.opTIMER_WAIT Add a opTIMER_WAIT
            %
            % Command.opTIMER_WAIT(time,timer) adds a opTIMER opcode with a
            % WAIT subcode to the command object.
            %
            % Notes::
            % - time is the time is wait in ms
            % - timer is the local variable used for timing
            % - opTIMER_WAIT,LC2(time),LV0(timer)
            %
            % Example::
            %           cmd.opTIMER_WAIT(1000,0)
            
            cmd.addDirectCommand(ByteCodes.TimerWait);
            cmd.LC2(time);
            cmd.LV0(timer);
        end
        
        function opTIMER_READY(cmd,timer)
            % Command.opTIMER_READY Add a opTIMER_READY
            %
            % Command.opTIMER_READY(timer) adds a opTIMER opcode with a
            % READY subcode to the command object.
            %
            % Notes::
            % - timer is the local variable used for timing
            % - opTIMER_READY,LV0(timer)
            %
            % Example::
            %           cmd.opTIMER_READY(0)
            
            cmd.addDirectCommand(ByteCodes.TimerReady);
            cmd.LV0(timer);
        end
        
        function opTIMER_READ(cmd,time)
            % Command.opTIMER_READ Add a opTIMER_READ
            %
            % Command.opTIMER_READ(time) adds a opTIMER opcode with a READ
            % subcode to the command object.
            %
            % Notes::
            % - time is the timer to be read in ms
            % - opTIMER_READ,LV0(time)
            %
            % Example::
            %           cmd.opTIMER_READ(0)
            
            cmd.addDirectCommand(ByteCodes.TimerRead);
            cmd.LV0(time);
        end
        
        function opSOUND_BREAK(cmd)
            % Command.opSOUND_BREAK Add a opSOUND_BREAK
            %
            % Command.opSOUND_BREAK() adds a opSOUND opcode with a BREAK
            % subcode to the command object.
            %
            % Notes::
            % - opSOUND,LC0(0)
            %
            % Example::
            %           cmd.opSOUND_BREAK()
            
            cmd.addDirectCommand(ByteCodes.Sound);
            cmd.LC0(SoundSubCodes.Break);
        end
        
        function opSOUND_TONE(cmd,volume,frequency,duration)
            % Command.opSOUND_TONE Add a opSOUND_TONE
            %
            % Command.opSOUND_TONE(volume,frequency,duration) adds a
            % opSOUND opcode with a TONE subcode to the command object.
            %
            % Notes::
            % - volume is the tone volume from 0 to 100
            % - frequency is the tone frequency in Hz
            % - duration is the tone duration in ms
            % - opSOUND,LC0(1),LC1(volume),LC2(frequency),LC2(duration)
            %
            % Example::
            %           cmd.opSOUND_TONE(5,1000,500)
            
            cmd.addDirectCommand(ByteCodes.Sound);
            cmd.LC0(SoundSubCodes.Tone);
            cmd.LC1(volume);
            cmd.LC2(frequency);
            cmd.LC2(duration);
        end
        
        function opSOUND_PLAY(cmd,volume,name)
            % Command.opSOUND_PLAY Add a opSOUND_PLAY
            %
            % Command.opSOUND_PLAY(volume,name) adds a opSOUND opcode with
            % a PLAY subcode the command object.
            %
            % Example::
            %           cmd.opSOUND_PLAY(5,'test.rsf')
            %
            % Notes::
            % - volume is the volume to play the file at
            % - name is the name of the file to play
            % - opSOUND,LC0(2),LC0(volume),LCS,'A','B' ... '0'
            
            cmd.addDirectCommand(ByteCodes.Sound);
            cmd.LC0(SoundSubCodes.Play);
            cmd.LC0(volume);
            cmd.addLCSString(name);
        end
        
        function opSOUND_REPEAT(cmd,volume,name)
            % Command.opSOUND_REPEAT Add a opSOUND_REPEAT
            %
            % Command.opSOUND_REPEAT(volume,name) adds a opSOUND opcode
            % with a REPEAT subcode to the command object.
            %
            % Notes::
            % - volume is the volume to repeat the file at
            % - name is the name of the file to repeat
            % - opSOUND,LC0(2),LC0(volume),LCS,'A','B' ... '0'
            %
            % Example::
            %           cmd.opSOUND_REPEAT(1,'test.rsf')
            
            cmd.addDirectCommand(ByteCodes.Sound);
            cmd.addDirectCommand(SoundSubCodes.Repeat);
            cmd.LC0(volume);
            cmd.addLCSString(name);
        end
        
        function opSOUND_TEST(cmd,busy)
            % Command.opSOUND_TEST Add a opSOUND_TEST
            %
            % Command.opSOUND_TEST(busy) adds a opSOUND_TEST to the command
            % object. 
            %
            % Notes::
            % - busy is the returned busy flag (0 = ready, 1 = busy)
            % - opSOUND_TEST,LV0(busy)
            %
            % Example::
            %           cmd.opSOUND_TEST(0)
            
            cmd.addDirectCommand(ByteCodes.SoundTest);
            cmd.LV0(busy);
        end
        
        function opSOUND_READY(cmd)
            % Commad.opSOUND_READY Add a opSOUND_READY
            %
            % Command.opSOUND_READY() adds a opSOUND_READY opcode to the 
            % command object.
            %
            % Example::
            %           cmd.opSOUND_READY()
            %
            % Notes::
            % - opSOUND_READY
            
            cmd.addDirectCommand(ByteCodes.SoundReady);           
        end
        
        function opINPUT_DEVICE_LIST(cmd,length,array,changed)
            % Command.opINPUT_DEVICE_LIST Add a opINPUT_DEVICE_LIST
            %
            % Command.opINPUT_DEVICE_LIST(length,array,changed) adds a
            % opINPUT_DEVICE_LIST opcode to the command object.
            %
            % Notes::
            % - length is maximum number of device types (norm 32)
            % - array is the returned device array list (DATA8 but can be 32 long so DATA32)
            % - changed is the returned device changed status (DATA8)
            % - opINPUT_DEVICE_LIST,LC0(length),GV0(array),GV0(changed)
            %
            % Example::
            %           cmd.opINPUT_DEVICE_LIST(32,0,1)
            
            cmd.addDirectCommand(ByteCodes.InputDeviceList);
            cmd.LC0(length);
            cmd.GV0(array);
            cmd.GV0(changed);
        end
        
        function opINPUT_DEVICE_GET_TYPEMODE(cmd,layer,no,type,mode)
            % Command.opINPUT_DEVICE_GET_TYPEMODE Add a opINPUT_DEVICE_GET_TYPEMODE
            %
            % Command.opINPUT_DEVICE_GET_TYPEMODE(layer,no,type,mode) adds
            % a opINPUT_DEVICE opcode with a GET_TYPEMODE subcode to the
            % command object.
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            % - NO is the output port number from [0..3] or sensor port number minus 1
            % - type is the returned type (DATA8)
            % - mode is the returned mode (DATA8)
            % - opINPUT_DEVICE,LC0(5),LC0(layer),LC0(no),GV0(type),GV0(mode)
            %
            % Example::
            %           cms.opINPUT_DEVICE_GET_TYPEMODE(0,Device.Port1,0,1)
            
            cmd.addDirectCommand(ByteCodes.InputDevice);
            cmd.LC0(InputDeviceSubCodes.GetTypeMode);
            cmd.LC0(layer);
            cmd.LC0(no);
            cmd.GV0(type);
            cmd.GV0(mode);
        end
        
        function opINPUT_DEVICE_GET_SYMBOL(cmd,layer,no,length,destination)
            % Command.opINPUT_DEVICE_GET_SYMBOL Add a opINPUT_DEVICE_GET_SYMBOL
            %
            % Command.opINPUT_DEVICE_GET_SYMBOL(layer,no,length,destination)
            % adds a opINPUT_DEVICE opcode with a GET_SYMBOL subcode to the
            % command object.
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            % - NO is the output port number from [0..3] or sensor port number minus 1
            % - length maximal length of string returned (-1 no check)
            % - destination string variable or handle to string
            % - opINPUT_DEVICE,LC0(6),LC0(layer),LC0(no),LC0(length),GV0(destination)
            %
            % Example::
            %           cmd.opINPUT_DEVICE_GET_SYMBOL(0,Device,Port1,5,0)
            
            cmd.addDirectCommand(ByteCodes.InputDevice);
            cmd.LC0(InputDeviceSubCodes.GetSymbol);
            cmd.LC0(layer);
            cmd.LC0(no);
            cmd.LC0(length);
            cmd.GV0(destination);
        end
        
        function opINPUT_DEVICE_CLR_ALL(cmd,layer)
            % Command.opINPUT_DEVICE_CLR_ALL Add a opINPUT_DEVICE_CLR_ALL
            %
            % Command.opINPUT_DEVICE_CLR_ALL(layer) adds a opINPUT_DEVICE
            % opcode with a CLR_ALL subcode to the command object.
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            %
            % Example::
            %           cmd.opINPUT_DEVICE_CLR_ALL(0)
            
            cmd.addDirectCommand(ByteCodes.InputDevice);
            cmd.LC0(InputDeviceSubCodes.ClrAll);
            cmd.LC0(layer);
        end 
        
        function opINPUT_DEVICE_GET_NAME(cmd,layer,no,length,destination)
            % Command.opINPUT_DEVICE_GET_NAME Add a opINPUT_DEVICE_GET_NAME
            %
            % Command.opINPUT_DEVICE_GET_NAME(layer,no,length,destination)
            % adds a opINPUT_DEVICE opcode with a GET_NAME subcode to the
            % command object.
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            % - NO is the output port number from [0..3] or sensor port number minus 1
            % - length maximal length of string returned (-1 no check)
            % - destination string variable or handle to string
            % - opINPUT_DEVICE,LC0(21),LC0(layer),LC0(no),LC0(length),GV0(destination)
            %
            % Example::
            %           cmd.opINPUT_DEVICE_GET_NAME(0,Device,Port1,12,0)
            
            cmd.addDirectCommand(ByteCodes.InputDevice);
            cmd.LC0(InputDeviceSubCodes.GetName);
            cmd.LC0(layer);
            cmd.LC0(no);
            cmd.LC0(length);
            cmd.GV0(destination);
        end 

        function opINPUT_READ(cmd,layer,no,type,mode,pct)
            % Command.opINPUT_READ Add a opINPUT_READ
            %
            % Command.opINPUT_READ(layer,no,type,mode,pct) adds a opINPUT_READ 
            % opcode to the command object.
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            % - NO is the output port number from [0..3] or sensor port number minus 1
            % - type is the sensor type from types.html (0 = don't change)
            % - mode is the sensor mode from types.html [0..7] (-1 = don't change)
            % - PCT is the percent value from the device (DATA8)
            % - The device type is already set when the sensor is
            % connected, you should only need to specify the mode
            % - opINPUT_READ,LC0(layer),LC0(no),LC0(type),LC0(mode),GV0(pct)
            %
            % Example::
            %           cmd.opINPUT_READ(0,Device.Port1,Device.Ultraonsic,Device.USDistCM,0)
            
            cmd.addDirectCommand(ByteCodes.InputReadSI);
            cmd.LC0(layer);
            cmd.LC0(no);
            cmd.LC0(type);
            cmd.LC0(mode);
            cmd.GV0(pct);
        end
        
        function opINPUT_READY(cmd,layer,no)
            % Command.opINPUT_READY Add a opINPUT_READY
            %
            % Command.opINPUT_READY(layer,no) adds a opINPUT_READY opcode
            % to the command object. 
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            % - NO is the output port number from [0..3] or sensor port number minus 1
            % - opINPUT_READY,LC0(layer),LC0(no)
            %
            % Example::
            %           cmd.opINPUT_READY(0,Device.Port1)
            
            cmd.addDirectCommand(ByteCodes.InputReady);
            cmd.LC0(layer);
            cmd.LC0(no);
        end
        
        function opINPUT_READSI(cmd,layer,no,type,mode,si)
            % Command.opINPUT_READSI Add a opINPUT_READSI
            %
            % Command.opINPUT_READSI(layer,no,type,mode,si) adds a opINPUT_READSI  
            % opcode to the command object.
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            % - NO is the output port number from [0..3] or sensor port number minus 1
            % - type is the sensor type from types.html (0 = don't change)
            % - mode is the sensor mode from types.html [0..7] (-1 = don't change)
            % - SI unit value from the device (DATAF)
            % - The device type is already set when the sensor is
            % connected, you should only need to specify the mode
            % - opINPUT_READSI,LC0(layer),LC0(no),LC0(type),LC0(mode),GV0(si)
            %
            % Example::
            %           cmd.opINPUT_READ(0,Device.Port1,Device.Ultraonsic,Device.USDistCM,0)

            cmd.addDirectCommand(ByteCodes.InputReadSI);
            cmd.LC0(layer);
            cmd.LC0(no);
            cmd.LC0(type);
            cmd.LC0(mode);
            cmd.GV0(si);
        end
        
        function opOUTPUT_SET_TYPE(cmd,layer,nos,type)
            % Command.opOUTPUT_SET_TYPE Add a opOUTPUT_SET_TYPE
            %
            % Command.opOUTPUT_SET_TYPE(layer,nos,type) adds a opOUTPUT_SET_TYPE
            % opcode to the command object.
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            % - NOS is a bit field representing output 1 to 4 (0x01, 0x02, 0x04, 0x08)
            % - type is the output device type from 0 to 255 (see types.html)
            % - opOUTPUT_SET_TYPE,LC0(layer),LC0(nos),LC0(type)
            %
            % Example::
            %           cmd.opOUTPUT_SET_TYPE(0,Device.MotorA,0)
            
            cmd.addDirectCommand(ByteCodes.OutputSetType);
            cmd.LC0(layer);
            cmd.LC0(nos);
            cmd.LC0(type);
        end
        
        function opOUTPUT_RESET(cmd,layer,nos)
            % Command.opOUTPUT_RESET Add a opOUTPUT_RESET
            %
            % Command.opOUTPUT_RESET(layer,nos) adds a opOUTPUT_RESET opcode 
            % to the command object.
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            % - NOS is a bit field representing output 1 to 4 (0x01, 0x02, 0x04, 0x08)
            % - opOUTPUT_RESET,LC0(layer),LC0(nos)
            %
            % Example::
            %           cmd.opOUTPUT_RESET(0,Device.MotorA)
                        
            cmd.addDirectCommand(ByteCodes.OutputReset);
            cmd.LC0(layer);
            cmd.LC0(nos);
        end
        
        function opOUTPUT_STOP(cmd,layer,nos,brake)
            % Command.opOUTPUT_STOP Add a opOUTPUT_STOP
            %
            % Command.opOUTPUT_STOP(layer,nos,brake) adds a opOUTPUT_STOP opcode 
            % to the command object.
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            % - NOS is a bit field representing output 1 to 4 (0x01, 0x02, 0x04, 0x08)
            % - opOUTPUT_STOP,LC0(layer),LC0(nos),LC0(brake)
            %
            % Example::
            %           cmd.opOUTPUT_STOP(0,Device.MotorA,Device.Coast)
            
            cmd.addDirectCommand(ByteCodes.OutputStop);
            cmd.LC0(layer);
            cmd.LC0(nos);
            cmd.LC0(brake);
        end
        
        function opOUTPUT_SPEED(cmd,layer,nos,speed)
            % Command.opOUTPUT_SPEED Add a opOUTPUT_SPEED
            %
            % Command.opOUTPUT_SPEED(layer,nos,speed) adds a opOUTPUT_SPEED opcode 
            % to the command object.
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            % - NOS is a bit field representing output 1 to 4 (0x01, 0x02, 0x04, 0x08)
            % - speed is the output speed with [+-0..100%] range
            % - opOUTPUT_POWER,LC0(layer),LC0(nos),LC1(speed)
            %
            % Example::
            %           cmd.opOUTPUT_SPEED(0,Device.MotorA,10)
           
            cmd.addDirectCommand(ByteCodes.OutputSpeed);
            cmd.LC0(layer);
            cmd.LC0(nos);
            cmd.LC1(speed);
        end
        
        function opOUTPUT_POWER(cmd,layer,nos,power)
            % Command.opOUTPUT_POWER Add a opOUTPUT_POWER
            %
            % Command.opOUTPUT_POWER(layer,nos,speed) adds a opOUTPUT_POWER opcode 
            % to the command object.
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            % - NOS is a bit field representing output 1 to 4 (0x01, 0x02, 0x04, 0x08)
            % - power is the output power with [+-0..100%] range
            % - opOUTPUT_POWER,LC0(layer),LC0(nos),LC1(power)
            %
            % Example::
            %           cmd.opOUTPUT_POWER(0,Device.MotorA,10)
            
            cmd.addDirectCommand(ByteCodes.OutputPower);
            cmd.LC0(layer);
            cmd.LC0(nos);
            cmd.LC1(power);
        end
        
        function opOUTPUT_START(cmd,layer,nos)
            % Command.opOUTPUT_START Add a opOUTPUT_START
            %
            % Command.opOUTPUT_START(layer,nos) adds a opOUTPUT_START opcode 
            % to the command object.
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            % - NOS is a bit field representing output 1 to 4 (0x01, 0x02, 0x04, 0x08)
            % - opOUTPUT_START,LC0(layer),LC0(nos)
            %
            % Example::
            %           cmd.opOUTPUT_START(0,Device.MotorA)
            
            cmd.addDirectCommand(ByteCodes.OutputStart);
            cmd.LC0(layer);
            cmd.LC0(nos);
        end
        
        function opOUTPUT_POLARITY(cmd,layer,nos,pol)
            % Command.opOUTPUT_POLARITY Add a opOUTPUT_POLARITY
            %
            % Command.opOUTPUT_POLARITY(layer,nos,pol) adds a opOUTPUT_POLARITY opcode 
            % to the command object.
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            % - NOS is a bit field representing output 1 to 4 (0x01, 0x02, 0x04, 0x08)
            % - pol is the polarity [-1,0,1], -1 makes the motor run
            % backward, 1 makes the motor run forward, 0 makes the motor
            % run the opposite direction
            % - opOUTPUT_POLARITY,LC0(layer),LC0(nos),LC0(pol)
            %
            % Example::
            %           cmd.opOUTPUT_POLARITY(0,Device.MotorA,0)
            
            cmd.addDirectCommand(ByteCodes.OutputPolarity);
            cmd.LC0(layer);
            cmd.LC0(nos);
            cmd.LC0(pol);
        end
        
        function opOUTPUT_READ(cmd,layer,nos,speed,tacho)
            % Command.opOUTPUT_READ Add a opOUTPUT_READ
            %
            % Command.opOUTPUT_READ(layer,nos,speed,tacho) adds a opOUTPUT_READ opcode 
            % to the command object.
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            % - NOS is a bit field representing output 1 to 4 (0x01, 0x02, 0x04, 0x08)
            % - speed is the returned speed with [+-0..100%] range (DATA8)
            % - tacho is the number of tacho pulses (DATA32)
            % - opOUTPUT_READ,LC0(layer),LC0(nos),GV0(speed),GV0(tacho)
            %
            % Example::
            %           cmd.opOUTPUT_READ(0,Device.MotorA,0,1)
            
            cmd.addDirectCommand(ByteCodes.OutputRead);
            cmd.LC0(layer);
            cmd.LC0(nos);
            cmd.GV0(speed);
            cmd.GV0(tacho);
        end 
        
        function opOUTPUT_TEST(cmd,layer,nos,value)
            % Command.opOUTPUT_TEST Add a opOUTPUT_TEST
            %
            % Command.opOUTPUT_READ(layer,nos) adds a opOUTPUT_TEST opcode 
            % to the command object.
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            % - NOS is a bit field representing output 1 to 4 (0x01, 0x02, 0x04, 0x08)
            % - value is the global variable index for the return state where (0 = ready) or (1 = busy)
            % - value is DATA8
            % - opOUTPUT_TEST,LC0(layer),LC0(nos),GV0(value)
            %
            % Example::
            %           cmd.opOUTPUT_TEST(0,Device.MotorA,0)
            
            cmd.addDirectCommand(ByteCodes.OutputTest);
            cmd.LC0(layer);
            cmd.LC0(nos);
            cmd.GV0(value);
        end
        
        function opOUTPUT_READY(cmd,layer,nos)
            % Command.opOUTPUT_READY Add a opOUTPUT_READY
            %
            % Command.opOUTPUT_READY(layer,nos) adds a opOUTPUT_READY opcode with 
            % to the command object.
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            % - NOS is a bit field representing output 1 to 4 (0x01, 0x02, 0x04, 0x08)
            % - opOUTPUT_READY,LC0(layer),LC0(nos)
            %
            % Example::
            %           cmd.opOUTPUT_READY(0,Device.MotorA)
            
            cmd.addDirectCommand(ByteCodes.OutputReady);
            cmd.LC0(layer);
            cmd.LC0(nos);
        end 
        
        function opOUTPUT_STEP_POWER(cmd,layer,nos,power,step1,step2,step3,brake)
            % Command.opOUTPUT_STEP_POWER Add a opOUTPUT_STEP_POWER
            %
            % Command.opOUTPUT_STEP_POWER(layer,nos,power,step1,step2,step3,brake) 
            % adds a opOUTPUT_STEP_POWER opcode to the command object. 
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            % - NOS is a bit field representing output 1 to 4 (0x01, 0x02, 0x04, 0x08)
            % - power is the output power with [+-0..100%] range
            % - step1 is the steps used to ramp up
            % - step2 is the steps used for constant power
            % - step3 is the steps used for ramp down
            % - brake is [0..1] (0=Coast,  1=Brake)
            % - opOUTPUT_STEP_POWER,LC0(layer),LC0(nos),LC1(power),LC4(step1),LC4(step2),LC4(step3),LC0(brake)
            %
            % Example::
            %           cmd.opOUTPUT_STEP_POWER(0,Device.MotorA,50,50,360,50,Device.Coast)
            
            cmd.addDirectCommand(ByteCodes.OutputStepPower);
            cmd.LC0(layer);
            cmd.LC0(nos);
            cmd.LC1(power);
            cmd.LC4(step1);
            cmd.LC4(step2);
            cmd.LC4(step3);
            cmd.LC0(brake);
        end
        
        function opOUTPUT_STEP_SPEED(cmd,layer,nos,speed,step1,step2,step3,brake)
            % Command.opOUTPUT_STEP_SPEED Add a opOUTPUT_STEP_SPEED
            % 
            % Command.opOUTPUT_STEP_SPEED(layer,nos,speed,step1,step2,step3,brake) 
            % adds a opOUTPUT_STEP_SPEED opcode to the command object.
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            % - NOS is a bit field representing output 1 to 4 (0x01, 0x02, 0x04, 0x08)
            % - speed is the output speed with [+-0..100%] range
            % - step1 is the steps used to ramp up
            % - step2 is the steps used for constant speed
            % - step3 is the steps used for ramp down
            % - brake is [0..1] (0=Coast,  1=Brake)
            % - opOUTPUT_STEP_POWER,LC0(layer),LC0(nos),LC1(speed),LC4(step1),LC4(step2),LC4(step3),LC0(brake)
            %
            % Example::
            %           cmd.opOUTPUT_STEP_SPEED(0,Device.MotorA,50,50,360,50,Device.Coast)
            
            cmd.addDirectCommand(ByteCodes.OutputStepSpeed);
            cmd.LC0(layer);
            cmd.LC0(nos);
            cmd.LC1(speed);
            cmd.LC4(step1);
            cmd.LC4(step2);
            cmd.LC4(step3);
            cmd.LC0(brake);
        end
        
        function opOUTPUT_TIME_SPEED(cmd,layer,nos,speed,step1,step2,step3,brake)
            % Command.opOUTPUT_TIME_SPEED Add a opOUTPUT_TIME_SPEED
            %
            % Command.opOUTPUT_TIME_SPEED(layer,nos,speed,step1,step2,step3,brake) 
            % adds a opOUTPUT_TIME_SPEED opcode to the command object. 
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            % - NOS is a bit field representing output 1 to 4 (0x01, 0x02, 0x04, 0x08)
            % - speed is the output speed with [+-0..100%] range
            % - step1 is the time used for ramp up
            % - step2 is the time used for constant speed
            % - step3 is the time used for ramp down
            % - brake is [0..1] (0=Coast,  1=Brake)
            % - opOUTPUT_TIME_SPEED,LC0(layer),LC0(nos),LC1(speed),LC4(step1),LC4(step2),LC4(step3),LC0(brake)
            %
            % Example::
            %           cmd.opOUTPUT_TIME_SPEED(0,Device.MotorA,50,100,1000,100,Device.Coast)
            
            cmd.addDirectCommand(ByteCodes.OutputTimeSpeed);
            cmd.LC0(layer);
            cmd.LC0(nos);
            cmd.LC1(speed);
            cmd.LC4(step1);
            cmd.LC4(step2);
            cmd.LC4(step3);
            cmd.LC0(brake);
        end
        
        function opOUTPUT_STEP_SYNC(cmd,layer,nos,speed,turn,step,brake)
            % Command.opOUTPUT_STEP_SYNC Add a opOUTPUT_STEP_SYNC
            %
            % Command.opOUTPUT_STEP_SYNC(layer,nos,speed,turn,step,brake) 
            % adds a opOUTPUT_STEP_SYNC opcode to the command object. 
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            % - NOS is a bit field representing output 1 to 4 (0x01, 0x02, 0x04, 0x08)
            % - speed is the output speed with [+-0..100%] range
            % - turn is the turn ratop with [+-0..200] range
            % - step is the number of tacho pulses
            % - brake is [0..1] (0=Coast,  1=Brake)
            % - opOUTPUT_STEP_SYNC,LC0(layer),LC0(nos),LC1(speed),LC2(turn),LC4(step),LC0(brake)
            %
            % Example::
            %           cmd.opOUTPUT_STEP_SYNC(0,Device.MotorA,50,30,100,Device.Coast)
            
            cmd.addDirectCommand(ByteCodes.OutputStepSync);
            cmd.LC0(layer);
            cmd.LC0(nos);
            cmd.LC1(speed);
            cmd.LC2(turn);
            cmd.LC4(step);
            cmd.LC0(brake);
        end
        
        function opOUTPUT_TIME_SYNC(cmd,layer,nos,speed,turn,time,brake)
            % Command.opOUTPUT_TIME_SYNC Add a opOUTPUT_TIME_SYNC
            %
            % Command.opOUTPUT_TIME_SYNC(layer,nos,speed,turn,time,brake) 
            % adds a opOUTPUT_TIME_SYNC opcode to the command object. 
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            % - NOS is a bit field representing output 1 to 4 (0x01, 0x02, 0x04, 0x08)
            % - speed is the output speed with [+-0..100%] range
            % - turn is the turn ratop with [+-0..200] range
            % - time is the time in ms
            % - brake is [0..1] (0=Coast,  1=Brake)
            % - opOUTPUT_STEP_SYNC,LC0(layer),LC0(nos),LC1(speed),LC2(turn),LC4(time),LC0(brake)
            %
            % Example::
            %           cmd.opOUTPUT_TIME_SYNC(0,Device.MotorA,50,30,1000,Device.Coast)
            
            cmd.addDirectCommand(ByteCodes.OutputTimeSync);
            cmd.LC0(layer);
            cmd.LC0(nos);
            cmd.LC1(speed);
            cmd.LC2(turn);
            cmd.LC4(time);
            cmd.LC0(brake);
        end
        
        function opOUTPUT_CLR_COUNT(cmd,layer,nos)
            % Command.opOUTPUT_CLR_COUNT Add a opOUTPUT_CLR_COUNT
            %
            % Command.opOUTPUT_TIME_SYNC(layer,nos) adds a opOUTPUT_CLR_COUNT opcode 
            % to the command object. 
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            % - NOS is a bit field representing output 1 to 4 (0x01, 0x02, 0x04, 0x08)
            % - opOUTPUT_CLR_COUNT,LC0(layer),LC0(nos)
            %
            % Example::
            %           cmd.opOUTPUT_CLR_COUNT(0,Device.MotorA)

            cmd.addDirectCommand(ByteCodes.OutputClrCount);
            cmd.LC0(layer);
            cmd.LC0(nos);
        end
        
        function opOUTPUT_GET_COUNT(cmd,layer,nos,tacho)
            % Command.opOUTPUT_GET_COUNT Add a opOUTPUT_GET_COUNT
            %
            % Command.opOUTPUT_GET_COUNT(layer,nos,tacho) adds a opOUTPUT_GET_COUNT
            % to the command object. 
            %
            % Notes::
            % - layer is the usb chain layer (usually 0)
            % - NOS is a bit field representing output 1 to 4 (0x01, 0x02, 0x04, 0x08)
            % - tacho is the returned tacho pulse count (DATA32)
            % - opOUTPUT_GET_COUNT,LC0(layer),LC0(NOS),GV0(tacho)
            %
            % Example::
            %           cmd.opOUTPUT_GET_COUNT(0,Device.MotorA,4)
 
            cmd.addDirectCommand(ByteCodes.OutputGetCount);
            cmd.LC0(layer);
            cmd.LC0(nos);
            cmd.GV0(tacho);
        end
        
        function opCOMGET_GET_BRICKNAME(cmd,length,name)
            % Command.opCOMGET_GET_BRICKNAME Add a opCOMGET_GET_BRICKNAME
            %
            % Command.opCOMGET_GET_BRICKNAME(length,name) adds a opCOMGET
            % with a GET_BRICKNAME subcode to the command object. 
            %
            % Notes::
            % - length is the max length of the returned string
            % - name is first character in the returned brick name (DATA8)
            % - opCOMGET_GET_BRICKNAME,LC0(13),LC0(length),GV0(name)
            %
            % Example::
            %           cmd.opCOMGET_GET_BRICKNAME(10,0)
 
            cmd.addDirectCommand(ByteCodes.COMGet);
            cmd.LC0(COMGetSubCodes.GetBrickName);
            cmd.LC0(length);
            cmd.GV0(name);
        end 
        
        function opCOMSET_SET_BRICKNAME(cmd,name)
            % Command.opCOMSET_SET_BRICKNAME Add a opCOMSET_SET_BRICKNAME
            %
            % Command.opCOMSET_SET_BRICKNAME(name) adds a opCOMSET
            % with a SET_BRICKNAME subcode to the command object. 
            %
            % Notes::
            % - name is the brick name to be set
            % - opCOMSET_SET_BRICKNAME,LC0(13),LCS,'E','V','3',0
            %
            % Example::
            %           cmd.opCOMSET_SET_BRICKNAME('EV3')
 
            cmd.addDirectCommand(ByteCodes.COMSet);
            cmd.LC0(COMSetSubCodes.SetBrickName);
            cmd.addLCSString(name);  
        end
        
        function opMAILBOX_WRITE(cmd,brickname,boxname,type,msg)
            % Command.opMAILBOX_WRITE Add a opMAILBOX_WRITE
            %
            % Command.opMAILBOX_WRITE(brickname,boxname,type,msg) adds a
            % opMAILBOX_WRITE to the command object.
            %
            % Notes::
            % - brickname is the name of remote device
            % - boxname is the name of the receiving mailbox
            % - type is the data type of the values being sent where DATA_8
            % (0) is logic, DATA_F (3) is numeric and DATA_S (4) is text
            % - msg is the message to be sent
            % - opMAILBOX_WRITE,LCS,'T','5','0','0',0,0,LCS,'a','b','c',LC0(0),LC0(1),LC0(1)
            %
            % Example::
            %           cmd.opMAILBOX_WRITE('T500','abc','logical',1)
            
            cmd.addDirectCommand(ByteCodes.MailboxWrite);
            cmd.addLCSString(brickname);
            % hardware transportation media (not used)
            cmd.LC0(0);
            cmd.addLCSString(boxname);
            % type
            switch type
                case 'logic'
                    cmd.LC0(0);
                    cmd.LC0(1);
                    cmd.LC0(msg);
                case 'numeric'
                    cmd.LC0(3);
                    cmd.LC0(1);
                    cmd.LC4(typecast(single(msg),'int32'));
                case 'text'
                    cmd.LC0(4);
                    cmd.LC0(1);
                    cmd.addLCSString(msg);
                 otherwise
                    fprintf('Error! Type must be ''text'', ''numeric'' or ''logic''.\n');
            end
        end
    
        function BEGIN_DOWNLOAD(cmd,filelength,filename)
            % Command.BEGIN_DOWNLOAD Add a BEGIN_DOWNLOAD 
            %
            % Command.BEGIN_DOWNLOAD(filelength,filename) adds a BEGIN_DOWNLOAD
            % system command to the command object. Download is from PC to
            % brick.
            %
            % Notes::
            % - filelength is the length of the file in bytes
            % - filename is the PC file for download to the brick relative
            % to '/home/root/lms2012/sys' directory
            % - ss (BEGIN_DOWNLOAD) llllllll (filelength) nn.. (filename)
            %
            % Example::
            %           cmd.BEGIN_DOWNLOAD(60,'../apps/tst/tst.rbf')
            
            cmd.addSystemCommand(SystemCommands.BeginDownload);
            cmd.addArray(typecast(uint32(filelength),'uint8'));
            cmd.addString(filename);            
        end
            
        function CONTINUE_DOWNLOAD(cmd,handle,payload)
            % Command.CONTINUE_DOWNLOAD Add a CONTINUE_DOWNLOAD
            %
            % Command.CONTINUE_DOWNLOAD(handle,payload) adds a
            % CONTINUE_DOWNLOAD system command to the command object.
            % Download is from PC to brick.
            %
            % Notes::
            % - handle is the handle returned from BEGIN_DOWNLOAD
            % - payload is the byte data to be downloaded to the brick
            % - ss (CONTINUE_DOWNLOAD) hh (handle from BEGIN_DOWNLOAD REPLY) pppppppp ... (payload)
            %
            % Example::
            %           cmd.CONTINUE_DOWNLOAD(0,[10 20 30 .... ])
            
            cmd.addSystemCommand(SystemCommands.ContinueDownload);
            cmd.addValue(handle);
            cmd.addArray(payload);
        end
        
        function BEGIN_UPLOAD(cmd,filelength,filename)
            % Command.BEGIN_UPLOAD Add a BEGIN_UPLOAD
            %
            % Command.BEGIN_UPLOAD(filelength,filename) adds a BEGIN_UPLOAD
            % system command to the command object. Upload is from brick to
            % PC.
            %
            % Notes::
            % - filelength is the max buffer size used for file upload
            % - filename is the file to be uploaded from brick to PC
            % relative to '/home/root/lms2012/sys' directory
            % - ss (BEGIN_UPLOAD) llll (filelength/bytes to read) nn..(filename including path)
            %
            % Example::
            %            cmd.BEGIN_UPLOAD(100,'../apps/tst/tst.rbf')
           
           cmd.addSystemCommand(SystemCommands.BeginUpload);
           cmd.addArray(typecast(uint16(filelength),'uint8'));
           cmd.addString(filename);
        end
        
        function CONTINUE_UPLOAD(cmd,handle,maxlength)
            % Command.CONTINUE_UPLOAD Add a CONTINUE_UPLOAD
            %
            % Command.CONTINUE_UPLOAD(handle,maxlength) adds a
            % CONTINUE_UPLOAD system command to the commanc object. Upload
            % is from brick to PC.
            %
            % Notes::
            % - handle is the handle returned from BEGIN_UPLOAD
            % - maxlength is the max buffer size used for file upload
            % - ss (CONTINUE_UPLOAD) hh (handle from BEGIN_UPLOAD REPLY) llll (maxlength/bytes to read)
            % Example::
            %           cmd.CONTINUE_UPLOAD(0,100)
            
            cmd.addSystemCommand(SystemCommands.ContinueUpload);
            cmd.addValue(handle);
            cmd.addArray(typecast(uint16(maxlength),'uint8'));
        end
        
        function LIST_FILES(cmd,maxlength,pathname)
            % Command.LIST_FILES Add a LIST_FILES
            %
            % Command.LIST_FILES(maxlength,pathname) adds a LIST_FILES
            % system command to the command object.
            %
            % Notes::
            % - maxlength is the max buffer size used for file listing
            % - pathname is the absolute pathname use for file listing
            % - ss (LIST_FILES) llll (maxlength/max bytes to read) nn..(pathname)
            %
            % Example::
            %           cmd.LIST_FILES(100,'/home/root/lms2012/')
            
            cmd.addSystemCommand(SystemCommands.ListFiles);
            cmd.addArray(typecast(uint16(maxlength),'uint8'));
            cmd.addString(pathname)
        end
        
        function CONTINUE_LIST_FILES(cmd,handle,maxlength)
            % Command.CONTINUE_LIST_FILES Add a CONTINUE_LIST_FILES
            %
            % Command.CONTINUE_LIST_FILES(handle,maxlength) adds a
            % CONTINUE_LIST_FILES system command to the command object.
            %
            % Notes::
            % - handle is the handle returned by LIST_FILES
            % - maxlength is the max buffer size used for file listing
            % - ss (CONTINUE_LIST_FILES) hh (handle) llll (maxlength/max bytes to read)
            
            %
            % Example::
            %           cmd.CONTINUE_LIST_FILES(0,100)
            
            cmd.addSystemCommand(SystemCommands.ContinueListFiles);
            cmd.addValue(handle);
            cmd.addArray(typecast(uint16(maxlength),'uint8'));
        end
        
        function CREATE_DIR(cmd,pathname)
            % Command.CREATE_DIR Add a CREATE_DIR
            %
            % Command.CREATE_DIR(pathname) adds a CREATE_DIR system command
            % to the command object.
            %
            % Notes::
            % - pathname is the absolute path for directory creation.
            % - ss (CREATE_DIR) pp (pathname)
            
            %
            % Example::
            %           cmd.CREATE_DIR('/home/root/lms2012/newdir')
            
            cmd.addSystemCommand(SystemCommands.CreateDir);
            cmd.addString(pathname);
        end
        
        function DELETE_FILE(cmd,pathname)
            % Command.DELETE_FILE Add a DELETE_FILE
            %
            % Command.DELETE_FILE(pathname) adds a DELETE_FILE system
            % command to the command object.
            %
            % Notes::
            % - pathname is the absolute file path for deleteion
            % - ss (DELETE_FILE) pp (pathname)
            %
            % Example::
            %           cmd.DELETE_FILE('/home/root/lms2012/newdir')
            
            cmd.addSystemCommand(SystemCommands.DeleteFile);
            cmd.addString(pathname);
        end
        
        function WRITEMAILBOX(cmd,title,type,msg)
            % Command.WRITEMAILBOX Add a WRITEMAILBOX
            %
            % Command.WRITEMAILBOX(title,type,msg) adds a WRITEMAILBOX
            % command to the command object. 
            %
            % Notes::
            % - title is the message title sent from the brick
            % - type is the sent message type being either 'text',
            % 'numeric' or 'logic'
            % - msg is the message to be sent
            %
            % Example::
            %           cmd.WRITEMAILBOX('abc','text','hello!')
 
            cmd.addSystemCommand(SystemCommands.WriteMailBox);
            cmd.addValue(length(title)+1);
            cmd.addString(title);
            switch type
                case 'text'
                    cmd.addArray(typecast(uint16(length(msg)+1),'uint8'));
                    cmd.addString(msg);
                case 'numeric'
                    cmd.addArray(typecast(uint16(4),'uint8'));
                    cmd.addArray(typecast(single(msg),'uint8'));
                case 'logic'
                    cmd.addArray(typecast(uint16(1),'uint8'));
                    if msg(1) >= 1
                        msg(1) = 1;
                    else msg(1) = 0;
                    end
                    cmd.addArray(typecast(uint8(msg),'uint8'));
            end
        end 
        
    end
end
