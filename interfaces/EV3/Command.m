% EV3 command packet construction
%
% Methods::
% Command			Constructor, creates an empty packet
% delete			Destructor, clears the command

% addHeader			Add a header to the packet
% addHeaderReply    Add a header with reply to the packet
% addLength			Add the length of the packet to the front of the packet
% addOpCode			Add an opCode to the packet
% clear				Clear the packet
% LC0				Add a local constant 0 to the packet
% LC1				Add a local constant 1 to the packet
% LC4				Add a local constant 4 to the packet
% GV0				Add a global variable 0 to the packet
% GV4				Add a global variable 4 to the packet
% LCS				Add a local constant string to the packet
%
% Example::
%					cmd = Command();
%					cmd.addHeader(42,0,0);
%					cmd.addOpCode(ByteCodes.Sound);
%					cmd.LC0(SoundSubCodes.Tone);
%					cmd.LC0(volume);
%					cmd.LC4(freq);
%					cmd.LC4(duration);
%					cmd.addLength();
%
% Notes::
% - The LC0 .. LCS definitions are found in bytecodes.h from line 1584

classdef Command < handle

    properties
        msg
    end

    methods
        function cmd = Command(varargin)
			% Command.cmd Create an empty command
			%
			% c = Command(OPTIONS) is an object that represents an EV3 command
			
            cmd.msg = uint8([]);
        end
		
	    function delete(cmd)
			% Command.delete Clear command
			%
			% delete(c) clears the command
			
            cmd.msg = '';
        end 
        
        function addHeader(cmd, counter, GV, LV)
		   % Command.addHeader Adds packet header
		   %
		   % Adds a header to the packet (0x80)
		   
           cmd.msg = [cmd.msg typecast( uint16(counter), 'uint8'), uint8(128), typecast( bitor(bitshift(uint16(LV),10),uint16(GV)), 'uint8')];
        end
        
        function addHeaderReply(cmd, counter, nG, nL)
		   % Command.addHeaderReply Adds a packet header with reply
		   %
		   % Adds a header to the packet with reply (0x00)
		   
           cmd.msg = [cmd.msg typecast( uint16(counter), 'uint8'), uint8(0), typecast( bitor(bitshift(uint16(nL),10),uint16(nG)), 'uint8')];
        end
        
        function addLength(cmd)
		   % Command.addLength Adds packet length
		   %
		   % Adds the packet length to the start of the packet
		   
           cmd.msg = [typecast( uint16(length(cmd.msg)), 'uint8') cmd.msg]; 
        end
        
        function addOpCode(cmd,v)
		   % Command.addOpCode Adds an opCode
		   %
		   % Adds an opCode to the packet
		   %
		   % Notes::
		   % - The opcodes can be found in ByteCodes.m
		   
           cmd.msg = [cmd.msg uint8(v)]; 
        end
        
        function clear(cmd)
		    % Command.clear Clear command
			%
			% Clears the message
            cmd.msg = '';
        end
        
        function s = char(cmd)
            s = '';
            for i=1:length(cmd.msg)
                s = [s sprintf(' %d', cmd.msg(i))];
            end
        end
        
        function display(cmd)
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(cmd) );
        end % display()
        
        function LC0(cmd, v)
		    % Command.LC0 Add a local constant 0 to the packet
			%
            % Local constant 0 is defined as ((v & PRIMPAR_VALUE) | PRIMPAR_SHORT | PRIMPAR_CONST)
			
            cmd.msg = [cmd.msg bitor(bitand(uint8(v),uint8(Primitives.pvalue)),bitor(uint8(Primitives.pshort),uint8(Primitives.pconst)))]; 
        end
        
        function LC1(cmd, v)
		    % Command.LC1 Add a local constant 1 to the packet
			%
            % Local constant 1 is defined as (PRIMPAR_LONG  | PRIMPAR_CONST | PRIMPAR_1_BYTE),(v & 0xFF)
            cmd.msg = [cmd.msg bitor(bitor(uint8(Primitives.plong),uint8(Primitives.pconst)),(uint8(Primitives.p1_byte))), bitand(v,255)];
        end
        
        function LC4(cmd, v)
		    % Command.LC4 Add a local constant 4 to the packet
			%
            % Local constant 4 is defined as(PRIMPAR_LONG  | PRIMPAR_CONST | PRIMPAR_4_BYTES),((ULONG)v & 0xFF),(((ULONG)v >> (ULONG)8) & 0xFF),(((ULONG)v >> (ULONG)16) & 0xFF),(((ULONG)v >> (ULONG)24) & 0xFF)
            cmd.msg = [cmd.msg bitor(bitor(uint8(Primitives.plong),uint8(Primitives.pconst)),(uint8(Primitives.p4_byte))), typecast( uint32(v), 'uint8')];
        end
            
        function GV0(cmd, i)
		    % Command.GV0 Add a global constant 0 to the packet
			%
            % Global constant 0 is defined as ((i & PRIMPAR_INDEX) | PRIMPAR_SHORT | PRIMPAR_VARIABEL | PRIMPAR_GLOBAL)
            cmd.msg = [cmd.msg bitor(bitor(bitor(bitand(uint8(i),Primitives.pindex),Primitives.pshort),Primitives.pvariabel),Primitives.pglobal)];
        end
      
        function GV4(cmd, i)
		    % Command.GV5 Add a global constant 4 to the packet
			%
            % Global constant 4 is defined as (PRIMPAR_LONG  | PRIMPAR_VARIABEL | PRIMPAR_GLOBAL | PRIMPAR_4_BYTES),(i & 0xFF),((i >> 8) & 0xFF),((i >> 16) & 0xFF),((i >> 24) & 0xFF)
            cmd.msg = [cmd.msg bitor(bitor(bitor(uint8(Primitives.plong),uint8(Primitives.pvariabel)),Primitives.pglobal),(uint8(Primitives.p4_byte))), typecast( uint32(i), 'uint8')];
        end
        
        function LCS(cmd)
		    % Command.LCS Add a local constant string to the packet
			%
            % Local constant string is defined as (PRIMPAR_LONG | PRIMPAR_STRING)
            cmd.msg = [cmd.msg bitor(Primitives.plong,Primitives.pstring_old)];
        end
        
        function ASCII(cmd, v)
            % add ascii character
            cmd.msg = [cmd.msg uint8(v)];
        end
 
    end
    

end
