%Arbotix  Interface to Arbotix robot-arm controller
%
%  A concrete subclass of the abstract Machine class that implements a
%  connection over a serial port to an Arbotix robot-arm controller.
%
% Methods::
%  Arbotix      Constructor, establishes serial communications
%  delete       Destructor, closes serial connection
%  getpos       Get joint angles
%  setpos       Set joint angles and optionally speed
%  setpath      Load a trajectory into Arbotix RAM
%  relax        Control relax (zero torque) state
%  setled       Control LEDs on servos
%  gettemp      Temperature of motors
%-
%  writedata1   Write byte data to servo control table
%  writedata2   Write word data to servo control table
%  readdata     Read servo control table
%-
%  command      Execute command on servo
%  flush        Flushes serial data buffer
%  receive      Receive data
%
% Example::
%         arb=Arbotix('port', '/dev/tty.usbserial-A800JDPN', 'nservos', 5);
%         q = arb.getpos();
%         arb.setpos(q + 0.1);
%
% Notes::
% - This is experimental code.
% - Considers the robot as a string of motors, and the last joint is
%   assumed to be the gripper.  This should be abstracted, at the moment this
%   is done in RobotArm.
% - Connects via serial port to an Arbotix controller running the pypose
%   sketch.
%
% See also Machine, RobotArm.

% Copyright (C) 1993-2015, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

% Copyright (c) 2013 Peter Corke

% only a subset of Arbotix commands supported
% READDATA, WRITEDATA (1 or 2 bytes only)
% WRITESYNC and ACTION not supported but should be

% Should subclass an abstract superclass Machine

classdef Arbotix < Machine

    properties
        serPort;
        nservos;
        
        gripper
    end

    properties (Constant)
        %%% define some important memory locations
        ADDR_VERSION = 2;
        ADDR_DELAYTIME = 5;
        ADDR_CWLIMIT = 6;
        ADDR_CCWLIMIT = 8;
        ADDR_ALARM_LED = 17;
        ADDR_ALARM_SHUTDOWN = 18;
        ADDR_LED = 25;
        ADDR_TORQUE = 24
        ADDR_GOAL = 30;
        ADDR_SPEED = 32;
        ADDR_POS = 36;
        ADDR_TEMP = 43;
        
        %%% define the instructions
        %  native Arbotix instructions
        INS_READ_DATA = 2;
        INS_WRITE_DATA = 3;
        
        %  pseudo Arbotix instructions, implemented by Arbotix pypose
        INS_ARB_SIZE_POSE = 7;
        INS_ARB_LOAD_POSE = 8;
        INS_ARB_LOAD_SEQ = 9;
        INS_ARB_PLAY_SEQ = 10;
        INS_ARB_LOOP_SEQ = 11;
        INS_ARB_TEST = 25;
    end
    
    % Communications format:
    % 
    % To Arbotix:
    %
    % 0xff 0xff ID  LEN  INSTRUC PAYLOAD CHKSUM
    %
    % ID single byte, identifies the servo on the bus 0-(N-1)
    % LEN single byte, indicates the number of bytes to follow the LEN byte incl checksum
    % INSTRUC a single byte saying what to do:
    %   2  read data from control table: PAYLOAD=START_ADR, NUM_BYTES
    %   3  write data to control table: PAYLOAD=START_ADR, P1, P2, ...
    %
    %   7  set #DOF: PAYLOAD=NP
    %   8  load pose: PAYLOAD=POSE# $Q_1 $Q_2 ... $Q_NP
    %   9  load sequence: PAYLOAD=POSE# $T POSE# $T ... 255
    %  10  run sequence
    %  11  loop sequence
    %
    % CHKSUM is a single byte, modulo 256 sum over ID .. PAYLOAD.
    %
    % Notes::
    % - $X means that X is a 16-bit (word) quantity.
    % - Arbotix only passes instructions 2 and 3 to the servos, other Dynamixel
    %   instructions are inaccessible.
    % - Instructions 7-11 are implemented by the Arbotix pypose sketch.
    %
    % Methods::
    %  writedata1    Writes byte data to control table
    %  writedata2    Writes word data to control table
    %  readdata      Reads data from control table
    %  command       Invokes instruction on servo

    
    methods
        function arb = Arbotix(varargin)
            %Arbotix.Arbotix Create Arbotix interface object
            %
            % ARB = Arbotix(OPTIONS) is an object that represents a connection to a chain
            % of Arbotix servos connected via an Arbotix controller and serial link to the
            % host computer.
            %
            % Options::
            %  'port',P      Name of the serial port device, eg. /dev/tty.USB0
            %  'baud',B      Set baud rate (default 38400)
            %  'debug',D     Debug level, show communications packets (default 0)
            %  'nservos',N   Number of servos in the chain
            
            opt.port = '';
            opt.debug = false;
            opt.nservos = [];
            opt.baud = 38400;
            
            opt = tb_optparse(opt, varargin);
            
            arb.serPort = opt.port;
            arb.debug = opt.debug;
            arb.nservos = opt.nservos;
            
            arb.connect(opt);
                       
            % open and closed amount
            arb.gripper = [0 2.6];
            
        end
        
        function connect(arb, opt)
            %Arbotix.connect  Connect to the physical robot controller
            %
            % ARB.connect() establish a serial connection to the physical robot
            % controller.
            %
            % See also Arbotix.disconnect.
            
            % clean up any previous instances of the port, can happen...
            for tty = instrfind('port', opt.port)
                if ~isempty(tty)
                    disp(['serPort ' tty.port 'is in use.   Closing it.'])
                    fclose(tty);
                    delete(tty);
                end
            end
            
            if opt.verbose
                disp('Establishing connection to Arbotix chain...');
            end
            
            arb.serPort = serial(arb.serPort,'BaudRate', opt.baud);
            set(arb.serPort,'InputBufferSize',1000)
            set(arb.serPort, 'Timeout', 1)
            set(arb.serPort, 'Tag', 'Arbotix')
            
            if opt.verbose
                disp('Opening connection to Arbotix chain...');
            end
            
            pause(0.5);
            try
                fopen(arb.serPort);
            catch me
                disp('open failed');
                me.message
                return
            end
            
            arb.flush();
        end
        
        function disconnect(arb)
            %Arbotix.disconnect  Disconnect from the physical robot controller
            %
            % ARB.disconnect() closes the serial connection.
            %
            % See also Arbotix.connect.
            
            tty = instrfind('port', arb.serPort.port);
            fclose(tty);
            delete(tty);
        end
        
        function s = char(arb)
            %Arbotix.char  Convert Arbotix status to string
            %
            % C = ARB.char() is a string that succinctly describes the status
            % of the Arbotix controller link.
            
            % show serport Status, number of servos
            s = sprintf('Arbotix chain on serPort %s (%s)', ...
                arb.serPort.port, get(arb.serPort, 'Status'));
            if arb.nservos
                s = strvcat(s, sprintf(' %d servos in chain', arb.nservos));
            end
        end
        
        function display(arb)
            %Arbotix.display Display parameters
            %
            % ARB.display() displays the servo parameters in compact single line format.
            %
            % Notes::
            % - This method is invoked implicitly at the command line when the result
            %   of an expression is a Arbotix object and the command has no trailing
            %   semicolon.
            %
            % See also Arbotix.char.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(arb) );
        end % display()

                
        function p = getpos(arb, id)
            %Arbotix.getpos Get position
            %
            % P = ARB.GETPOS(ID) is the position (0-1023) of servo ID.
            %
            % P = ARB.GETPOS([]) is a vector (1xN) of positions of servos 1 to N.
            %
            % Notes::
            % - N is defined at construction time by the 'nservos' option.
            %
            % See also Arbotix.e2a.
            
            arb.flush();
            
            if nargin < 2
                id = [];
            end
            
            if ~isempty(id)
                retval = arb.readdata(id, Arbotix.ADDR_POS, 2);
                p = Arbotix.e2a( retval.params*[1; 256] );
            else
                if isempty(arb.nservos)
                    error('RTB:Arbotix:notspec', 'Number of servos not specified');
                end
                p = [];
                for j=1:arb.nservos
                    retval = arb.readdata(j, Arbotix.ADDR_POS, 2);
                    p(j) = Arbotix.e2a( retval.params*[1; 256] );
                end
            end
        end
        
        function setpos(arb, varargin)
            %Arbotix.setpos Set position
            %
            % ARB.SETPOS(ID, POS) sets the position (0-1023) of servo ID.
            % ARB.SETPOS(ID, POS, SPEED) as above but also sets the speed.
            %
            % ARB.SETPOS(POS) sets the position of servos 1-N to corresponding elements
            % of the vector POS (1xN).
            % ARB.SETPOS(POS, SPEED) as above but also sets the velocity SPEED (1xN).
            %
            % Notes::
            % - ID is in the range 1 to N
            % - N is defined at construction time by the 'nservos' option.
            % - SPEED varies from 0 to 1023, 0 means largest possible speed.
            %
            % See also Arbotix.a2e.

            
            if length(varargin{1}) > 1
                % vector mode
                pos = varargin{1};
                
                if isempty(arb.nservos)
                    error('RTB:Arbotix:notspec', 'Number of servos not specified');
                end
                if length(pos) ~= arb.nservos
                    error('RTB:Arbotix:badarg', 'Length of POS vector must match number of servos');
                end
                for j=1:arb.nservos
                    arb.writedata2(j, Arbotix.ADDR_GOAL, Arbotix.a2e( pos(j) ));
                end
                
                % need a separate write for this, since pypose writes max of 2 bytes
                if nargin == 3
                    speed = varargin{2};
                    if length(speed) ~= arb.nservos
                        error('RTB:Arbotix:badarg', 'Length of SPEED vector must match number of servos');
                    end
                    for j=1:arb.nservos
                        arb.writedata2(j, Arbotix.ADDR_SPEED, speed(j));
                    end
                end
            else
                % single joint mode
                id = varargin{1}; pos = varargin{2};
                
                arb.writedata2(id, Arbotix.ADDR_GOAL, Arbotix.a2e( pos ));
                
                if nargin == 4
                    speed = varargin{3};
                    arb.writedata2(id, Arbotix.ADDR_SPEED, speed);
                end
            end
        end
        

        function setpath(arb, jt, t)
            %Arbotix.setpath Load a path into Arbotix controller
            %
            % ARB.setpath(JT) stores the path JT (PxN) in the Arbotix controller
            % where P is the number of points on the path and N is the number of
            % robot joints.  Allows for smooth multi-axis motion.
            %
            % See also Arbotix.a2e.            
            
            % will the path fit in Arbotix memory?
            if numrows(jt) > 30
                error('RTB:Arbotix:badarg', 'Too many points on trajectory (30 max)');
            end
            
            jt = Arbotix.a2e(jt);   % convert to encoder values
            
            % set the number of servos
            arb.command(253, Arbotix.INS_ARB_SIZE_POSE, uint8(numcols(jt)));
            pause(0.2);  % this delay is important
            
            % set the poses
            %  payload: <pose#> q1 q2 .. qN
            for i=1:numrows(jt)
                pose = jt(i,:);
                arb.command(253, Arbotix.INS_ARB_LOAD_POSE, [i-1 typecast( uint16(pose), 'uint8')]);
            end
            
            % set the sequence in which to execute the poses
            if nargin < 3
                dt = 200;    % milliseconds between poses
            else
                dt = t*1000;
            end
            dt8 = typecast( uint16(dt), 'uint8');
            
            
            cmd = [];
            for i=1:numrows(jt)
                cmd = [cmd uint8(i-1) dt8];
            end

            cmd = [cmd 255 0 0];   % mark end of sequence
            arb.command(253, Arbotix.INS_ARB_LOAD_SEQ, cmd);

            
            % now do it
            arb.command(253, Arbotix.INS_ARB_PLAY_SEQ);
            
        end            
        
        

                
        function relax(arb, id, status)
            %Arbotix.relax Control relax state
            %
            % ARB.RELAX(ID) causes the servo ID to enter zero-torque (relax state)
            % ARB.RELAX(ID, FALSE) causes the servo ID to enter position-control mode
            % ARB.RELAX([]) causes servos 1 to N to relax
            % ARB.RELAX() as above
            % ARB.RELAX([], FALSE) causes servos 1 to N to enter position-control mode.
            %
            % Notes::
            % - N is defined at construction time by the 'nservos' option.
            
            if nargin == 1 || isempty(id)
                % vector mode
                if isempty(arb.nservos)
                    error('RTB:Arbotix:notspec', 'Number of servos not specified');
                end
                if nargin < 3
                    % relax mode
                    for j=1:arb.nservos
                        arb.writedata1(j, Arbotix.ADDR_TORQUE, 0);
                    end
                else
                    for j=1:arb.nservos
                        arb.writedata1(j, Arbotix.ADDR_TORQUE, status==false);
                    end
                end
            else
                % single joint mode
                if nargin < 3
                    % relax mode
                    arb.writedata1(id, Arbotix.ADDR_TORQUE, 0);
                else
                    arb.writedata1(id, Arbotix.ADDR_TORQUE, status==false);
                end
            end
        end
        
        function setled(arb, id, status)
            %Arbotix.led Set LEDs on servos
            %
            % ARB.led(ID, STATUS) sets the LED on servo ID to on or off according
            % to the STATUS (logical).
            %
            % ARB.led([], STATUS) as above but the LEDs on servos 1 to N are set.
            %
            % Notes::
            % - N is defined at construction time by the 'nservos' option.

            
            if isempty(id)
                % vector mode
                if isempty(arb.nservos)
                    error('RTB:Arbotix:notspec', 'Number of servos not specified');
                end
                
                for j=1:arb.nservos
                    
                    arb.writedata1(j, Arbotix.ADDR_LED, status);
                end
            else
                % single joint mode
                
                arb.writedata1(id, Arbotix.ADDR_LED, status);
            end
        end

        
        function t = gettemp(arb, id)
            %Arbotix.gettemp Get temperature
            %
            % T = ARB.GETTEMP(ID) is the tempeature (deg C) of servo ID.
            %
            % T = ARB.GETTEMP() is a vector (1xN) of the temperature of servos 1 to N.
            %
            % Notes::
            % - N is defined at construction time by the 'nservos' option.            
            
            arb.flush();
            
            if nargin == 2
                retval = arb.readdata(id, Arbotix.ADDR_TEMP, 1);
                t = retval.params;
            elseif nargin < 2
                if isempty(arb.nservos)
                    error('RTB:Arbotix:notspec', 'Number of servos not specified');
                end
                t = [];
                for j=1:arb.nservos
                    retval = arb.readdata(j, Arbotix.ADDR_TEMP, 1);
                    t(j) = retval.params;
                end
            end
        end
        
        function retval = readdata(arb, id, addr, len)
            %Arbotix.readdata Read byte data from servo control table
            %
            % R = ARB.READDATA(ID, ADDR) reads the successive elements of the servo
            % control table for servo ID, starting at address ADDR.  The complete
            % return status in the structure R, and the byte data is a vector in the 
            % field 'params'.
            %
            % Notes::
            % - ID is in the range 0 to N-1, where N is the number of servos in the system.
            % - If 'debug' was enabled in the constructor then the hex values are echoed
            %   to the screen as well as being sent to the Arbotix.
            %
            % See also Arbotix.receive, Arbotix.command.
            
            arb.command(id, Arbotix.INS_READ_DATA, [addr len]);
            
            retval = arb.receive();
        end
        
        function writedata1(arb, id, addr, data)
            %Arbotix.writedata1 Write byte data to servo control table
            %
            % ARB.WRITEDATA1(ID, ADDR, DATA) writes the successive elements of DATA to the
            % servo control table for servo ID, starting at address ADDR.  The values of
            % DATA must be in the range 0 to 255.
            %
            % Notes::
            % - ID is in the range 0 to N-1, where N is the number of servos in the system.
            % - If 'debug' was enabled in the constructor then the hex values are echoed
            %   to the screen as well as being sent to the Arbotix.
            %
            % See also Arbotix.writedata2, Arbotix.command.
            
            % each element of data is converted to a single byte

            arb.command(id, Arbotix.INS_WRITE_DATA, [addr uint8(data)]);
        end
        
        function writedata2(arb, id, addr, data)
            %Arbotix.writedata2 Write word data to servo control table
            %
            % ARB.WRITEDATA2(ID, ADDR, DATA) writes the successive elements of DATA to the
            % servo control table for servo ID, starting at address ADDR.  The values of
            % DATA must be in the range 0 to 65535.
            %
            % Notes::
            % - ID is in the range 0 to N-1, where N is the number of servos in the system.
            % - If 'debug' was enabled in the constructor then the hex values are echoed
            %   to the screen as well as being sent to the Arbotix.
            %
            % See also Arbotix.writedata1, Arbotix.command.
            
            % each element of data is converted to a two byte value
            arb.command(id, Arbotix.INS_WRITE_DATA, [addr typecast( uint16(data), 'uint8')]);
        end
        
        
        function out = command(arb, id, instruc, data)
            %Arbotix.command Execute command on servo
            %
            % R = ARB.COMMAND(ID, INSTRUC) executes the instruction INSTRUC on servo ID.
            %
            % R = ARB.COMMAND(ID, INSTRUC, DATA) as above but the vector DATA forms the
            % payload of the command message, and all numeric values in DATA must be
            % in the range 0 to 255.
            %
            % The optional output argument R is a structure holding the return status.
            %
            % Notes::
            % - ID is in the range 0 to N-1, where N is the number of servos in the system.
            % - Values for INSTRUC are defined as class properties INS_*.
            % - If 'debug' was enabled in the constructor then the hex values are echoed
            %   to the screen as well as being sent to the Arbotix.
            % - If an output argument is requested the serial channel is flushed first.
            %
            % See also Arbotix.receive, Arbotix.flush.
            
            if nargout > 0
                arb.flush();
            end
            
            if isempty(id)
                id = 254;   % 0xFE is broadcast
            end
            
            if nargin < 4
                data = [];
            end
            out = [id length(data)+2 instruc data];
            cs = bitcmp(uint8( mod(sum(out), 256)));
            out = [255 255 uint8(out) cs];
            if arb.debug > 0
                fprintf('send:    ');
                fprintf('0x%02x ', out);
                fprintf('\n');
            end
            fwrite(arb.serPort, out);
            
            if nargout > 0
                out = arb.receive();
            end
            
        end
        
        
        function out = flush(robot)
            %Arbotix.flush Flush the receive buffer
            %
            % ARB.FLUSH() flushes the serial input buffer, data is discarded.
            %
            % S = ARB.FLUSH() as above but returns a vector of all bytes flushed from
            % the channel.
            %           
            % Notes::
            % - Every command sent to the Arbotix elicits a reply.
            % - The method receive() should be called after every command.
            % - Some Arbotix commands also return diagnostic text information.
            %
            % See also Arbotix.receive, Arbotix.parse.
            
            %Flush Buffer
            N = robot.serPort.BytesAvailable();
            data = [];
            % this returns a maximum of input buffer size
            while (N ~= 0)
                data = [data; fread(robot.serPort, N)];
                pause(0.1); % seem to need this
                N = robot.serPort.BytesAvailable();
            end
            
            if nargout > 0
                out = data;
            end
            
        end
       
        
        function s = receive(arb)
            %Arbotix.receive Decode Arbotix return packet
            %
            % R = ARB.RECEIVE() reads and parses the return packet from the Arbotix 
            % and returns a structure with the following fields:
            %  id        The id of the servo that sent the message
            %  error     Error code, 0 means OK
            %  params    The returned parameters, can be a vector of byte values
            %           
            % Notes::
            % - Every command sent to the Arbotix elicits a reply.
            % - The method receive() should be called after every command.
            % - Some Arbotix commands also return diagnostic text information.
            % - If 'debug' was enabled in the constructor then the hex values are echoed

            %
            % See also Arbotix.command, Arbotix.flush.
            state = 0;
            if arb.debug > 0
                fprintf('receive: ');
            end
            while true
                c = fread(arb.serPort, 1, 'uint8');
                if arb.debug > 0
                    fprintf('0x%02x ', c);
                end
                switch state
                    case 0  % expecting first 0xff
                        if c == 255
                            state = 1;
                        end
                    case 1  % expecting second 0xff
                        if c == 255
                            state = 2;
                        end
                    case 2  % expecting id
                        s.id = c;
                        state = 3;
                    case 3  % expecting length
                        len = c;
                        count = len-2;
                        params = [];
                        state = 4;
                    case 4  % expecting error code
                        s.error = c;
                        state = 5;
                    case 5  % expecting parameters
                        params = [params c];
                        count = count - 1;
                        if count == 0
                            state = 6;
                            s.params = params;
                        end
                    case 6  % expecting checksum
                        cs = bitcmp(uint8( mod(s.id + len + sum(params), 256)));
                        
                        if arb.debug > 0
                            fprintf('[0x%02x]\n', cs);
                        end
                        if cs ~= c
                            fprintf('checksum fail: is %d, should be %d\n', ...
                                c, cs);
                        end
                        state = 0;
                        
                        
                        break;
                end
            end
        end
        
%        % Low-level Dynamixel bus functions not supported by pypose sketch
%        % Need to create better code for the Arbotix board
%
%        function setpos_sync(arb, pos, speed)
%            % pos, speed are vectors
%        end
%        function discover(arb)
%            % find how many servos in the chain
%        end
%        function ping(arb, id)
%            arb.command(id, 1);
%            
%            retval = arb.receive();
%            retval
%        end
%        
%        function regwrite(arb, id, addr, data)
%            arb.command(id, 4, [addr data]);
%        end
%        
%        function action(arb)
%            arb.command(id, 5);
%        end
%        
%        function reset(arb, id)
%            arb.command(id, 6);
%        end
%        
%        function syncwrite(arb, addr, matrix)
%            % one column per actuator
%            arb.command(id, hex2dec('83'));
%        end
    end
    
    methods(Static)
        function a = e2a(e)
            %Arbotix.e2a Convert encoder to angle
            %
            % A = ARB.E2A(E) is a vector of joint angles A corresponding to the
            % vector of encoder values E.
            %
            % TODO:
            % - Scale factor is constant, should be a parameter to constructor.
            a = (e - 512) / 512 * 150 / 180 * pi;
            
        end
        
        function e = a2e(a)
            %Arbotix.a2e Convert angle to encoder
            %
            % E = ARB.A2E(A) is a vector of encoder values E corresponding to the
            % vector of joint angles A.
            % TODO:
            % - Scale factor is constant, should be a parameter to constructor.
            e = a / pi * 180 / 150 * 512  + 512;
        end
        
        function parse(s)
            %Arbotix.parse Parse Arbotix return strings
            %
            % ARB.PARSE(S) parses the string returned from the Arbotix controller and
            % prints diagnostic text.  The string S contains a mixture of Dynamixel
            % style return packets and diagnostic text.
            %
            % Notes::
            % - Every command sent to the Arbotix elicits a reply.
            % - The method receive() should be called after every command.
            % - Some Arbotix commands also return diagnostic text information.
            %
            % See also Arbotix.flush, Arbotix.command.
            
            str = [];
            
            while length(s) > 0
                if s(1) == 255 && s(2) == 255
                    % we have a packet
                    len = s(4);
                    pkt = s(1:4+len);
                    s = s(4+len+1:end);
                else
                    % we have a regular string character
                    str = [str s(1)];
                    s = s(2:end);
                end
            end
            fprintf('str: %s\n', char(str));
        end
        
    end
end
