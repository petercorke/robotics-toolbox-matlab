%Dynamixel  Interface to Dynamixel servos
%
% Methods::
%  Dynamixel    Constructor
%  getpos       Get joint angles
%  setpos       Set joint angles and optionally speed
%  relax        Control relax (zero torque) state
%  setled       Control LEDs on servos
%  gettemp      Temperature of motors
%
% Example::
%
% Notes::
% - interface is via serial to an Arbotix controller running the pypose sketch

% Copyright (c) 2013 Peter Corke

% only a subset of Dynamixel commands supported
% READDATA, WRITEDATA (1 or 2 bytes only)
% WRITESYNC and ACTION not supported but should be



classdef Dynamixel < handle

    properties
        debug;
        serPort;
        nservos;
    end

    properties (Constant)
        % define some imserPortant memory locations
        VERSION = 2;
        DELAYTIME = 5;
        CWLIMIT = 6;
        CCWLIMIT = 8;
        ALARM_LED = 17;
        ALARM_SHUTDOWN = 18;
        LED = 25;
        TORQUE = 24
        GOAL = 30;
        SPEED = 32;
        POS = 36;
        TEMP = 43;
    end

    methods
        function dm = Dynamixel(varargin)
            %Dynamixel.Dynamixel Create Dynamixel interface object
            %
            % DM = Dynamixel(OPTIONS) is an object that represents a connection to a chain
            % of Dynamixel servos connected via an Arbotix controller and serial link to the
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
            
            dm.serPort = opt.port;
            dm.debug = opt.debug;
            dm.nservos = opt.nservos;
            
            % clean up any previous instances of the port, can happen...
            for tty = instrfind('port', opt.port)
                if ~isempty(tty)
                    disp(['serPort ' tty.serPort.port 'is in use.   Closing it.'])
                    fclose(tty);
                    delete(tty);
                end
            end
            
            if opt.verbose
                disp('Establishing connection to Dynamixel chain...');
            end
            
            dm.serPort = serial(dm.serPort,'BaudRate', opt.baud);
            set(dm.serPort,'InputBufferSize',100)
            set(dm.serPort, 'Timeout', 1)
            set(dm.serPort, 'Tag', 'Dynamixel')
                        
            if opt.verbose
                disp('Opening connection to Dynamixel chain...');
            end
            
            pause(0.5);
            try
                fopen(dm.serPort);
            catch me
                disp('open failed');
                me.message
                return
            end
        end
        
        function delete(dm)
            %Dynamixel.delete  Close the serial connection
            %
            % delete(DM) closes the serial connection and removes the DM object
            % from the workspace.
            
            tty = instrfind('port', dm.serPort.port);
            fclose(tty);
            delete(tty);
        end 
        
        function s = char(dm)
            % show serport Status, number of servos
            s = sprintf('Dynamixel chain on serPort %s (%s)', ...
                dm.serPort.port, get(dm.serPort, 'Status'));
            if dm.nservos
                s = strvcat(s, sprintf(' %d servos in chain', dm.nservos));
            end
        end
        
        function display(dm)
            %Dynamixel.display Display parameters
            %
            % DM.display() displays the servo parameters in compact single line format.
            %
            % Notes::
            % - This method is invoked implicitly at the command line when the result
            %   of an expression is a Dynamixel object and the command has no trailing
            %   semicolon.
            %
            % See also Dynamixel.char.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(dm) );
        end % display()
        
        
        function setpos(dm, varargin)
            %Dynamixel.setpos Set position
            %
            % DM.SETPOS(ID, POS) sets the position (0-1023) of servo ID.
            % DM.SETPOS(ID, POS, SPEED) as above but also sets the speed.
            %
            % DM.SETPOS(POS) sets the position of servos 1-N to corresponding elements
            % of the vector POS (1xN).
            % DM.SETPOS(POS, SPEED) as above but also sets the velocity SPEED (1xN).
            %
            % Notes::
            % - N is defined at construction time by the 'nservos' option.
            % - SPEED varies from 0 to 1023, 0 means largest possible speed.
            
            if length(varargin{1}) > 0
                % vector mode
                pos = varargin{1};
                
                if isempty(dm.nservos)
                    error('RTB:Dynamixel:notspec', 'Number of servos not specified');
                end
                if length(pos) ~= dm.nservos
                    error('RTB:Dynamixel:badarg', 'Length of POS vector must match number of servos');
                end
                for j=1:dm.nservos
                    dm.writedata2(j, Dynamixel.GOAL, pos(j));
                end
                
                % need a separate write for this, since pypose writes max of 2 bytes
                if nargin == 3
                    speed = varargin{2};
                    if length(speed) ~= dm.nservos
                        error('RTB:Dynamixel:badarg', 'Length of SPEED vector must match number of servos');
                    end
                    for j=1:dm.nservos
                        dm.writedata2(j, Dynamixel.SPEED, speed(j));
                    end
                end
            else
                % single joint mode
                id = varargin{1}; pos = varargin{2};
                
                dm.writedata2(id, Dynamixel.GOAL, pos);
                
                if nargin == 4
                    speed = varargin{3};
                    dm.writedata2(jid, Dynamixel.SPEED, speed);
                end
            end
        end
        
        
        function relax(dm, id, status)
            %Dynamixel.relax Control relax state
            %
            % DM.RELAX(ID) causes the servo ID to enter zero-torque (relax state)
            % DM.RELAX(ID, FALSE) causes the servo ID to enter position-control mode
            % DM.RELAX([]) causes servos 1 to N to relax
            % DM.RELAX() as above
            % DM.RELAX([], FALSE) causes servos 1 to N to enter position-control mode.
            %
            % Notes::
            % - N is defined at construction time by the 'nservos' option.
            
            if nargin == 1 || isempty(id)
                % vector mode
                if isempty(dm.nservos)
                    error('RTB:Dynamixel:notspec', 'Number of servos not specified');
                end
                if nargin < 3
                    % relax mode
                    for j=1:dm.nservos
                        dm.writedata1(j, Dynamixel.TORQUE, 0);
                    end
                else
                    for j=1:dm.nservos
                        dm.writedata1(j, Dynamixel.TORQUE, status==false);
                    end
                end
            else
                % single joint mode
                if nargin < 3
                    % relax mode
                    dm.writedata1(id, Dynamixel.TORQUE, 0);
                else
                    dm.writedata1(id, Dynamixel.TORQUE, status==false);
                end
            end
        end
        
        function setled(dm, id, status)
            %Dynamixel.led Set LEDs on servos
            %
            % DM.led(ID, STATUS) sets the LED on servo ID to on or off according
            % to the STATUS (logical).
            %
            % DM.led([], STATUS) as above but the LEDs on servos 1 to N are set.
            %
            % Notes::
            % - N is defined at construction time by the 'nservos' option.

            
            if isempty(id)
                % vector mode
                if isempty(dm.nservos)
                    error('RTB:Dynamixel:notspec', 'Number of servos not specified');
                end
                
                for j=1:dm.nservos
                    
                    dm.writedata1(j, Dynamixel.LED, status);
                end
            else
                % single joint mode
                
                dm.writedata1(id, Dynamixel.LED, status);
            end
        end
        
        function p = getpos(dm, id)
            %Dynamixel.getpos Get position
            %
            % P = DM.GETPOS(ID) is the position (0-1023) of servo ID.
            %
            % P = DM.GETPOS() is a vector (1xN) of positions of servos 1 to N.
            %
            % Notes::
            % - N is defined at construction time by the 'nservos' option.
            
            dm.flush();
            
            if nargin == 2
                retval = dm.readdata(id, Dynamixel.POS, 2);
                p = retval.params*[1; 256];
            elseif nargin < 2
                if isempty(dm.nservos)
                    error('RTB:Dynamixel:notspec', 'Number of servos not specified');
                end
                p = [];
                for j=1:dm.nservos
                    retval = dm.readdata(j, Dynamixel.POS, 2);
                    p(j) = retval.params*[1; 256];
                end
            end
        end
        
        function t = gettemp(dm, id)
            %Dynamixel.gettemp Get temperature
            %
            % P = DM.GETTEMP(ID) is the tempeature (deg C) of servo ID.
            %
            % P = DM.GETTEMP() is a vector (1xN) of the temperature of servos 1 to N.
            %
            % Notes::
            % - N is defined at construction time by the 'nservos' option.            
            
            dm.flush();
            
            if nargin == 2
                retval = dm.readdata(id, Dynamixel.TEMP, 1);
                t = retval.params;
            elseif nargin < 2
                if isempty(dm.nservos)
                    error('RTB:Dynamixel:notspec', 'Number of servos not specified');
                end
                t = [];
                for j=1:dm.nservos
                    retval = dm.readdata(j, Dynamixel.TEMP, 1);
                    t(j) = retval.params;
                end
            end
        end
        
        function retval = readdata(dm, id, addr, len)
            dm.command(id, 2, [addr len]);
            
            retval = dm.receive();
        end
        
        function writedata1(dm, id, addr, data)
            % each element of data is converted to a single byte
            dm.command(id, 3, [addr uint8(data)]);
        end
        
        function writedata2(dm, id, addr, data)
            % each element of data is converted to a two byte value
            dm.command(id, 3, [addr typecast( uint16(data), 'uint8')]);
        end
        
        
        function command(dm, id, instruc, data)
            if isempty(id)
                id = 254;   % 0xFE is broadcast
            end
            
            if nargin < 4
                data = [];
            end
            out = [id length(data)+2 instruc data];
            cs = bitcmp(uint8( mod(sum(out), 256)));
            out = [255 255 uint8(out) cs];
            if dm.debug > 0
                fprintf('send:    ');
                fprintf('0x%02x ', out);
                fprintf('\n');
            end
            fwrite(dm.serPort, out)
        end
        
        
        function flush(robot)
            %Flush Buffer
            N = robot.serPort.BytesAvailable();
            while(N ~= 0)
                fread(robot.serPort,N);
                N = robot.serPort.BytesAvailable();
            end
        end
        
        
        function s = receive(dm)
            state = 0;
            if dm.debug > 0
                fprintf('receive: ');
            end
            while true
                c = fread(dm.serPort, 1, 'uint8');
                if dm.debug > 0
                    fprintf('0x%02x ', c);
                end
                switch state
                    case 0  % expecting first 0xff
                        if c == 255
                            state = 1;
                        end
                    case 1  % expecting first 0xff
                        if c == 255
                            state = 2;
                        end
                    case 2  % expecting second 0xff
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
                        
                        if dm.debug > 0
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
        
       %{
        % Low-level Dynamixel bus functions not supported by pypose sketch
        % Need to create better code for the Arbotix board

        function setpos_sync(dm, pos, speed)
            % pos, speed are vectors
        end
        function discover(dm)
            % find how many servos in the chain
        end
        function ping(dm, id)
            dm.command(id, 1);
            
            retval = dm.receive();
            retval
        end
        
        function regwrite(dm, id, addr, data)
            dm.command(id, 4, [addr data]);
        end
        
        function action(dm)
            dm.command(id, 5);
        end
        
        function reset(dm, id)
            dm.command(id, 6);
        end
        
        function syncwrite(dm, addr, matrix)
            % one column per actuator
            dm.command(id, hex2dec('83'));
        end
        %}
    end
end
