classdef Create < handle
    properties
        serPort
        delay
        cur_mode           % current mode: passive, safe, full
    end

    methods
        function robot = Create(varargin)
        %[serPort] = RoombaInit(my_COM)
        % initializes serial port for use with Roomba
        % COMMport is the number of the comm port 
        % ex. RoombaInit(1) sets port = 'COM1'
        % note that it sets baudrate to a default of 57600
        % can be changed (see SCI).  
        % An optional time delay can be added after all commands
        % if your code crashes frequently.  15 ms is recommended by irobot
        % By; Joel Esposito, US Naval Academy, 2011

        % This code puts the robot in CONTROL(132) mode, which means does NOT stop 
        % when cliff sensors or wheel drops are true; can also run while plugged into charger

            Contrl = 132;

            % Esposito 9/2008 

            warning off

            opt.port = '';
            opt.delay = 0.015;

            opt = tb_optparse(opt, varargin);

            robot.delay = opt.delay;

            % clean up any previous instances of the port, can happen...
            for tty = instrfind('port', opt.port)
                if ~isempty(tty)
                    disp(['serPort ' tty.serPort.port 'is in use.   Closing it.'])
                    fclose(tty);
                    delete(tty);
                end
            end
                
            if opt.verbose
                disp('Establishing connection to Roomba...');
            end

            % defaults at 57600, can change
            robot.serPort = serial(opt.port,'BaudRate', 57600);
            set(robot.serPort,'Terminator','LF')
            set(robot.serPort,'InputBufferSize',100)
            set(robot.serPort, 'Timeout', 1)
            set(robot.serPort, 'ByteOrder','bigEndian');
            set(robot.serPort, 'Tag', 'Roomba')

            robot.serPort

            if opt.verbose
                disp('Opening connection to Roomba...');
            end

            pause(0.5);
            try
            fopen(robot.serPort);
            catch me
                disp('open failed');
                me.message
                return
            end

            %% Confirm two way connumication
            disp('Setting Roomba to Control Mode...');

            % Start! and see if its alive
            robot.mode('passive');

            robot.mode('full');

            % light LEDS
            robot.write([139 25 0 128]);

            % set song 1, default beep
            robot.write([140 1 1 48 20], 0.05);

            % sing it
            robot.write([141 1])

            disp('I am alive if my two outboard lights came on')

            confirmation = robot.read(4, 0.1);
        end

        function msg = read(robot, nbytes, delay)
            msg = fread(robot.serPort, nbytes);
            if nargin > 2
                pause(delay);
            end
        end

        function v = fread(robot, n, type, delay)
            v = fread(robot.serPort, n, type);

            if nargin > 3
                pause(delay);
            else
                pause(robot.delay)
            end
        end

        function write(robot, msg, delay)
            fwrite(robot.serPort, msg);
            fprintf('write: '); disp(msg);
            
            if nargin > 2
                pause(delay);
            else
                pause(robot.delay)
            end
        end

        function fwrite(robot, val, type, delay)
            fwrite(robot.serPort, val, type);
            disp(['fwrite:' val '(' type ')']);

            if nargin > 3
                pause(delay);
            else
                pause(robot.delay)
            end
        end


        function flush(robot)
            %Flush Buffer    
            N = robot.serPort.BytesAvailable();
            while(N ~= 0) 
                fread(robot.serPort,N);
                N = robot.serPort.BytesAvailable();
            end
        end

        function delete(robot)
            tty = instrfind('port', robot.serPort.port);
            fclose(tty);
            pause(1)
            delete(tty);
            pause(1)
        end

        function s = char(robot)
            s = sprintf('iCreate robot on port %s', robot.serPort.port);
        end
    end % methods
end % classdef
