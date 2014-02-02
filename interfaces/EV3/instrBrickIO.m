%instrBrickIO Instr interface between MATLAB and the brick (uses the
%instrumentation and control toolbox)
%
% Methods::
%
%  instrBrickIO     Constructor, initialises and opens the instr connection
%  delete           Destructor, closes the instr connection
%
%  open             Open a instr connection to the brick
%  close            Close the instr connection to the brick
%  read             Read data from the brick through instr
%  write            Write data to the brick through instr
%
% Example::
%           instrbtbrick = instrBrickIO(1,'bt','EV3',1)
%           instrwfbrick = instrBrickIO(1,'wf','192.168.1.104',5555,'0016533dbaf5')
%
% Notes::
% - Can connect to the brick through bluetooth or wifi.
% - Uses the instrumentation and control toolbox so only works in windows.

classdef instrBrickIO < BrickIO
    properties
        % connection handle
        handle
        % debug input
        debug = 0;
        % serial connection interface
        serConn;
        % bluetooth brick device name
        btDevice = 'EV3';
        % bluetooth connection channel
        btChannel = 1;
        % wifi brick ip address
        wfAddr = '192.168.1.104';
        % wifi brick tcp port
        wfPort = 5555;
        % wifi brick serial number
        wfSN = '0016533dbaf5';
    end
    
    methods

        function brickIO = instrBrickIO(varargin)
            %instrBrickIO.btBrickIO Create a instrBrickIO object
            %
            % instrBrickIO = btBrickIO(varargin) is an object which
            % initialises and opens a instr connection between MATLAB
            % and the brick using the instrumentation and control toolbox.
            %
            % Notes::
            % - For bluetooth; instrBrickIO(debug,btDevice,btChannel) where
            % debug is a flag specifying output printing (0 or 1), btDevice
            % is the bluetooth device name of the brick and btChannel is 
            % the bluetooth channel.
            % - For wifi; instrBrickIO(debug,wfAddr,wfPort,wfSn) where
            % debug is a flag specifying output printing(0 or 1), wfAddr is
            % the IP address of the brick, wfPort is the TCP connection
            % port and wfSN is the serial number of the brick.
            
            if nargin > 0
                brickIO.debug = varargin{1};
                brickIO.serConn = varargin{2};
                if brickIO.debug > 0
                    fprintf('instrBrickIO init\n');
                end
            end
            % bluetooth
            % brickIO = instrBrickIO(1,'bt','EV3',1)
            if(strcmp(brickIO.serConn,'bt'))
               brickIO.btDevice = varargin{3};
               brickIO.btChannel = varargin{4};
               % set the bt handle
               brickIO.handle = Bluetooth(brickIO.btDevice,brickIO.btChannel);
               % open the bt handle
               brickIO.open;
            end
            % wifi
            % brickIO = instrBrickIO(1,'wf','192.168.1.104',5555,'0016533dbaf5')
            if(strcmp(brickIO.serConn,'wf'))
               brickIO.wfAddr = varargin{3};
               brickIO.wfPort = varargin{4};
               brickIO.wfSN = varargin{5};
               % set the wf handle
               brickIO.handle = tcpip(brickIO.wfAddr,brickIO.wfPort);
               % connection message to the brick
               wmsg = ['GET /target?sn=' brickIO.wfSN ' VMTP1.0' char(10) 'Protocol: EV3'];
               % open the wf handle
               brickIO.open;
               % send the connection message
               brickIO.write(wmsg);
               pause(0.5);
               % receive the reply
               rmsg = fread(brickIO.handle,16);
               if brickIO.debug > 0
                 fprintf('%s',char(rmsg));
               end
            end
        end
        
        function delete(brickIO)
            %instrBrickIO.delete Delete the instrBrickIO object
            %
            % delete(brickIO) closes the instr connection handle
            
            if brickIO.debug > 0
                fprintf('instrBrickIO delete\n');
            end
            % delete the instr handle 
            brickIO.close;
        end
        
        function open(brickIO)
            %instrBrickIO.open Open the instrBrickIO object
            %
            % instrBrickIO.open() opens the instr connection to the brick
            % using fopen.
            
            if brickIO.debug > 0
                fprintf('instrBrickIO open\n');
            end
            % open the instr handle
            fopen(brickIO.handle);
        end
        
        function close(brickIO)
            %instrBrickIO.close Close the instrBrickIO object
            %
            % btBrickIO.close() closes the instr connection the brick
            % using fclose.
            if brickIO.debug > 0
                fprintf('instrBrickIO close\n');
            end 
            % close the instr handle
            fclose(brickIO.handle);
        end
        
        function rmsg = read(brickIO)
            %instrBrickIO.read Read data from the instrBrickIO object
            %
            % rmsg = instrBrickIO.read() reads data from the brick through
            % instr (either bluetooth or wifi) via fread and returns the 
            % data in uint8 format.
            if brickIO.debug > 0
                fprintf('instrBrickIO read\n');
            end 
            % get the number of bytes to be read from the instrCtrl handle
            nLength = fread(brickIO.handle,2);
            % read the remaining bytes
            rmsg = fread(brickIO.handle,double(typecast(uint8(nLength),'uint16')));
            % append the reply size to the return message
            rmsg = uint8([nLength' rmsg']);
        end
        
        function write(brickIO,wmsg)
            %instrBrickIO.write Write data to the instrBrickIO object
            %
            % instrBrickIO.write(wmsg) writes data to the brick through
            % instr (either bluetooth or wifi).
            %
            % Notes::
            % - wmsg is the data to be written to the brick via instr
            % in uint8 format.
            if brickIO.debug > 0
                fprintf('instrBrickIO write\n');
            end 
            fwrite(brickIO.handle,wmsg);
        end
    end 
end