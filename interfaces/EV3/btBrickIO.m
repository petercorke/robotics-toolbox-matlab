%btBrickIO Bluetooth interface between MATLAB and the brick
%
% Methods::
%
%  btBrickIO    Constructor, initialises and opens the bluetooth connection
%  delete       Destructor, closes the bluetooth connection
%
%  open         Open a bluetooth connection to the brick
%  close        Close the bluetooth connection to the brick
%  read         Read data from the brick through bluetooth
%  write        Write data to the brick through bluetooth
%
% Example::
%           btbrick = btBrickIO(1,'/dev/rfcomm0')
%
% Notes::
% - Connects to the bluetooth module on the host through a serial
% connection. Hence be sure that a serial connection to the bluetooth
% module can be made. Also be sure that the bluetooth module can be paired
% to the brick before MATLAB is opened.
% - Works under mac and potentially linux (have yet to find a suitable
% bluetooth device that can pair with the brick under linux).
% - Does not work under windows (will need to either virtualises the serial
% bluetooth port or use the instrumentation and control toolbox BrickIO
% version).

classdef btBrickIO < BrickIO
    properties
        % connection handle
        handle
        % debug input
        debug = 0;
        % bluetooth serial port
        serialPort = '/dev/rfcomm0'
    end
    
    methods
        
        function brickIO = btBrickIO(debug,serialPort)
            %btBrickIO.btBrickIO Create a btBrickIO object
            %
            % btbrick = btBrickIO(debug,serialPort) is an object which
            % initialises and opens a bluetooth connection between MATLAB
            % and the brick using serial functions.
            %
            % Notes::
            % - debug is a flag specifying output printing (0 or 1).
            
            if nargin > 1
                brickIO.debug = debug;
                brickIO.serialPort = serialPort;
            end
            if brickIO.debug > 0
                fprintf('btBrickIO init\n');
            end
            % set the connection handle
            brickIO.handle = serial(brickIO.serialPort);
            % open the conneciton handle
            brickIO.open;
        end
        
        function delete(brickIO)
            %btBrickIO.delete Delete the btBrickIO object
            %
            % delete(brickIO) closes the bluetooth connection handle
            if brickIO.debug > 0
                fprintf('btBrickIO delete\n');
            end
            % delete the bt handle 
            brickIO.close;
        end
        
        function open(brickIO)
            %btBrickIO.open Open the btBrickIO object
            %
            % btBrickIO.open() opens the bluetooth connection to the brick
            % using fopen.
            
            if brickIO.debug > 0
                fprintf('btBrickIO open\n');
            end
            % open the bt handle
            fopen(brickIO.handle);
        end

        function close(brickIO)
            %btBrickIO.close Close the btBrickIO object
            %
            % btBrickIO.close() closes the bluetooth connection the brick
            % using fclose.
            
            if brickIO.debug > 0
                fprintf('btBrickIO close\n');
            end 
            % close the close handle
            fclose(brickIO.handle);
        end
        
        function rmsg = read(brickIO)
            %btBrickIO.read Read data from the btBrickIO object
            %
            % rmsg = btBrickIO.read() reads data from the brick through
            % bluetooth via fread and returns the data in uint8 format.
            
            if brickIO.debug > 0
                fprintf('btBrickIO read\n');
            end 
            % get the number of bytes to be read from the bt handle
            nLength = fread(brickIO.handle,2);
            % read the remaining bytes
            rmsg = fread(brickIO.handle,double(typecast(uint8(nLength),'uint16')));
            % append the reply size to the return message
            rmsg = uint8([nLength' rmsg']);
        end
        
        function write(brickIO,wmsg)
            %btBrickIO.write Write data to the btBrickIO object
            %
            % btBrickIO.write(wmsg) writes data to the brick through
            % bluetooth.
            %
            % Notes::
            % - wmsg is the data to be written to the brick via bluetooth
            % in uint8 format.
            
            if brickIO.debug > 0
                fprintf('btBrickIO write\n');
            end 
            fwrite(brickIO.handle,wmsg);
        end
    end 
end