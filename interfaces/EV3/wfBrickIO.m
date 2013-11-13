%wfBrickIO Wifi interface between MATLAB and the brick
%
% Methods::
%
%  wfBrickIO    Constructor, initialises and opens the wifi connection
%  delete       Destructor, closes the wifi connection
%
%  open         Open a wifi connection to the brick
%  close        Close the wifi connection to the brick
%  read         Read data from the brick through wifi
%  write        Write data to the brick through wifi
%
% Example::
%           wfbrick = wfBrickIO(1,'192.168.1.104',5555,'0016533dbaf5')
%
% Notes::
% - Java string casting is inconsistent in MATLAB across different
% operating systems. The purpose of OutputStreamSend is to ensure that the
% correct string sequence is sent to the brick, no matter what operating
% system MATLAB is running under.
% - The OutputStreamSend class file will need to be present in the java
% path before a wfBrickIO object is made.
% - This class works across all 3 operating systems.

classdef wfBrickIO < BrickIO
    properties
        % connection handle
        handle
        % debug input
        debug = 0;
        % socket input strem
        inputStream
        % socket output stream
        outputStream
        % brick IP address
        addr = '192.168.1.104';
        % brick tcp port (default is 5555)
        port = 5555;
        % brick serial number
        serialNum = '0016533dbaf5';
    end
    
    methods
        
        function brickIO = wfBrickIO(debug,addr,port,serialNum)
            %wfBrickIO.wfBrickIO Create a wfBrickIO object
            %
            % wfbrick = wfBrickIO(debug,addr,port,serialNum) is an object
            % which initialises and opens a wifi connection between MATLAB
            % and the brick using java functions. The following string is
            % sent to the brick
            % ['GET /target?sn=' brickIO.serialNum ' VMTP1.0' char(10) 'Protocol: EV3']
            % which initialises the conncetion. The brick returns 
            % ['Accept:EV340']
            % 
            % Notes::
            % - debug is a flag specifying output printing (0 or 1).
            % - addr is the IP address of the brick
            % - port is the TCP port of the brick (default is 5555)
            % - serialNum is the serial number of the brick. The serial
            % number can be found in the menu on the EV3 brick or obtained
            % through the emitted UDP packet on port 3015.
            
            if nargin > 0
                brickIO.debug = debug;
                brickIO.addr = addr;
                brickIO.port = port;
                brickIO.serialNum = serialNum;
            end
            if brickIO.debug > 0
                fprintf('wfBrickIO init\n');
            end
            % import the required java libraries
            import java.io.*;
            import java.net.*;
            % add the java path to the class path
            javaaddpath .
            % open the brick IO connection
            brickIO.handle = Socket(brickIO.addr, brickIO.port);
            % set the input stream
            brickIO.inputStream = DataInputStream(brickIO.handle.getInputStream);
            % set the output stream
            brickIO.outputStream = OutputStreamSend();
            % connection message to the brick
            wmsg = ['GET /target?sn=' brickIO.serialNum ' VMTP1.0' char(10) 'Protocol: EV3'];
            if brickIO.debug > 0
                fprintf('Sent: [ %s ]\n',char(wmsg));
            end
            % send the connection message
            brickIO.write(wmsg);
            % receive the reply
            rmsg = brickIO.read;
            if brickIO.debug > 0
                fprintf('Returned: [ %s ]\n',char(rmsg));
            end
        end
        
        function delete(brickIO)
            %wfBrickIO.delete Delete the wfBrickIO object
            %
            % delete(brickIO) closes the wifi connection handle
            
            if brickIO.debug > 0
                fprintf('wfBrickIO delete\n');
            end
            % delete the wf handle 
            brickIO.close
        end
        
        function handle = open(brickIO)
            %wfBrickIO.open Open the wfBrickIO object
            %
            % handle = wfBrickIO.open() returns a handle to the wifi 
            % socket connection using the initialised IP address and TCP 
            % port values from the wfBrickIO constructor.
            
            if brickIO.debug > 0
                fprintf('wfBrickIO open\n');
            end
            % open the socket handle
            handle = Socket(brickIO.addr, brickIO.port);
        end
        
        function close(brickIO)
            %wfBrickIO.close Close the wfBrickIO object
            %
            % wfBrickIO.close() closes the wifi socket connection
            
            if brickIO.debug > 0
                fprintf('wfBrickIO close\n');
            end 
            % close the close handle
            brickIO.handle.close();
        end
        
        function rmsg = read(brickIO)
            %wfBrickIO.read Read data from the wfBrickIO object
            %
            % rmsg = wfBrickIO.read() reads data from the brick through wifi
            % and returns the data in uint8 format.
            %
            % Notes::
            % - This function is blocking with no time out in the current
            % implementation.
            
            if brickIO.debug > 0
                fprintf('wfBrickIO read\n');
            end 
            % block until bytes have been received
            while (brickIO.inputStream.available == 0)
                
            end
            % get the number of bytes to be read from the input stream
            nLength = brickIO.inputStream.available;
            % read the bytes from the input stream
            rmsg = zeros(1,nLength);
            for ii=1:nLength
                rmsg(ii) =  brickIO.inputStream.readByte;
            end
            % convert from double to int8
            rmsg = cast(rmsg,'int8');
            % change from int8 to uint8 (cannot do this in one step)
            rmsg = typecast(rmsg,'uint8');
        end
        
        function write(brickIO,wmsg)
            %wfBrickIO.write Write data to the wfBrickIO object
            %
            % wfBrickIO.write(wmsg) writes data to the brick through wifi.
            %
            % Notes::
            % - wmsg is the data to be written to the brick via wifi in  
            % uint8 format.
            
            if brickIO.debug > 0
                fprintf('wfBrickIO write\n');
            end 
            % add the message to the output stream
            brickIO.outputStream.addtoBufferN(char(wmsg),length(wmsg));
            % send the output stream data
            brickIO.outputStream.send(brickIO.handle.getOutputStream());
            % clear the output stream
            brickIO.outputStream.clear();
        end
    end 
end