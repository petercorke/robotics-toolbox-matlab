%usbBrickIO USB interface between MATLAB and the brick
%
% Methods::
%
%  usbBrickIO    Constructor, initialises and opens the usb connection
%  delete       Destructor, closes the usb connection
%
%  open         Open a usb connection to the brick
%  close        Close the usb connection to the brick
%  read         Read data from the brick through usb
%  write        Write data to the brick through usb
%
% Example::
%           usbbrick = usbBrickIO()
%
% Notes::
% - Uses the hid library implementation in hidapi.m

classdef usbBrickIO < BrickIO
    properties
        % connection handle
        handle
        % debug input
        debug = 0;
        % vendor ID (EV3 = 0x0694)
        vendorID = 1684;
        % product ID (EV3 = 0x0005)
        productID = 5;
        % read buffer size
        nReadBuffer = 1024;
        % write buffer size
        nWriteBuffer = 1025;
    end
    
    methods
 
        function brickIO = usbBrickIO(varargin)
            %usbBrickIO.usbBrickIO Create a usbBrickIO object
            %
            % usbbrick = usbBrickIO(varargin) is an object which
            % initialises a usb connection between MATLAB and the brick
            % using hidapi.m.
            % 
            % Notes::
            % - Can take one parameter debug which is a flag specifying
            % output printing (0 or 1).
            
            if nargin == 0
                brickIO.debug = 0;
            end
            if nargin > 0
                brickIO.debug = varargin{1}; 
            end
            if brickIO.debug > 0
                fprintf('usbBrickIO init\n');
            end
            % create the usb handle 
            brickIO.handle = hidapi(0,brickIO.vendorID,brickIO.productID,brickIO.nReadBuffer,brickIO.nWriteBuffer);
            % open the brick Io connection
            brickIO.open;
        end
        
        function delete(brickIO)
            %usbBrickIO.delete Delete the usbBrickIO object
            %
            % delete(brickIO) closes the usb connection handle
            
            if brickIO.debug > 0
                fprintf('usbBrickIO delete\n');
            end
            % delete the usb handle 
            delete(brickIO.handle)
        end
        
        % open the brick IO connection
        function open(brickIO)
            %usbBrickIO.open Open the usbBrickIO object
            %
            % usbBrickIO.open() opens the usb handle through the hidapi
            % interface.
            
            if brickIO.debug > 0
                fprintf('usbBrickIO open\n');
            end
            % open the usb handle
            brickIO.handle.open;
        end
        
        function close(brickIO)
            %usbBrickIO.close Close the usbBrickIO object
            %
            % usbBrickIO.close() closes the usb handle through the hidapi
            % interface.
            if brickIO.debug > 0
                fprintf('usbBrickIO close\n');
            end 
            % close the usb handle
            brickIO.handle.close;
        end
        
        function rmsg = read(brickIO)
            %usbBrickIO.read Read data from the usbBrickIO object
            %
            % rmsg = usbBrickIO.read() reads data from the brick through
            % usb and returns the data in uint8 format.
            %
            % Notes::
            % - This function is blocking with no time out in the current
            % implementation.
            
            if brickIO.debug > 0
                fprintf('usbBrickIO read\n');
            end 
            % read from the usb handle
            rmsg = brickIO.handle.read;
            % get the number of read bytes
            nLength = double(typecast(uint8(rmsg(1:2)),'uint16'));
            % format the read message (2 byte length plus message)
            rmsg = rmsg(1:nLength+2);
        end
        
        function write(brickIO,wmsg)
            %usbBrickIO.write Write data to the usbBrickIO object
            %
            % usbBrickIO.write(wmsg) writes data to the brick through usb.
            %
            % Notes::
            % - wmsg is the data to be written to the brick via usb in  
            % uint8 format.
            
            if brickIO.debug > 0
                fprintf('usbBrickIO write\n');
            end 
            % write to the usb handle using report ID 0
            brickIO.handle.write(wmsg,0);
        end
    end 
end