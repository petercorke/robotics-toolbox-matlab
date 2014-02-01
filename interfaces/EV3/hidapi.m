%hidpi Interface to the hidapi library
%
% Methods::
%  hidpi                    Constructor, loads the hidapi library
%  delete                   Destructor, closes any open hid connection
%
%  open                     Open the hid device with vendor and product ID
%  close                    Close the hid device connection
%  read                     Read data from the hid device
%  write                    Write data to the hid device
%
%  getHIDInfoString         Get the relevant hid info from the hid device
%  getManufacturersString   Get the manufacturers string from the hid device
%  getProductString         Get the product string from the hid device
%  getSerialNumberString    Get the serial number from the hid device 
%  setNonBlocking           Set non blocking hid read
%  init                     Init the hidapi (executed in open by default)
%  exit                     Exit the hidapi
%  error                    Return the error string 
%  enumerate                Enumerate the connected hid devices
%
% Example::
%           hid = hidapi(1,1684,0005,1024,1024)
%
% Notes::
% - Developed from the hidapi available from http://www.signal11.us/oss/hidapi/
% - Windows: need the hidapi.dll file
% - Mac: need the hidapi.dylib file. Will also need Xcode installed to run load library
% - Linux: will need to compile on host system and copy the resulting .so file 

classdef hidapi < handle
    properties
        % connection handle
        handle
        % debug input
        debug = 0;
        % vendor ID
        vendorID = 0;
        % product ID
        productID = 0;
        % read buffer size
        nReadBuffer = 256;
        % write buffer size
        nWriteBuffer = 256;
        % shared library
        slib = 'hidapi';
        % shared library header
        sheader = 'hidapi.h';
        % isOpen flag
        isOpen = 0;
    end
    
    methods
        
        function hid = hidapi(debug,vendorID,productID,nReadBuffer,nWriteBuffer)
            %hidapi.hidapi Create a hidapi library interface object
            % 
            % hid = hidapi(debug,vendorID,productID,nReadBuffer,nWriteButter)
            % is an object which initialises the hidapi from the corresponding
            % OS library. Other parameters are also initialised. Some OS
            % checking is required in this function to load the correct
            % library.
            %
            % Notes::
            % - debug is a flag specifying output printing (0 or 1).
            % - vendorID is the vendor ID of the hid device (decimal not hex).
            % - productID is the product ID of the hid device (decimal not hex).
            % - nReadBuffer is the length of the read buffer.
            % - nWriteBuffer is the length of the write buffer.
            
            hid.debug = debug;
            if hid.debug > 0
                fprintf('hidapi init\n');
            end
            if nargin > 1
                hid.vendorID = vendorID;
                hid.productID = productID;
                hid.nReadBuffer = nReadBuffer;
                hid.nWriteBuffer = nWriteBuffer;
            end
            % disable the type not found for structure warning
            warning('off','MATLAB:loadlibrary:TypeNotFoundForStructure');
            % check if the library is loaded
            if ~libisloaded('hidapiusb')
                % check the operating system type and load slib 
                if (ispc == 1)
                    % check the bit version
                    if (strcmp(mexext,'mexw32'))
                        hid.slib = 'hidapi32';
                        % load the library via the proto file
                        loadlibrary(hid.slib,@hidapi32_proto,'alias','hidapiusb')
                    end
                    
                    if (strcmp(mexext,'mexw64'))
                        hid.slib = 'hidapi64';
                        % load the library via the proto file
                        loadlibrary(hid.slib,@hidapi64_proto,'alias','hidapiusb')
                    end                    
                else if (ismac == 1)
                        hid.slib = 'hidapi64';
                        % load the library via the proto file
                        loadlibrary(hid.slib,@hidapi64mac_proto,'alias','hidapiusb');
                     else if (isunix == 1)
                            hid.slib = 'hidapi.so';
                            % load the shared library
                            loadlibrary(hid.slib,hid.sheader,'alias','hidapiusb');
                         end
                    end
                end
            end
            % remove the library extension
            hid.slib = 'hidapiusb';
        end
        
        function delete(hid)
            %hidapi.delete Delete hid object
            %
            % delete(hid) closes an open hid device connection. 
            %
            % Notes::
            % - You cannot unloadlibrary in this function as the object is 
            % still present in the MATLAB work space.
            
            if hid.debug > 0
                fprintf('hidapi delete\n');
            end
            if hid.isOpen == 1
                % close the open connection
                hid.close();
            end
        end

        function open(hid)
            %hidapi.open Open a hid object
            %
            % hid.open() opens a connection with a hid device with the
            % initialised values of vendorID and productID from the hidapi
            % constructor.
            %
            % Notes::
            % - The pointer return value from this library call is always
            % null so it is not possible to know if the open was
            % successful. 
            % - The final paramter to the open hidapi library call has
            % different types depending on OS. In windows it is uint16 but
            % linux/mac it is int32.
            
            if hid.debug > 0
                fprintf('hidapi open\n');
            end
            % create a null pointer for the hid_open function (depends on OS)
            if (ispc == 1)
                pNull = libpointer('uint16Ptr');
            end
            if ((ismac == 1) || (isunix == 1))
                pNull = libpointer('int32Ptr');
            end
            % open the hid interface
            [hid.handle,value] = calllib(hid.slib,'hid_open',uint16(hid.vendorID),uint16(hid.productID),pNull);
            % set open flag
            hid.isOpen = 1;
        end
        
        function close(hid)
            %hidapi.close Close hid object
            %
            % hid.close() closes the connection to a hid device.
            
            if hid.debug > 0
                fprintf('hidapi close\n');
            end
            if hid.isOpen == 1
                % close the connect
                calllib(hid.slib,'hid_close',hid.handle);
                % clear open flag
                hid.isOpen = 0;
            end
        end
        

        function rmsg = read(hid)
            %hidapi.rmsg Read from hid object
            %
            % rmsg = hid.read() reads from a hid device and returns the
            % read bytes. Will print an error if no data was read.
 
            if hid.debug > 0
                fprintf('hidapi read\n');
            end
            % read buffer of nReadBuffer length
            buffer = zeros(1,hid.nReadBuffer);
            % create a unit8 pointer 
            pbuffer = libpointer('uint8Ptr', uint8(buffer));
            % read data from HID deivce
            [res,h] = calllib(hid.slib,'hid_read',hid.handle,pbuffer,uint64(length(buffer)));
            % check the response
            if res == 0
               fprintf('hidapi read returned no data\n');
            end
            % return the string value
            rmsg = pbuffer.Value;
        end

        function write(hid,wmsg,reportID)
            %hidapi.write Write to hid object
            %
            % hid.write() writes to a hid device. Will print an error if
            % there is a mismach between the buffer size and the reported
            % number of bytes written.
            
            if hid.debug > 0
                fprintf('hidapi write\n');
            end
            % append a 0 at the front for HID report ID
            wmsg = [reportID wmsg];
            % pad with zeros for nWriteBuffer length
            wmsg(end+(hid.nWriteBuffer-length(wmsg))) = 0;
            % create a unit8 pointer 
            pbuffer = libpointer('uint8Ptr', uint8(wmsg));
            % write the message
            [res,h] = calllib(hid.slib,'hid_write',hid.handle,pbuffer,uint64(length(wmsg)));
            % check the response
            if res ~= length(wmsg)
                fprintf('hidapi write error: wrote %d, sent %d\n',(length(wmsg)-1),res); 
            end
        end
        
        function str = getHIDInfoString(hid,info)
            %hidapi.getHIDInfoString get hid information from object
            %
            % hid.getHIDInfoString(info) gets the corresponding hid info
            % from the hid device
            %
            % Notes::
            % - info is the hid information string.
            
            if hid.debug > 0
                fprintf(['hidapi ' info '\n']);
            end
            % read buffer nReadBuffer length
            buffer = zeros(1,hid.nReadBuffer);
            % create a unit16 pointer (depends on OS)
            if (ispc == 1)
                pbuffer = libpointer('uint16Ptr', uint16(buffer));
            else if ((ismac == 1) || (isunix == 1))
                    pbuffer = libpointer('int32Ptr', int32(buffer));
                 end
            end
            % get the HID info string
            [res,h] = calllib(hid.slib,info,hid.handle,pbuffer,uint64(length(buffer)));
            % check the response
            if res ~= 0
               fprintf(['hidapi ' info ' error\n']);
            end
            % return the string value
            str = sprintf('%s',char(pbuffer.Value));
        end
        
        function str = getManufacturersString(hid)
            %hidapi.getManufacturersString get manufacturers string from hid object
            %
            % hid.getManufacturersString() returns the manufacturers string
            % from the hid device using getHIDInfoString.
            
            str = getHIDInfoString(hid,'hid_get_manufacturer_string');
        end
        
        function str = getProductString(hid)
            %hidapi.getProductString get product string from hid object
            %
            % hid.getProductString() returns the product string from the
            % hid device using getProductString.
            
            str = getHIDInfoString(hid,'hid_get_product_string');
        end
        
        function str = getSerialNumberString(hid)
            %hidapi.getSerialNumberString get product string from hid object
            %
            % hid.getSerialNumberString() returns the serial number string  
            % from the hid device using getSerialNumberString.
            
            str = getHIDInfoString(hid,'hid_get_serial_number_string');
        end  
        
        function setNonBlocking(hid,nonblock)
            %hidapi.setNonBlocking sets non blocking on the hid object
            %
            % hid.setNonBlocking(nonblock) sets the non blocking flag on
            % the hid device connection.
            %
            % Notes::
            % nonblock - 0 disables nonblocking, 1 enables nonblocking
            
            if hid.debug > 0
                fprintf('hidapi setNonBlocking\n');
            end
            % set non blocking
            [res,h] = calllib(hid.slib,'hid_set_nonblocking',hid.handle,uint64(nonblock));
            % check the response
            if res ~= 0
               fprintf('hidapi setNonBlocking error\n');
            end
        end
        
        function init(hid)
            %hidapi.init Init hidapi
            %
            % hid.init() inits the hidapi library. This is called
            % automatically in the library itself with the open function.
            %
            % Notes::
            % - You should not have to call this function directly.
            
            if hid.debug > 0
                fprintf('hidapi init\n');
            end
            res = calllib(hid.slib,'hid_init');
            if res ~= 0
               fprintf('hidapi init error\n');
            end
        end

        function exit(hid)
            %hidapi.exit Exit hidapi
            %
            % hid.exit() exits the hidapi library. 
            %
            % Notes::
            % - You should not have to call this function directly.
            
            if hid.debug > 0
                fprintf('hidapi exit\n');
            end
            res = calllib(hid.slib,'hid_exit');
            if res ~= 0
               fprintf('hidapi exit error\n');
            end
        end
        
        function str = error(hid)
            %hidapi.error Output the hid object error string
            %
            % hid.error() returns the hid device error string if a function
            % produced an error.
            %
            % Notes::
            % - This function must be called explicitly if you think an
            % error was generated from the hid device.
            
            if hid.debug > 0
                fprintf('hidapi error\n');
            end
            [h,str] = calllib(hid.slib,'hid_error',hid.handle);
        end
        
        function str = enumerate(hid,vendorID,productID)
            %hidapi.enumerate Enumerates the hid object
            %
            % str = hid.enumerate(vendorID,productID) enumerates the hid 
            % device with the given vendorID and productID and returns a
            % string with the returned hid information.
            %
            % Notes::
            % - vendorID is the vendor ID (in decimal not hex).
            % - productID is the vendor ID (in decimal not hex).
            % - Using a vendorID and productID of (0,0) will enumerate all
            % connected hid devices.
            % - MATLAB does not have the hid_device_infoPtr struct so some
            % of the returned information will need to be resized and cast
            % into uint8 or chars.
            
            if hid.debug > 0
                fprintf('hidapi enumerate\n');
            end
            % enumerate the hid devices
            str = calllib(u.slib,'hid_enumerate',uint16(vendorID),uint16(productID));
        end
    end 
end