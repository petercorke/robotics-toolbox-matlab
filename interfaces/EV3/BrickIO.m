%BrickIO Abstract class definition for brick input output
%
% Methods::
%  open         Open the connection to the brick
%  close        Close the connection to the brick
%  read         Read data from the brick
%  write        Write data to the brick
%
% Notes::
% - handle is the connection object
% - The read function should return a uint8 datatype
% - The write function should be given a uint8 datatype as a parameter

classdef BrickIO
    properties (Abstract)
        % connection handle
        handle
    end
    
    methods (Abstract)
        % open the brick connection
        open(BrickIO)
        % close the brick connection
        close(BrickIO)
        % read data from the brick
        read(BrickIO)
        % write data to the brick
        write(BrickIO)
    end
end