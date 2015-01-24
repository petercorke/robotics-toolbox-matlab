%Machine  Superclass for an interface to a physical robot controller
%
% An abstract superclass for implementing connections to physical robot controllers.  
%
%
% Abstract methods::
% These methods must be implemented in a subclass
%
%  open         Establishes connection to physical robot
%  close        Close connection to physical robot
%  getpos       Get joint angles
%  setpos       Set joint angles and optionally speed
%
% Notes::
% - Subclasses the MATLAB handle class which means that pass by reference semantics
%   apply.
% - experimental code.
%
% See also Arbotix, RobotArm.

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

classdef Machine < handle

    properties
        debug;
    end

    methods (Abstract)
        connect(m, varargin)
        disconnect(m)
        p = getpos(m)
        setpos(m, varargin)
    end
    
    methods
        function m = Machine(varargin)
        end
    end
end
