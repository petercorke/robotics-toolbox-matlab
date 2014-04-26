%VREP_mirror V-REP mirror object class
%
% Mirror objects are MATLAB objects that reflect objects in the V-REP
% environment.  Methods allow the V-REP state to be examined or changed.
%
% This abstract class is the root class for all V-REP mirror objects.
%
% Methods throw exception if an error occurs.
%
% Methods::
%
%  setobjparam_bool    set object boolean parameter
%  setobjparam_int     set object integer parameter
%  setobjparam_float   set object float parameter
%
% See also VREP_obj, VREP_arm, VREP_camera, VREP_hokuyo.

% Copyright (C) 1993-2014, by Peter I. Corke
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

classdef (Abstract=true) VREP_mirror < handle
    
    properties(GetAccess=public, SetAccess=protected)
        vrep
        h
        name
    end

            
        methods
            
            function obj = VREP_mirror(vrep, name)
            %VREP_mirror.VREP_mirror VREP_mirror object constructor
            %
            % v = VREP_mirror(NAME) creates a V-REP mirror object.
                obj.vrep = vrep;
                
                if nargin > 1
                     obj.h = vrep.gethandle(name);
                end
                obj.name = name;
            end
            
            function setobjparam_bool(obj, paramid, val)
                %VREP.setparam_bool Set boolean parameter of V-REP object
                %
                % V.setparam_bool(NAME, VAL) sets the boolean parameter with name NAME
                % to value VAL within the V-REP simulation engine.
                obj.vrep.setobjparam_bool(obj, obj.h, paramid, val);
            end
            function setobjparam_int(obj, paramid, val)
                %VREP.setparam_bool Set integer parameter of V-REP object
                %
                % V.setparam_int(NAME, VAL) sets the integer parameter with name NAME
                % to value VAL within the V-REP simulation engine.
                obj.vrep.setobjparam_int(obj, obj.h, paramid, val);
            end
            function setobjparam_float(obj, paramid, val)
                %VREP.setparam_float Set float parameter of V-REP object
                %
                % V.setparam_float(NAME, VAL) sets the float parameter with name NAME
                % to value VAL within the V-REP simulation engine.
                obj.vrep.setobjparam_float(obj, obj.h, paramid, val);
            end
            
            function val = getobjparam_bool(obj, paramid)
                %VREP.getparam_bool Get boolean parameter of V-REP object
                %
                % V.getparam_bool(NAME, VAL) is the boolean parameter with name NAME
                % of the corresponding V-REP object.
                obj.vrep.setobjparam_bool(obj, obj.h, paramid, val);
            end
            function val = getobjparam_int(obj, paramid, val)
                %VREP.getparam_bool Get integer parameter of V-REP object
                %
                % V.getparam_int(NAME, VAL) is the integer parameter with name NAME
                % of the corresponding V-REP object.
                obj.vrep.setobjparam_int(obj, obj.h, paramid);
            end
            function getobjparam_float(obj, paramid, val)
                %VREP.getparam_float Get float parameter of V-REP object
                %
                % V.getparam_float(NAME, VAL) is the float parameter with name NAME
                % of the corresponding V-REP object.
                val = obj.vrep.setobjparam_float(obj, obj.h, paramid);
            end
        end
end