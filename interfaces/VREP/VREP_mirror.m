%VREP_mirror V-REP mirror object class
%
% Mirror objects are MATLAB objects that reflect the state of objects in
% the V-REP environment.  Methods allow the V-REP state to be examined or
% changed.
%
% This abstract class is the root class for all V-REP mirror objects.
%
% Methods throw exception if an error occurs.
%
% Methods::
%  getname          get object name
%-
%  setparam_bool    set object boolean parameter
%  setparam_int     set object integer parameter
%  setparam_float   set object float parameter
%  getparam_bool    get object boolean parameter
%  getparam_int     get object integer parameter
%  getparam_float   get object float parameter
%-
%  remove           remove object from scene
%  display          display object info
%  char             convert to string
%
% Properties (read only)::
%   h                V-REP integer handle for the object
%   name             Name of the object in V-REP
%   vrep             Reference to the V-REP connection object
%
% Notes::
% - This has nothing to do with mirror objects in V-REP itself which are
%   shiny reflective surfaces.
%
% See also VREP_obj, VREP_arm, VREP_camera, VREP_hokuyo.

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

classdef (Abstract=true) VREP_mirror < handle
    
    properties(GetAccess=public, SetAccess=protected)
        vrep
        h
        name
    end

            
        methods
            
            function obj = VREP_mirror(vrep, varargin)
            %VREP_mirror.VREP_mirror Construct VREP_mirror object
            %
            % OBJ = VREP_mirror(NAME) is a V-REP mirror object that represents the
            % object named NAME in the V-REP simulator.
                obj.vrep = vrep;
                
                if nargin > 1
                    if isstr(varargin{1})
                        % passed a string name, convert to handle
                        obj.h = vrep.gethandle(varargin{1});
                        obj.name = varargin{1};
                    else
                        obj.h = varargin{1};
                        if nargin > 2
                            obj.name = varargin{2};
                        else
                            obj.name = [];
                        end
                    end
                end
            end
            
            function name = getname(obj)
                %VREP_mirror.getname Get object name
                %
                % OBJ.getname() is the name of the object in the VREP simulator.
                name = obj.vrep.getobjname(obj.h);
            end

            function setparam_bool(obj, paramid, val)
                %VREP_mirror.setparam_bool Set boolean parameter of V-REP object
                %
                % OBJ.setparam_bool(ID, VAL) sets the boolean parameter with ID
                % to value VAL within the V-REP simulation engine.
                %
                % See also VREP_mirror.getparam_bool, VREP_mirror.setparam_int,
                % VREP_mirror.setparam_float.
                obj.vrep.setobjparam_bool(obj.h, paramid, val);
            end
            function setparam_int(obj, paramid, val)
                %VREP_mirror.setparam_bool Set integer parameter of V-REP object
                %
                % OBJ.setparam_int(ID, VAL) sets the integer parameter with ID
                % to value VAL within the V-REP simulation engine.
                %
                % See also VREP_mirror.getparam_int, VREP_mirror.setparam_bool,
                % VREP_mirror.setparam_float.
                obj.vrep.setobjparam_int(obj.h, paramid, val);
            end
            function setparam_float(obj, paramid, val)
                %VREP_mirror.setparam_float Set float parameter of V-REP object
                %
                % OBJ.setparam_float(ID, VAL) sets the float parameter with ID
                % to value VAL within the V-REP simulation engine.
                %
                % See also VREP_mirror.getparam_float, VREP_mirror.setparam_bool,
                % VREP_mirror.setparam_int.
                obj.vrep.setobjparam_float(obj.h, paramid, val);
            end
            
            function val = getparam_bool(obj, paramid)
                %VREP_mirror.getparam_bool Get boolean parameter of V-REP object
                %
                % OBJ.getparam_bool(ID) is the boolean parameter with ID
                % of the corresponding V-REP object.
                %
                % See also VREP_mirror.setparam_bool, VREP_mirror.getparam_int,
                % VREP_mirror.getparam_float.
                val = obj.vrep.getobjparam_bool(obj, obj.h, paramid, val);
            end
            function val = getparam_int(obj, paramid)
                %VREP_mirror.getparam_int Get integer parameter of V-REP object
                %
                % OBJ.getparam_int(ID) is the integer parameter with ID
                % of the corresponding V-REP object.
                %
                % See also VREP_mirror.setparam_int, VREP_mirror.getparam_bool,
                % VREP_mirror.getparam_float.
                val = obj.vrep.getobjparam_int(obj.h, paramid);
            end
            function val = getparam_float(obj, paramid)
                %VREP_mirror.getparam_float Get float parameter of V-REP object
                %
                % OBJ.getparam_float(ID) is the float parameter with ID
                % of the corresponding V-REP object.
                %
                % See also VREP_mirror.setparam_bool, VREP_mirror.getparam_bool,
                % VREP_mirror.getparam_int.
                val = obj.vrep.getobjparam_float(obj.h, paramid);
            end
            
            function remove(obj)
                s = obj.vrep.vrep.simxRemoveModel(obj.vrep.client, obj.h, obj.vrep.mode);
                if s ~= 0
                    throw( obj.vrep.except(s) );
                end
            end
        
            
            function display(l)
                %VREP_mirror.display Display parameters
                %
                % OBJ.display() displays the VREP parameters in compact format.
                %
                % Notes::
                % - This method is invoked implicitly at the command line when the result
                %   of an expression is a VREP object and the command has no trailing
                %   semicolon.
                %
                % See also VREP.char.
                loose = strcmp( get(0, 'FormatSpacing'), 'loose');
                if loose
                    disp(' ');
                end
                disp([inputname(1), ' = '])
                disp( char(l) );
            end % display()
            
            function s = char(obj)
                %VREP_mirror.char Convert to string
                %
                % OBJ.char() is a string representation the VREP parameters in human
                % readable foramt.
                %
                % See also VREP.display.
                                
                s = sprintf('VREP mirror object: %s', obj.name);
            end
        end
end
