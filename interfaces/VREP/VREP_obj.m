%VREP_obj V-REP mirror of simple object
%
% Mirror objects are MATLAB objects that reflect objects in the V-REP
% environment.  Methods allow the V-REP state to be examined or changed.
%
% This is a concrete class, derived from VREP_mirror, for all V-REP objects 
% and allows access to pose and object parameters.
%
% Example::
%          vrep = VREP();
%          bill = vrep.object('Bill');  % get the human figure Bill
%          bill.setpos([1,2,0]);
%          bill.setorient([0 pi/2 0]);
%
% Methods throw exception if an error occurs.
%
% Methods::
%
%  getpos              get position of object 
%  setpos              set position of object 
%  getorient           get orientation of object
%  setorient           set orientation of object
%  getpose             get pose of object
%  setpose             set pose of object
%
% Superclass methods (VREP_mirror)::
%  getname          get object name
%-
%  setparam_bool    set object boolean parameter
%  setparam_int     set object integer parameter
%  setparam_float   set object float parameter
%-
%  getparam_bool    get object boolean parameter
%  getparam_int     get object integer parameter
%  getparam_float   get object float parameter
%-
%  display             print the link parameters in human readable form
%  char                convert to string
%
% See also VREP_mirror, VREP_obj, VREP_arm, VREP_camera, VREP_hokuyo.

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

classdef VREP_obj < VREP_mirror
    
    properties
    end
    
    methods
        
        function obj = VREP_obj(vrep, varargin)
            %VREP_obj.VREP_obj VREP_obj mirror object constructor
            %
            % v = VREP_base(NAME) creates a V-REP mirror object for a
            % simple V-REP object type.
            obj = obj@VREP_mirror(vrep, varargin{:});
        end
        
        function p = getpos(obj, relto)
            %VREP_obj.getpos Get position of V-REP object
            %
            % V.getpos() is the position (1x3) of the corresponding V-REP object.
            %
            % V.getpos(BASE) as above but position is relative to the VREP_obj
            % object BASE.
            %
            % See also VREP_obj.setpos, VREP_obj.getorient, VREP_obj.getpose.
            if nargin == 1
                p = obj.vrep.getpos(obj.h);
            elseif nargin == 2 && isa(relto, 'VREP_base')
                p = obj.vrep.getpos(obj.h, relto.h);
            end
        end
        
        function setpos(obj, p, relto)
            %VREP_obj.setpos Set position of V-REP object
            %
            % V.setpos(T) sets the position of the corresponding V-REP object
            % to T (1x3).
            %
            % V.setpos(T, BASE) as above but position is set relative to the
            % position of the VREP_obj object BASE.
            %
            % See also VREP_obj.getpos, VREP_obj.setorient, VREP_obj.setpose.
            if nargin == 2
                obj.vrep.setpos(obj.h, p);
            elseif nargin == 3 && isa(relto, 'VREP_mirror')
                obj.vrep.setpos(obj.h, p, relto.h);
            end
        end
        
        function p = getorient(obj, varargin)
            %VREP_obj.getorient Get orientation of V-REP object
            %
            % V.getorient() is the orientation of the corresponding V-REP
            % object as a rotation matrix (3x3).
            %
            % V.getorient('euler', OPTIONS) as above but returns ZYZ Euler
            % angles.
            %
            % V.getorient(BASE) is the orientation of the corresponding V-REP
            % object relative to the VREP_obj object BASE.
            %
            % V.getorient(BASE, 'euler', OPTIONS) as above but returns ZYZ Euler
            % angles.
            %
            % Options::
            %   See tr2eul.
            %
            % See also VREP_obj.setorient, VREP_obj.getopos, VREP_obj.getpose.
            if nargin == 1
                p = obj.vrep.getorient(obj.h);
            else
                if isa(varargin{1}, 'VREP_base')
                    p = obj.vrep.getorient(obj.h, relto.h, varargin{2:end});
                else
                    p = obj.vrep.getorient(obj.h, relto.h, varargin{:});
                end
            end
        end
        
        function setorient(obj, p, relto)
            %VREP_obj.setorient Set orientation of V-REP object
            %
            % V.setorient(R) sets the orientation of the corresponding V-REP to rotation matrix R (3x3).
            %
            % V.setorient(T) sets the orientation of the corresponding V-REP object to rotational component of homogeneous transformation matrix
            % T (4x4).
            %
            % V.setorient(E) sets the orientation of the corresponding V-REP object to ZYZ Euler angles (1x3).
            %
            % V.setorient(X, BASE) as above but orientation is set relative to the
            % orientation of VREP_obj object BASE.
            %
            % See also VREP_obj.getorient, VREP_obj.setpos, VREP_obj.setpose.
            if nargin == 2
                obj.vrep.setorient(obj.h, p);
                
            elseif nargin == 3 && isa(relto, 'VREP_base')
                obj.vrep.setorient(obj.h, p, relto.h);
            end
        end
        
        function p = getpose(obj, relto)
            %VREP_obj.getpose Get pose of V-REP object
            %
            % V.getpose() is the pose (4x4) of the the corresponding V-REP object.
            %
            % V.getpose(BASE) as above but pose is relative to the
            % pose the VREP_obj object BASE.
            %
            % See also VREP_obj.setpose, VREP_obj.getorient, VREP_obj.getpos.
            if nargin == 1
                p = obj.vrep.getpose(obj.h);
            elseif nargin == 2 && isa(relto, 'VREP_base')
                p = obj.vrep.getpose(obj.h, relto.h);
            end
        end
        
        function setpose(obj, p, relto)
            %VREP_obj.setpose Set pose of V-REP object
            %
            % V.setpose(T) sets the pose of the corresponding V-REP object
            % to T (4x4).
            %
            % V.setpose(T, BASE) as above but pose is set relative to the
            % pose of the VREP_obj object BASE.
            %
            % See also VREP_obj.getpose, VREP_obj.setorient, VREP_obj.setpos.
            if nargin == 2
                obj.vrep.setpose(obj.h, p);
            elseif nargin == 3 && isa(relto, 'VREP_base')
                obj.vrep.setpose(obj.h, p, relto.h);
            end
        end
    end
end
    
