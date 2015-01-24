%VREP_camera Mirror of V-REP vision sensor object
%
% Mirror objects are MATLAB objects that reflect the state of objects in
% the V-REP environment.  Methods allow the V-REP state to be examined or
% changed.
%
% This is a concrete class, derived from VREP_mirror, for all V-REP vision 
% sensor objects and allows access to images and image parameters.
%
% Methods throw exception if an error occurs.
%
% Example::
%          vrep = VREP();
%          camera = vrep.camera('Vision_sensor');
%          im = camera.grab();
%          camera.setpose(T);
%          R = camera.getorient();
%
% Methods::
%
%  grab               return an image from simulated camera
%  setangle           set field of view
%  setresolution      set image resolution
%  setclipping        set clipping boundaries
%
% Superclass methods (VREP_obj)::
%  getpos              get position of object 
%  setpos              set position of object 
%  getorient           get orientation of object 
%  setorient           set orientation of object
%  getpose             get pose of object
%  setpose             set pose of object
%
% can be used to set/get the pose of the robot base.
%
% Superclass methods (VREP_mirror)::
%  getname          get object name
%-
%  setparam_bool    set object boolean parameter
%  setparam_int     set object integer parameter
%  setparam_float   set object float parameter
%
%  getparam_bool    get object boolean parameter
%  getparam_int     get object integer parameter
%  getparam_float   get object float parameter
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

classdef VREP_camera < VREP_obj
    
    properties
    end
    
    methods
        function obj = VREP_camera(vrep, name)
            %VREP_camera.VREP_camera Create a camera mirror object
            %
            % C = VREP_camera(NAME, OPTIONS) is a mirror object that corresponds to the
            % vision senor named NAME in the V-REP environment.
            %
            % Options::
            % 'fov',A           Specify field of view in degreees (default 60)
            % 'resolution',N    Specify resolution.  If scalar NxN else N(1)xN(2) 
            % 'clipping',Z      Specify near Z(1) and far Z(2) clipping boundaries
            %
            % Notes::
            % - Default parameters are set in the V-REP environmen
            % - Can be applied to "DefaultCamera" which controls the view in the
            %   simulator GUI.
            %
            % See also VREP_obj.
            obj = obj@VREP_obj(vrep, name);
        end
        
        function im = setclipping(obj, near, far)
             %VREP_camera.setclipping  Set clipping boundaries for V-REP vision sensor
             %
             % C.setclipping(NEAR, FAR) set clipping boundaries to the
             % range of Z from NEAR to FAR.  Objects outside this range
             % will not be rendered.
             %
             % See also VREP_camera.getclipping.

             obj.setparam_float(1000, near);
             obj.setparam_float(1001, far);
        end
        
        function clip = getclipping(obj)
             %VREP_camera.getclipping  Get clipping boundaries for V-REP vision sensor
             %
             % C.getclipping() is the near and far clipping boundaries (1x2) in the
             % Z-direction as a 2-vector [NEAR,FAR]. 
             %
             % See also VREP_camera.setclipping.
             
             clip(1) = obj.getparam_float(1000);
             clip(2) = obj.getparam_float(1001);
        end
        
        function im = setangle(obj, angle)
            %VREP_camera.setangle  Set field of view for V-REP vision sensor
            %
            % C.setangle(FOV) set the field-of-view angle to FOV in
            % radians.
            %
            % See also VREP_camera.getangle.
            
            obj.setparam_float(1004, angle);
        end
        
        function fov = getangle(obj)
            %VREP_camera.fetangle  Fet field of view for V-REP vision sensor
            %
            % FOV = C.getangle(FOV) is the field-of-view angle to FOV in
            % radians.
            %
            % See also VREP_camera.setangle.
            
            fov = obj.getparam_float(1004);
        end
        
        function setresolution(obj, res)
            %VREP_camera.setresolution  Set resolution for V-REP vision sensor
            %
            % C.setresolution(R) set image resolution to RxR if R is a scalar or
            % R(1)xR(2) if it is a 2-vector.
            %
            % Notes::
            % - By default V-REP cameras seem to have very low (32x32) resolution.
            % - Frame rate will decrease as frame size increases.
            %
            % See also VREP_camera.getresolution.            
            if isscalar(res)
                rx = res; ry = res;
            else
                rx = res(1); ry = res(2);
            end
            obj.setparam_int(1002, rx);
            obj.setparam_int(1003, ry);
        end
        
        function res = getresolution(obj)
            %VREP_camera.getresolution  Get resolution for V-REP vision sensor
            %
            % R = C.getresolution() is the image resolution (1x2) of the
            % vision sensor R(1)xR(2).
            %
            % See also VREP_camera.setresolution.             
            
            res(1) = obj.getparam_int(1002);
            res(2) = obj.getparam_int(1003);
        end
        
        function im = grab(obj, varargin)
            %VREP_camera.grab  Get image from V-REP vision sensor
            %
            % IM = C.grab(OPTIONS) is an image (WxH) returned from the V-REP
            % vision sensor.
            %
            % C.grab(OPTIONS) as above but the image is displayed using
            % idisp.
            %
            % Options::
            % 'grey'     Return a greyscale image (default color).
            %
            % Notes::
            % - V-REP simulator must be running.
            % - Color images can be quite dark, ensure good light sources.
            % - Uses the signal 'handle_rgb_sensor' to trigger a single
            %   image generation.
            %
            % See also idisp, VREP.simstart.
            
            opt.grey = false;
            opt = tb_optparse(opt, varargin);
            
            % Ask the sensor to turn itself on, take A SINGLE 3D IMAGE, and turn itself off again
            obj.vrep.signal_int('handle_rgb_sensor', 1);
            %fprintf('Capturing image...\n');
            [s,res,image] = obj.vrep.vrep.simxGetVisionSensorImage2(obj.vrep.client, obj.h, opt.grey, obj.vrep.vrep.simx_opmode_oneshot_wait);
            if s ~= 0
                throw( obj.except(s) );
            end
            %fprintf('Captured %dx%d image\n', res(1), res(2));
            
            if nargout == 0
                idisp(image);
            elseif nargout > 0
                im = image;
            end
        end
        
        function s = char(obj)
            %VREP_camera.char Convert to string
            %
            % V.char() is a string representation the VREP parameters in human
            % readable foramt.
            %
            % See also VREP.display.
            
            s = sprintf('VREP camera object: %s', obj.name);
            s = strvcat(s, sprintf('resolution: %d x %d', obj.getresolution()));
            s = strvcat(s, sprintf('clipping: %g to %g', obj.getclipping()));
            
        end
    end
end
