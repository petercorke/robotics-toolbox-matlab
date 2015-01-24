%VREP V-REP simulator communications object
%
% A VREP object holds all information related to the state of a connection
% to an instance of the V-REP simulator running on this or a networked
% computer. Allows the creation of references to other objects/models in
% V-REP which can be manipulated in MATLAB.
%
% This class handles the interface to the simulator and low-level object
% handle operations.
%
% Methods throw exception if an error occurs.
%
% Methods::
%
%  gethandle     get handle to named object
%  getchildren   get children belonging to handle
%  getobjname    get names of objects
%-
%  object              return a VREP_obj object for named object
%  arm                 return a VREP_arm object for named robot
%  camera              return a VREP_camera object for named vosion sensor
%  hokuyo              return a VREP_hokuyo object for named Hokuyo scanner
%-
%  getpos              return position of object given handle
%  setpos              set position of object given handle
%  getorient           return orientation of object given handle
%  setorient           set orientation of object given handle
%  getpose             return pose of object given handle
%  setpose             set pose of object given handle
%-
%  setobjparam_bool    set object boolean parameter
%  setobjparam_int     set object integer parameter
%  setobjparam_float   set object float parameter
%  getobjparam_bool    get object boolean parameter
%  getobjparam_int     get object integer parameter
%  getobjparam_float   get object float parameter
%-
%  signal_int          send named integer signal
%  signal_float        send named float signal
%  signal_str          send named string signal
%-
%  setparam_bool       set simulator boolean parameter
%  setparam_int        set simulator integer parameter
%  setparam_str        set simulator string parameter
%  setparam_float      set simulator float parameter
%  getparam_bool       get simulator boolean parameter
%  getparam_int        get simulator integer parameter
%  getparam_str        get simulator string parameter
%  getparam_float      get simulator float parameter
%-
%  delete              shutdown the connection and cleanup
%-
%  simstart            start the simulator running
%  simstop             stop the simulator running
%  simpause            pause the simulator
%  getversion          get V-REP version number
%  checkcomms          return status of connection
%  pausecomms          pause the comms
%-
%  loadscene           load a scene file
%  clearscene          clear the current scene
%  loadmodel           load a model into current scene
%-
%  display             print the link parameters in human readable form
%  char                convert to string
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

classdef VREP < handle
        
    properties(GetAccess=public, SetAccess=protected)
        vrep    % the remApi object
        client  % the comms handle
        path    % path to V-REP root
        mode    % the communications mode
        libpath  % list of libraries added to path
        version  % the version as given by user 304, 311 etc.
    end
    
    
    methods
    
        function obj = VREP(varargin)
            %VREP.VREP VREP object constructor
            %
            % v = VREP(OPTIONS) create a connection to an instance of the V-REP
            % simulator.
            %
            % Options::
            % 'timeout',T     Timeout T in ms (default 2000)
            % 'cycle',C       Cycle time C in ms (default 5)
            % 'port',P        Override communications port
            % 'reconnect'     Reconnect on error (default noreconnect)
            % 'path',P        The path to VREP install directory
            %
            % Notes::
            % - The default path is taken from the environment variable VREP
            
            opt.timeout = 2000;
            opt.cycle = 5;
            opt.port = 19997;
            opt.reconnect = false;
            opt.path = getenv('VREP');

            [opt,args] = tb_optparse(opt, varargin);
                        
            
            % setup paths
            if isempty(opt.path)
                error('RTB:VREP:badargs', 'no VREP path specified');
            end
            
            if ~exist(opt.path, 'dir')
                error('RTB:VREP:badargs', 'Root VREP folder %s does not exist', opt.path);
            end
            obj.path = opt.path;
            
            
            % % for 3.0.4
            %   libpath = { fullfile(path, 'programming', 'remoteApi')
            %   fullfile(path, 'programming', 'Matlab')
            %  fullfile(path, 'programming', 'remoteApiSharedLib')
            %  };
            %  port = 19998;
            
            % for 3.1.x
            libpath = { fullfile(opt.path, 'programming', 'remoteApi')
                fullfile(opt.path, 'programming', 'remoteApiBindings', 'matlab', 'matlab')
                fullfile(opt.path, 'programming', 'remoteApiBindings', 'lib', 'lib')
                };
            
            addpath( libpath{:} );
            
            obj.libpath = libpath;
            
            obj.vrep = remApi('remoteApi','extApi.h');

            % IP address and port
            % wait until connected
            % do not attempt to reconnect
            % timeout is 2000ms
            % comms cycle time is 5ms
            
            
            % establish the connection
            obj.client = obj.vrep.simxStart('127.0.0.1', opt.port, true, ...
                ~opt.reconnect, opt.timeout, opt.cycle);
            
            if obj.client < 0
                error('RTB:VREP:noconnect', 'Can''t connect to V-REP simulator');
            end
            
            obj.mode = obj.vrep.simx_opmode_oneshot_wait;
            
            % get actual version number as a string
            vint = obj.getversion();
            v = '';
            for i=1:3
                v = [num2str(rem(vint,100)) v];
                vint = floor(vint/100);
                if i < 3
                    v = ['.' v];
                end
            end
            obj.version = v;
        end
        
        function display(v)
            %VREP.display Display parameters
            %
            % V.display() displays the VREP parameters in compact format.
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
            disp( char(v) );
        end % display()
        
        function s = char(obj)
            %VREP.char Convert to string
            %
            % V.char() is a string representation the VREP parameters in human
            % readable foramt.
            %
            % See also VREP.display.

            s = sprintf('V-REP robotic simulator interface (active=%d)', obj.checkcomms() );

            s = strvcat(s, sprintf('path: %s (ver %s)', obj.path, obj.version));
            switch obj.mode
                case obj.vrep.simx_opmode_oneshot
                    s = strvcat(s, 'mode: simx_opmode_oneshot (non-blocking)');
                case obj.vrep.simx_opmode_oneshot_wait
                    s = strvcat(s, 'mode: simx_opmode_oneshot_wait (blocking)');
                case obj.vrep.simx_opmode_streaming
                    s = strvcat(s, 'mode: simx_opmode_streaming (non-blocking)');
            end
        end
        
        function delete(obj)
            %VREP.delete VREP object destructor
            %
            % delete(v) closes the connection to the V-REP simulator
            
            disp('--VREP destructor called');
            obj.vrep.simxFinish(obj.client);
            obj.vrep.delete(); % explicitly call the destructor!
            obj.vrep = [];
            
            rmpath( obj.libpath{:} );
        end
        
        %---- wrapper functions for getting object handles
        function [h,s] = gethandle(obj, fmt, varargin)
            %VREP.gethandle Return handle to VREP object
            %
            % H = V.gethandle(NAME) is an integer handle for named V-REP object.
            %
            % H = V.gethandle(FMT, ARGLIST) as above but the name is formed
            % from sprintf(FMT, ARGLIST).
            %
            % See also sprintf.
            [s,h] = obj.vrep.simxGetObjectHandle(obj.client, sprintf(fmt, varargin{:}), obj.mode);
        end
        
        function children = getchildren(obj, h)
            %VREP.getchildren Find children of object
            %
            % C = V.getchildren(H) is a vector of integer handles for the children of
            % the V-REP object denoted by the integer handle H.
            i = 0;
            while true
            [s,ch] = obj.vrep.simxGetObjectChild(obj.client, h, i, obj.mode);
            if s < 0
                break
            end
            i = i+1;
            children(i) = ch;
            end
        end
        
        function out = getobjname(obj, h)
            %VREP.getname Find names of objects
            %
            % V.getobjname() will display the names and object handle (integers) for
            % all objects in the current scene.
            %
            % NAME = V.getobjname(H) will return the name of the object with handle H.

            [s,handles,~,~,strd] = obj.vrep.simxGetObjectGroupData(obj.client, obj.vrep.sim_appobj_object_type, 0, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
            
            if nargin < 2
                name = strd(:,1);
            else
                name = strd{handles == h,1};
            end
            
            if (nargout == 0) && (nargin < 2)
                for i=1:length(name)
                    fprintf('%3d: %s\n', handles(i), strd{i});
                    return
                end
            end
            out = name;
        end
        
        % HOW DO I GET OBJECT TYPE FROM V-REP??
%         		% Scene object types
%  		sim_object_shape_type           =0;
% 		sim_object_joint_type           =1;
% 		sim_object_graph_type           =2;
% 		sim_object_camera_type          =3;
% 		sim_object_dummy_type           =4;
% 		sim_object_proximitysensor_type =5;
% 		sim_object_reserved1            =6;
% 		sim_object_reserved2            =7;
% 		sim_object_path_type            =8;
% 		sim_object_visionsensor_type    =9;
% 		sim_object_volume_type          =10;
% 		sim_object_mill_type            =11;
% 		sim_object_forcesensor_type     =12;
% 		sim_object_light_type           =13;
% 		sim_object_mirror_type          =14;
        
        %---- simulation control
        function simstart(obj)
            %VREP.simstart Start V-REP simulation
            %
            % V.simstart() starts the V-REP simulation engine.
            %
            % See also VREP.simstop, VREP.simpause.
            s = obj.vrep.simxStartSimulation(obj.client, obj.mode);
        end
        
        function simstop(obj)
            %VREP.simstop Stop V-REP simulation
            %
            % V.simstop() stops the V-REP simulation engine.
            %
            % See also VREP.simstart.
            s = obj.vrep.simxStopSimulation(obj.client, obj.mode);
        end
        
        function simpause(obj)
            %VREP.pause Pause V-REP simulation
            %
            % V.simpause() pauses the V-REP simulation engine.  Use
            % V.simstart() to resume the simulation.
            %
            % See also VREP.simstart.
            s = obj.vrep.simxPauseSimulation(obj.client, obj.mode);
        end
        
        function v = getversion(obj)
            %VREP.getversion Get version of the V-REP simulator
            %
            % V.getversion() is the version of the V-REP simulator
            % server as an integer MNNNN where M is the major version
            % number and NNNN is the minor version number.
            [s,v] = obj.vrep.simxGetIntegerParameter(obj.client, obj.vrep.sim_intparam_program_version, obj.mode);
        end

        function s = checkcomms(obj)
            %VREP.checkcomms Check communications to V-REP simulator
            %
            % V.checkcomms() is true if a valid connection to the V-REP simulator exists.
            s = obj.vrep.simxGetConnectionId(obj.client);
        end
        
        function pausecomms(obj, onoff)
            %VREP.pausecomms Pause communcations to the V-REP simulator
            %
            % V.pausecomms(P) pauses communications to the V-REP simulation engine if P is true
            % else resumes it.  Useful to ensure an atomic update of
            % simulator state.
            
            s = obj.vrep.simxPauseCommunication(obj.client, onoff, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end
        
        function loadscene(obj, name, varargin)
            %VREP.loadscene Load a scene into the V-REP simulator
            %
            % V.loadscene(FILE, OPTIONS) loads the scene file FILE with extension .ttt
            % into the simulator.
            %
            % Options::
            % 'local'   The file is loaded relative to the MATLAB client's current
            %           folder, otherwise from the V-REP root folder.
            %
            % Example::
            %          vrep.loadscene('2IndustrialRobots');
            %
            % Notes::
            % - If a relative filename is given in non-local (server) mode it is
            %   relative to the V-REP scenes folder.
            %
            % See also VREP.clearscene.

            opt.local = false;
            
            opt = tb_optparse(opt, varargin);
            
            if ~opt.local
                if name(1) ~= '/'
                    % its a relative path, add in full path
                    name = fullfile(obj.path, 'scenes', [name '.ttt']);
                end
            end
            s = obj.vrep.simxLoadScene(obj.client, name, opt.local, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end
        
        function clearscene(obj, name, varargin)
            %VREP.clearscene Clear current scene in the V-REP simulator
            %
            % V.clearscene() clears the current scene and switches to another open
            % scene, if none, a new (default) scene is created.
            %
            % See also VREP.loadscene.

            s = obj.vrep.simxCloseScene(obj.client, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end
        
        function m = loadmodel(obj, model, varargin)
            %VREP.loadmodel Load a model into the V-REP simulator
            %
            % M = V.loadmodel(FILE, OPTIONS) loads the model file FILE with extension .ttm
            % into the simulator and returns a VREP_obj object that mirrors it in
            % MATLAB.
            %
            % Options::
            % 'local'   The file is loaded relative to the MATLAB client's current
            %           folder, otherwise from the V-REP root folder.
            %
            % Example::
            %          vrep.loadmodel('people/Walking Bill');
            %
            % Notes::
            % - If a relative filename is given in non-local (server) mode it is
            %   relative to the V-REP models folder.
            %
            % See also VREP.arm, VREP.camera, VREP.object.

            opt.local = false;
            opt.name = [];
            opt = tb_optparse(opt, varargin);
            
            if ~opt.local
                if model(1) ~= '/'
                    % its a relative path, add in full path
                    model = fullfile(obj.path, 'models', [model '.ttm']);
                end
            end
            [s,bh] = obj.vrep.simxLoadModel(obj.client, model, opt.local, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
            m = obj.object(bh, varargin{:});
            pause(0.5);
        end
        
        
        %---- signals

        function signal_int(obj, name, val)
            %VREP.signal_int Send an integer signal to the V-REP simulator
            %
            % V.signal_int(NAME, VAL) send an integer signal with name NAME
            % and value VAL to the V-REP simulation engine.
            s = obj.vrep.simxSetIntegerSignal(obj.client, name, val, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end            
        end
        
        function signal_float(obj, name, val)
            %VREP.signal_float Send a float signal to the V-REP simulator
            %
            % V.signal_float(NAME, VAL) send a float signal with name NAME
            % and value VAL to the V-REP simulation engine.
            s = obj.vrep.simxSetFloatSignal(obj.client, name, val, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end
        
        function signal_str(obj, name, val)
            %VREP.signal_str Send a string signal to the V-REP simulator
            %
            % V.signal_str(NAME, VAL) send a string signal with name NAME
            % and value VAL to the V-REP simulation engine.
            s = obj.vrep.simxSetStringSignal(obj.client, name, val, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end
        
        %---- set/get simulator parameters
        function setparam_bool(obj, paramid, val)
            %VREP.setparam_bool Set boolean parameter of the V-REP simulator
            %
            % V.setparam_bool(NAME, VAL) sets the boolean parameter with name NAME
            % to value VAL within the V-REP simulation engine.
            %
            % See also VREP.getparam_bool.
            if isstr(paramid)
                paramid = eval(['obj.vrep.' paramid]);
            end
            s = obj.vrep.simxSetBooleanParameter(obj.client, paramid, val, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end

        function setparam_int(obj, paramid, val)
            %VREP.setparam_int Set integer parameter of the V-REP simulator
            %
            % V.setparam_int(NAME, VAL) sets the integer parameter with name NAME
            % to value VAL within the V-REP simulation engine.
            %
            % See also VREP.getparam_int.
            if isstr(paramid)
                paramid = eval(['obj.vrep.' paramid]);
            end
            s = obj.vrep.simxSetIntegerParameter(obj.client, paramid, val, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end
        
        function setparam_str(obj, paramid, val)
            %VREP.setparam_str Set string parameter of the V-REP simulator
            %
            % V.setparam_str(NAME, VAL) sets the integer parameter with name NAME
            % to value VAL within the V-REP simulation engine.
            %
            % See also VREP.getparam_str.
            if isstr(paramid)
                paramid = eval(['obj.vrep.' paramid]);
            end
            s = obj.vrep.simxSetStringParameter(obj.client, paramid, val, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end
        
        function setparam_float(obj, paramid, val)
            %VREP.setparam_float Set float parameter of the V-REP simulator
            %
            % V.setparam_float(NAME, VAL) sets the float parameter with name NAME
            % to value VAL within the V-REP simulation engine.
            %
            % See also VREP.getparam_float.
            if isstr(paramid)
                paramid = eval(['obj.vrep.' paramid]);
            end
            s = obj.vrep.simxSetFloatingParameter(obj.client, paramid, val, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end
        
        function v = getparam_bool(obj, paramid)
            %VREP.getparam_bool Get boolean parameter of the V-REP simulator
            %
            % V.getparam_bool(NAME) is the boolean parameter with name NAME
            % from the V-REP simulation engine.
            %
            % Example::
            %         v = VREP();
            %         v.getparam_bool('sim_boolparam_mirrors_enabled')
            %
            % See also VREP.setparam_bool.
            if isstr(paramid)
                paramid = eval(['obj.vrep.' paramid]);
            end
            [s,v] = obj.vrep.simxGetBooleanParameter(obj.client, paramid, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end

        function v = getparam_int(obj, paramid)
            %VREP.getparam_int Get integer parameter of the V-REP simulator
            %
            % V.getparam_int(NAME) is the integer parameter with name NAME
            % from the V-REP simulation engine.
            %
            % Example::
            %         v = VREP();
            %         v.getparam_int('sim_intparam_settings')
            %
            % See also VREP.setparam_int.
            
            if isstr(paramid)
                paramid = eval(['obj.vrep.' paramid]);
            end
            [s,v] = obj.vrep.simxGetIntegerParameter(obj.client, paramid, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end

        function v = getparam_str(obj, paramid)
            %VREP.getparam_str Get string parameter of the V-REP simulator
            %
            % V.getparam_str(NAME) is the string parameter with name NAME
            % from the V-REP simulation engine.
            %
            % Example::
            %         v = VREP();
            %         v.getparam_str('sim_stringparam_application_path')
            %
            % See also VREP.setparam_str.
            
            if isstr(paramid)
                paramid = eval(['obj.vrep.' paramid]);
            end
            [s,v] = obj.vrep.simxGetStringParameter(obj.client, paramid, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end 
        
        function v = getparam_float(obj, paramid)
            %VREP.getparam_float Get float parameter of the V-REP simulator
            %
            % V.getparam_float(NAME) gets the float parameter with name NAME
            % from the V-REP simulation engine.
            %
            % Example::
            %         v = VREP();
            %         v.getparam_float('sim_floatparam_simulation_time_step')
            %
            % See also VREP.setparam_float.
            if isstr(paramid)
                paramid = eval(['obj.vrep.' paramid]);
            end
            [s,v] = obj.vrep.simxGetFloatingParameter(obj.client, paramid, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end
        
        %---- set/get object parameters
        function setobjparam_bool(obj, h, paramid, val)
            %VREP.setobjparam_bool Set boolean parameter of a V-REP object
            %
            % V.setobjparam_bool(H, PARAM, VAL) sets the boolean parameter
            % with identifier PARAM of object H to value VAL.
            s = obj.vrep.simxSetObjectIntParameter(obj.client, h, paramid, val, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end

        function setobjparam_int(obj, h, paramid, val)
            %VREP.setobjparam_int Set Integer parameter of a V-REP object
            %
            % V.setobjparam_int(H, PARAM, VAL) sets the integer parameter
            % with identifier PARAM of object H to value VAL.
            s = obj.vrep.simxSetObjectIntParameter(obj.client, h, paramid, val, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end

        function setobjparam_float(obj, h, paramid, val)
            %VREP.setobjparam_float Set float parameter of a V-REP object
            %
            % V.setobjparam_float(H, PARAM, VAL) sets the float parameter
            % with identifier PARAM of object H to value VAL.
            s = obj.vrep.simxSetObjectFloatParameter(obj.client, h, paramid, val, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end
        
        function val = getobjparam_bool(obj, h, paramid)
            %VREP.getobjparam_bool Get boolean parameter of a V-REP object
            %
            % V.getobjparam_bool(H, PARAM) gets the boolean parameter
            % with identifier PARAM of object with integer handle H.
            [s,val] = obj.vrep.simxGetObjectIntParameter(obj.client, h, paramid, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end

        function val = getobjparam_int(obj, h, paramid)
            %VREP.getobjparam_int Get integer parameter of a V-REP object
            %
            % V.getobjparam_int(H, PARAM) gets the integer parameter
            % with identifier PARAM of object with integer handle H.
            [s,val] = obj.vrep.simxGetObjectIntParameter(obj.client, h, paramid, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end

        function val = getobjparam_float(obj, h, paramid)
            %VREP.getobjparam_float Get float parameter of a V-REP object
            %
            % V.getobjparam_float(H, PARAM) gets the float parameter
            % with identifier PARAM of object with integer handle H.
            [s,val] = obj.vrep.simxGetObjectFloatParameter(obj.client, h, paramid, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end
        
        %---- wrapper functions for using joint objects    
        function setjoint(obj, h, q)
            %VREP.setjoint Set value of V-REP joint object
            %
            % V.setjoint(H, Q) sets the position of joint object with integer handle H
            % to the value Q.
            s = obj.vrep.simxSetJointPosition(obj.client, h, q, obj.mode);
        end
        
        function q = getjoint(obj, h)
            %VREP.getjoint Get value of V-REP joint object
            %
            % V.getjoint(H, Q) is the position of joint object with integer handle H.
            [s,q] = obj.vrep.simxGetJointPosition(obj.client, h, obj.mode);
            
            if s ~= 0
                throw( obj.except(s) );
            end
        end
        
        function setjointtarget(obj, h, q)
            %VREP.setjointtarget Set target value of V-REP joint object
            %
            % V.setjointtarget(H, Q) sets the target position of joint object with integer handle H
            % to the value Q.
            s = obj.vrep.simxSetJointTargetPosition(obj.client, h, q, obj.mode);
            
            if s ~= 0
                throw( obj.except(s) );
            end
        end
        
        function setjointvel(obj, h, qd)
            %VREP.setjointvel Set velocity of V-REP joint object
            %
            % V.setjointvel(H, QD) sets the target velocity of joint object with integer handle H
            % to the value QD.
            s = obj.vrep.simxSetJointTargetVelocity(obj.client, h, qd, obj.mode);
            
            if s ~= 0
                throw( obj.except(s) );
            end
        end
        

        %---- wrapper functions for position of objects
        function setpos(obj, h, t, relto)
            %VREP.setpos Set position of V-REP object             
            %
            % V.setpos(H, T) sets the position of V-REP object with integer
            % handle H to T (1x3).
            %
            % V.setpos(H, T, HR) as above but position is set relative to the
            % position of object with integer handle HR.
            %
            % See also VREP.getpos, VREP.setpose, VREP.setorient.

            if nargin < 4
                relto = -1;
            end
            
            s = obj.vrep.simxSetObjectPosition(obj.client, h, relto, t, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end
        
        function t = getpos(obj, h, relto)
            %VREP.getpos Get position of V-REP object             
            %
            % V.getpos(H) is the position (1x3) of the V-REP object with integer
            % handle H.
            %
            % V.getpos(H, HR) as above but position is relative to the
            % position of object with integer handle HR.
            %
            % See also VREP.setpose, VREP.getpose, VREP.getorient.
            if nargin < 4
                relto = -1;
            end
            
            [s,t] = obj.vrep.simxGetObjectPosition(obj.client, h, relto, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end
        
       %---- wrapper functions for pose of objects

        function setpose(obj, h, T, relto)
            %VREP.setpose Set pose of V-REP object             
            %
            % V.setpos(H, T) sets the pose of V-REP object with integer
            % handle H according to homogeneous transform T (4x4).
            %
            % V.setpos(H, T, HR) as above but pose is set relative to the
            % pose of object with integer handle HR.
            %
            % See also VREP.getpose, VREP.setpos, VREP.setorient.

            if nargin < 4
                relto = -1;
            elseif isa(relto, 'VREP_base')
                relto = relto.h;
            end
            
            pos = transl(T);
            eul = tr2eul(T, 'deg');
            s = obj.vrep.simxSetObjectPosition(obj.client, h, relto, pos, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
            s = obj.vrep.simxSetObjectOrientation(obj.client, h, relto, eul, obj.mode);
            if s < 0
                throw( obj.except(s) );
            end
        end
        
        function T = getpose(obj, h, relto)
            %VREP.getpose Get pose of V-REP object             
            %
            % T = V.getpose(H) is the pose of the V-REP object with integer
            % handle H as a homogeneous transformation matrix (4x4).
            %
            % T = V.getpose(H, HR) as above but pose is relative to the
            % pose of object with integer handle R.
            %
            % See also VREP.setpose, VREP.getpos, VREP.getorient.
            if nargin < 4
                relto = -1;
            elseif isa(relto, 'VREP_base')
                relto = relto.h;
            end
            
            [s,pos] = obj.vrep.simxGetObjectPosition(obj.client, h, relto, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
            [s,eul] = obj.vrep.simxGetObjectOrientation(obj.client, h, relto, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
            
            T = transl(pos) * eul2tr(eul, 'deg');
        end
        
        %---- wrapper functions for orientation of objects

        function setorient(obj, h, R, relto)
            %VREP.setorient Set orientation of V-REP object             
            %
            % V.setorient(H, R) sets the orientation of V-REP object with integer
            % handle H to that given by rotation matrix R (3x3).
            %
            % V.setorient(H, T) sets the orientation of V-REP object with integer
            % handle H to rotational component of homogeneous transformation matrix
            % T (4x4).
            %
            % V.setorient(H, E) sets the orientation of V-REP object with integer
            % handle H to ZYZ Euler angles (1x3).
            %
            % V.setorient(H, X, HR) as above but orientation is set relative to the
            % orientation of object with integer handle HR.
            %
            % See also VREP.getorient, VREP.setpos, VREP.setpose.

            if nargin < 4
                relto = -1;
            elseif isa(relto, 'VREP_base')
                relto = relto.h;
            end
            
            if isrot(R) || ishomog(R)
                eul = tr2eul(R);
            elseif isvec(R, 3)
                eul = R;
            end
            
            s = obj.vrep.simxSetObjectOrientation(obj.client, h, relto, eul, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end
        
        function R = getorient(obj, h, relto, varargin)
            %VREP.getorient Get orientation of V-REP object             
            %
            % R = V.getorient(H) is the orientation  of the V-REP object with integer
            % handle H as a rotation matrix (3x3).
            %
            % EUL = V.getorient(H, 'euler', OPTIONS) as above but returns ZYZ Euler
            % angles.
            %
            % V.getorient(H, HRR) as above but orientation is relative to the
            % position of object with integer handle HR.
            %
            % V.getorient(H, HRR, 'euler', OPTIONS) as above but returns ZYZ Euler
            % angles.
            %
            % Options::
            % See tr2eul.
            %
            % See also VREP.setorient, VREP.getpos, VREP.getpose.
            
            if nargin < 4
                relto = -1;
            elseif isa(relto, 'VREP_base')
                relto = relto.h;
            end
            
            opt.euler = false;
            [opt,args] = tb_optparse(opt, varargin);
            
            [s,eul] = obj.vrep.simxGetObjectOrientation(obj.client, h, relto, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
            
            if opt.euler
                R = eul;
            else
                R = eul2tr(eul, args{:});
            end
        end
        
        %---- factory methods to create instances of specific objects that mirror VREP objects
        function a = arm(obj, name)
            %VREP.arm Return VREP_arm object
            %
            % V.arm(name) is a factory method that returns a VREP_arm object for the V-REP robot
            % object named NAME.
            %
            % Example::
            %         vrep.arm('IRB 140');
            %
            % See also VREP_arm.
            
            if isa(name, 'VREP_obj')
                h = name.h;
                name = name.getname();
            end
            a = VREP_arm(obj, name);
        end
        
        function o = object(obj, varargin)
            %VREP.arm Return VREP_obj object
            %
            % V.objet(name) is a factory method that returns a VREP_obj object for the V-REP
            % object or model named NAME.
            % Example::
            %         vrep.obj('Walking Bill');
            %
            % See also VREP_obj.
            o = VREP_obj(obj, varargin{:});
        end
        
        function m = mobile(obj, name)
            %VREP.mobile Return VREP_mobile object
            %
            % V.mobile(name) is a factory method that returns a
            % VREP_mobile object for the V-REP mobile base object named NAME.
            %
            % See also VREP_mobile.
            m = VREP_mobile(obj, name);
        end
        
        function m = camera(obj, name)
            %VREP.camera Return VREP_camera object
            %
            % V.camera(name) is a factory method that returns a VREP_camera object for the V-REP vision
            % sensor object named NAME.
            %
            % See also VREP_camera.
            m = VREP_camera(obj, name);
        end
        
        function m = hokuyo(obj, name)
            %VREP.hokuyo Return VREP_hokuyo object
            %
            % V.hokuyo(name) is a factory method that returns a VREP_hokuyo 
            % object for the V-REP Hokuyo laser scanner object named NAME.
            %
            % See also VREP_hokuyo.
            m = VREP_mobile(obj, name);
        end
        
        function m = youbot(obj, name)
            %VREP.youbot Return VREP_youbot object
            %
            % V.youbot(name) is a factory method that returns a VREP_youbot
            % object for the V-REP YouBot object named NAME.
            %
            % See also VREP_youbot.
            m = VREP_youbot(obj, name);
        end
    end
    methods(Access=private)
        function e = except(obj, s)
            switch (s)
                case obj.vrep.simx_return_novalue_flag  %1
                    msgid = 'VREP:noreply';
                    err = 'No command reply in the input buffer';
                case obj.vrep.simx_return_timeout_flag %2
                    msgid = 'VREP:timedout';
                    err = 'function timed out (probably the network is down or too slow)';
                case obj.vrep.simx_return_illegal_opmode_flag %4
                    msgid = 'VREP:notsupported';
                    err = 'The specified operation mode is not supported for the given function';
                case obj.vrep.simx_return_remote_error_flag %8
                    msgid = 'VREP:serversideerrror';
                    err = 'function caused an error on the server side (e.g. an invalid handle was specified)';
                case obj.vrep.simx_return_split_progress_flag %16
                    msgid = 'VREP:busy';
                    err = 'The communication thread is still processing previous split command of the same type';
                case obj.vrep.simx_return_local_error_flag %32
                    msgid = 'VREP:clientsideerror';
                    err = 'The function caused an error on the client side';
                case obj.vrep.simx_return_initialize_error_flag %64
                    msgid = 'VREP:nostarted';
                    err = 'simxStart was not yet called'
            end
            
            e = MException(msgid, err);
        end
end
end
