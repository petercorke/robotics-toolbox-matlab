%VREP V-REP simulator communications object
%
% A VREP object holds all information related to the state of a connection.
% References are passed to other objects which mirror the V-REP environment
% in MATLAB.
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
%  setparam_bool       set object boolean parameter
%  setparam_int        set object integer parameter
%  setparam_float      set object float parameter
%-
%  delete              shutdown the connection and cleanup
%-
%  startsim            start the simulator running
%  stopsim             stop the simulator running
%  pausesim            pause the simulator
%  getversion          get V-REP version number
%  checkcomms          return status of connection
%  pausecomms          pause the comms
%-
%  display             print the link parameters in human readable form
%  char                convert to string
%
% See also VREP_obj, VREP_arm, VREP_camera, VREP_hokuyo.

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
    
        function obj = VREP(path, varargin)
            %VREP.VREP VREP object constructor
            %
            % v = VREP(OPTIONS) create a connection to the V-REP simulator.
            %
            % v = VREP(PATH, OPTIONS) as above but specify the root directory
            % of V-REP.
            %
            % Options::
            % 'version',V     Version of V-REP, V=304, 311 etc
            % 'timeout',T     Timeout T in ms (default 2000)
            % 'cycle',C       Cycle time C in ms (default 5)
            % 'port',P        Override communications port
            % 'reconnect'     Reconnect on error (default noreconnect)
            
            opt.version = 304;
            opt.timeout = 2000;
            opt.cycle = 5;
            opt.port = [];
            opt.reconnect = false;
            [opt,args] = tb_optparse(opt, varargin);
                        
            if isempty(args)
                path = '~/Desktop/V-REP_PRO_EDU_V3_0_4_Mac';
            else
                path = args{1};
            end
            
            if ~exist(path, 'dir')
                error('Root VREP folder %s does not exist', path);
            end
            obj.path = path;
            
            switch opt.version
                case 311
                    % for 3.1.1
                    libpath = { fullfile(path, 'programming', 'remoteApi')
                        fullfile(path, 'programming', 'remoteApiBindings', 'matlab', 'matlab')
                        fullfile(path, 'programming', 'remoteApiBindings', 'lib', 'lib')
                        };
                    
                    port = 19997;
                case 304
                    % for 3.0.4
                    libpath = { fullfile(path, 'programming', 'remoteApi')
                    fullfile(path, 'programming', 'Matlab')
                    fullfile(path, 'programming', 'remoteApiSharedLib')
                    };
                    port = 19998;
                otherwise
                    error('don''t know how to handle this version');
            end
            addpath( libpath{:} );
            
            obj.libpath = libpath;
            obj.version = opt.version;
            
            obj.vrep = remApi('remoteApi','extApi.h');
            
            % IP address and port
            % wait until connected
            % do not attempt to reconnect
            % timeout is 2000ms
            % comms cycle time is 5ms
            
            if ~isempty(opt.port)
                port = opt.port; % override the default port
            end
            obj.client = obj.vrep.simxStart('127.0.0.1', port, true, ~opt.reconnect, opt.timeout, opt.cycle);
            
            if obj.client < 0
                error('VREP:noconnect:Cant connect to V-REP simulator');
            end
            
            obj.mode = obj.vrep.simx_opmode_oneshot_wait;
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
            [s,h] = obj.vrep.simxGetObjectHandle(obj.client, sprintf(fmt, varargin{:}), obj.mode);
        end
        
        function children = getchildren(obj, h)
                        %VREP.getchildren Return children of object
            %
            % C = V.getchildren(H) is a vector of integer handles for the V-REP object
            % denoted by the integer handle H.
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
            s = obj.vrep.simxGetConnectionId(obj.client, obj.mode);
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
            s = obj.vrep.simxSetBooleanParameter(obj.client, paramid, val, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end

        function setparam_int(obj, paramid, val)
            %VREP.setparam_int Set intger parameter of the V-REP simulator
            %
            % V.setparam_int(NAME, VAL) sets the integer parameter with name NAME
            % to value VAL within the V-REP simulation engine.
            s = obj.vrep.simxSetIntParameter(obj.client, paramid, val, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end

        function setparam_float(obj, paramid, val)
            %VREP.setparam_float Set float parameter of the V-REP simulator
            %
            % V.setparam_float(NAME, VAL) sets the float parameter with name NAME
            % to value VAL within the V-REP simulation engine.
            s = obj.vrep.simxSetFloatParameter(obj.client, paramid, val, obj.mode);
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
            % V.setobjparam_bool(H, PARAM, VAL) sets the float parameter
            % with identifier PARAM of object H to value VAL.
            s = obj.vrep.simxSetObjectFloatParameter(obj.client, h, paramid, val, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end
        
        function val = getobjparam_bool(obj, h, paramid)
            %VREP.getobjparam_bool get boolean parameter of a V-REP object
            %
            % V.getobjparam_bool(H, PARAM) gets the boolean parameter
            % with identifier PARAM of object with integer handle H.
            [s,val] = obj.vrep.simxGetObjectIntParameter(obj.client, h, paramid, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end

        function val = getobjparam_int(obj, h, paramid)
            %VREP.getobjparam_int get Integer parameter of a V-REP object
            %
            % V.getobjparam_int(H, PARAM) gets the integer parameter
            % with identifier PARAM of object with integer handle H.
            [s,val] = obj.vrep.simxGetObjectIntParameter(obj.client, h, paramid, obj.mode);
            if s ~= 0
                throw( obj.except(s) );
            end
        end

        function val = getobjparam_float(obj, h, paramid)
            %VREP.getobjparam_float get float parameter of a V-REP object
            %
            % V.getobjparam_bool(H, PARAM) gets the float parameter
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
            % V.getpose(H) is the pose (4x4) of the V-REP object with integer
            % handle H.
            %
            % V.getpose(H, HR) as above but pose is relative to the
            % pose of object with integer handle R.
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
            % V.getorient(H) is the orientation as a rotation matrix (3x3) of 
            % the V-REP object with integer handle H.
            %
            % V.getorient(H, 'euler', OPTIONS) as above but returns ZYZ Euler
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
            % See also VREP_arm.

            a = VREP_arm(obj, name);
        end
        
        function o = object(obj, name)
            %VREP.arm Return VREP_obj object
            %
            % V.objet(name) is a factory method that returns a VREP_obj object for the V-REP
            % object named NAME.
            %
            % See also VREP_obj.
            o = VREP_obj(obj, name);
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
        end
        e = MException(msgid, err);
end
end