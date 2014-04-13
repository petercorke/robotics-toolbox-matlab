%VREP_arm V-REP mirror of robot arm object
%
% Mirror objects are MATLAB objects that reflect objects in the V-REP
% environment.  Methods allow the V-REP state to be examined or changed.
%
% This is a concrete class, derived from VREP_mirror, for all V-REP robot
% arm objects and allows access to joint variables.
%
% Methods throw exception if an error occurs.
%
% Example::
%          vrep = VREP();
%          arm = vrep.arm('IRB140');
%          q = arm.getq();
%          arm.setq(zeros(1,6));
%          arm.setpose(T);  % set pose of base
%   
% Methods::
%
%  getq              return joint coordinates
%  setq              set joint coordinates
%  animate           animate a joint coordinate trajectory
%
% Superclass methods (VREP_obj)::
%  getpos              return position of object given handle
%  setpos              set position of object given handle
%  getorient           return orientation of object given handle
%  setorient           set orientation of object given handle
%  getpose             return pose of object given handle
%  setpose             set pose of object given handle
%
% can be used to set/get the pose of the robot base.
%
% Superclass methods (VREP_base)::
%  setobjparam_bool    set object boolean parameter
%  setobjparam_int     set object integer parameter
%  setobjparam_float   set object float parameter
%
% Properties::
%  n     Number of joints
%
% See also VREP_mirror, VREP_obj, VREP_arm, VREP_camera, VREP_hokuyo.

classdef VREP_arm < VREP_obj
    
    properties(GetAccess=public, SetAccess=protected)
        q
        joint   % VREP joint object handles
        n       % number of joints
        
    end
    
    
    methods
        
        function arm = VREP_arm(vrep, name, varargin)
            %VREP_arm.VREP_arm Create a robot arm mirror object
            %
            % R = VREP_arm(NAME, OPTIONS) is a mirror object that corresponds to the
            % robot arm named NAME in the V-REP environment.
            %
            % Options::
            % 'fmt',F    Specify format for joint object names (default '%s_joint%d')
            %
            % Notes::
            % - The number of joints is found by searching for objects
            %   with names systematically derived from the root object name, by
            %   default named NAME_N where N is the joint number starting at 0.
            %
            % See also VREP.arm.
            
            h = vrep.gethandle(name);
            if h == 0
                error('no such object as %s in the scene', name);
            end
            arm = arm@VREP_obj(vrep, name);
            
            opt.fmt = '%s_joint%d';
            opt = tb_optparse(opt, varargin);
            
            arm.name = name;
            
            % find all the _joint objects, we don't know how many joints so we
            % keep going till we get an error
            j = 1;
            while true
                [h,s] = vrep.gethandle(opt.fmt, name, j);
                if s ~= 0
                    break
                end
                arm.joint(j) = h;
                
                j = j+1;
            end
            
            arm.n = j - 1;
            
            % set all joints to passive mode
            %             for j=1:arm.n
            %                 arm.vrep.simxSetJointMode(arm.client, arm.joint(j), arm.vrep.sim_jointmode_passive, arm.vrep.simx_opmode_oneshot_wait);
            %             end
        end
        
        function q = getq(arm)
            %VREP_arm.getq  Get joint angles of V-REP robot
            %
            % R.getq() is the vector of joint angles (1xN) from the corresponding
            % robot arm in the V-REP simulation.
            for j=1:arm.n
                q(j) = arm.vrep.getjoint(arm.joint(j));
            end
        end
        
        function setq(arm, q)
            %VREP_arm.setq  Set joint angles of V-REP robot
            %
            % R.setq(Q) sets the joint angles of the corresponding
            % robot arm in the V-REP simulation to Q (1xN).
            for j=1:arm.n
                arm.vrep.setjoint(arm.joint(j), q(j));
            end
        end
        
        function animate(arm, qt, varargin)
            %VREP_arm.setq  Animate V-REP robot
            %
            % R.animate(QT, OPTIONS) animate the corresponding V-REP robot with
            % configurations taken consecutive rows of QT (MxN) which represents 
            % an M-point trajectory.
            %
            % Options::
            % 'delay',D         Delay (s) betwen frames for animation (default 0.1)
            % 'fps',fps         Number of frames per second for display, inverse of 'delay' option
            % '[no]loop'        Loop over the trajectory forever
            %
            % See also SerialLink.plot.

            opt.delay = 0.1;
            opt.fps = [];
            opt.loop = false;
            opt = tb_optparse(opt, varargin);
            
            if ~isempty(opt.fps)
                opt.delay = 1/opt.fps;
            end
            
            while true
                for i=1:numrows(qt)
                    arm.setq(qt(i,:));
                    pause(opt.delay);
                end
                if ~opt.loop
                    break;
                end
            end
        end
        

        function teach(obj, varargin)
            %VREP_arm.teach Graphical teach pendant
            %
            % R.teach(OPTIONS) drive a V-REP robot by means of a graphical slider panel.
            %
            % Options::
            % 'degrees'    Display angles in degrees (default radians)
            % 'q0',q       Set initial joint coordinates
            %
            %
            % Notes::
            % - The slider limits are all assumed to be [-pi, +pi]
            %
            % See also SerialLink.plot.
            
            n = obj.n;
            
            %-------------------------------
            % parameters for teach panel
            bgcol = [135 206 250]/255;  % background color
            height = 1/(n+2);  % height of slider rows
            %-------------------------------
            opt.degrees = false;
            opt.q0 = [];
            % TODO: options for rpy, or eul angle display
            
            opt = tb_optparse(opt, varargin);
            
            % drivebot(r, q)
            % drivebot(r, 'deg')
            
            
            if isempty(opt.q0)
                q = obj.getq();
            else
                q = opt.q0;
            end
            
            % set up scale factor, from actual limits in radians/metres to display units
            qscale = ones(n,1);
            for j=1:n
                if opt.degrees && L.isrevolute
                    qscale(j) = 180/pi;
                end
                qlim(j,:) = [-pi pi];
            end
            
            handles.qscale = qscale;
            
            panel = figure(...
                'BusyAction', 'cancel', ...
                'HandleVisibility', 'off', ...
                'Color', bgcol);
            pos = get(panel, 'Position');
            pos(3:4) = [300 400];
            set(panel, 'Position', pos);
            set(panel,'MenuBar','none')
            set(panel, 'name', 'Teach');
            delete( get(panel, 'Children') )
            
            
            %---- now make the sliders
            for j=1:n
                % slider label
                uicontrol(panel, 'Style', 'text', ...
                    'Units', 'normalized', ...
                    'BackgroundColor', bgcol, ...
                    'Position', [0 height*(n-j) 0.15 height], ...
                    'FontUnits', 'normalized', ...
                    'FontSize', 0.5, ...
                    'String', sprintf('q%d', j));
                
                % slider itself
                q(j) = max( qlim(j,1), min( qlim(j,2), q(j) ) ); % clip to range
                handles.slider(j) = uicontrol(panel, 'Style', 'slider', ...
                    'Units', 'normalized', ...
                    'Position', [0.15 height*(n-j) 0.65 height], ...
                    'Min', qlim(j,1), ...
                    'Max', qlim(j,2), ...
                    'Value', q(j), ...
                    'Tag', sprintf('Slider%d', j));
                
                % text box showing slider value, also editable
                handles.edit(j) = uicontrol(panel, 'Style', 'edit', ...
                    'Units', 'normalized', ...
                    'Position', [0.80 height*(n-j)+.01 0.20 height*0.9], ...
                    'BackgroundColor', bgcol, ...
                    'String', num2str(qscale(j)*q(j), 3), ...
                    'HorizontalAlignment', 'left', ...
                    'FontUnits', 'normalized', ...
                    'FontSize', 0.4, ...
                    'Tag', sprintf('Edit%d', j));
            end
            
            %---- robot name text box
            uicontrol(panel, 'Style', 'text', ...
                'Units', 'normalized', ...
                'FontUnits', 'normalized', ...
                'FontSize', 1, ...
                'HorizontalAlignment', 'center', ...
                'Position', [0.05 1-height*1.5 0.9 height], ...
                'BackgroundColor', 'white', ...
                'String', 'bobbot');
            
            handles.arm = obj;
            
            % now assign the callbacks
            for j=1:n
                % text edit box
                set(handles.edit(j), ...
                    'Interruptible', 'off', ...
                    'Callback', @(src,event)teach_callback(j, handles, src));
                
                % slider
                set(handles.slider(j), ...
                    'Interruptible', 'off', ...
                    'BusyAction', 'queue', ...
                    'Callback', @(src,event)teach_callback(j, handles, src));        
            end
        end
    end
end

function teach_callback(j, handles, src)

    % called on changes to a slider or to the edit box showing joint coordinate
    %
    % src      the object that caused the event
    % name     name of the robot
    % j        the joint index concerned (1..N)
    % slider   true if the

    qscale = handles.qscale;

    switch get(src, 'Style')
        case 'slider'
            % slider changed, get value and reflect it to edit box
            newval = get(src, 'Value');
            set(handles.edit(j), 'String', num2str(qscale(j)*newval));
        case 'edit'
            % edit box changed, get value and reflect it to slider
            newval = str2double(get(src, 'String')) / qscale(j);
            set(handles.slider(j), 'Value', newval);
    end
    %fprintf('newval %d %f\n', j, newval);

    handles.arm.vrep.setjoint(handles.arm.joint(j), newval);

end





