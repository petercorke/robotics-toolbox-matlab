%SerialLink.teach Graphical teach pendant
%
% Allow the user to "drive" a graphical robot using a graphical slider
% panel.
%
% R.teach(OPTIONS) adds a slider panel to a current robot plot.
%
% R.teach(Q, OPTIONS) as above but the robot joint angles are set to Q (1xN).
%
%
% Options::
% 'eul'           Display tool orientation in Euler angles (default)
% 'rpy'           Display tool orientation in roll/pitch/yaw angles
% 'approach'      Display tool orientation as approach vector (z-axis)
% '[no]deg'       Display angles in degrees (default true)
% 'callback',CB   Set a callback function, called with robot object and
%                 joint angle vector: CB(R, Q)
%
% Example::
%
% To display the velocity ellipsoid for a Puma 560
%
%        p560.teach('callback', @(r,q) r.vellipse(q));
%
% GUI::
% - The specified callback function is invoked every time the joint configuration changes.
%   the joint coordinate vector.
% - The Quit (red X) button removes the teach panel from the robot plot.
%
% Notes::
% - If the robot is displayed in several windows, only one has the
%   teach panel added.
% - All currently displayed robots move as the sliders are adjusted.
% - The slider limits are derived from the joint limit properties.  If not
%   set then for
%   - a revolute joint they are assumed to be [-pi, +pi]
%   - a prismatic joint they are assumed unknown and an error occurs.
%
% See also SerialLink.plot, SerialLink.getpos.



% Copyright (C) 1993-2017, by Peter I. Corke
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

% a ton of handles and parameters created by this function are stashed in
% a structure which is passed into all callbacks

function teach(robot, varargin)
    
    
    %---- handle options
    opt.deg = true;
    opt.orientation = {'rpy', 'rpy/zyx', 'rpy/xyz', 'eul', 'approach'};
    opt.d_2d = false;
    opt.callback = [];
    opt.record = [];
    [opt,args] = tb_optparse(opt, varargin);
    
    % get the joint coordinates if given
    q = [];
    if isempty(args)
        % no joint angles passed, assume all zeros
        q = zeros(1, robot.n);
        
        % set any prismatic joints to the minimum value
        for j=find(robot.links.isprismatic)
                q(j) = robot.links(j).qlim(1);
        end
    else
        % joint angles passed
        if isnumeric(args{1})
            q = args{1};
            
            args = args(2:end);
        end
    end
    
    %---- get the current robot state
    
    % check to see if there are any graphical robots of this name
    rhandles = findobj('Tag', robot.name);   % find the graphical element of this name
    
    if isempty(rhandles)
        % no robot, plot one so long as joint coordinates were given
        assert( ~isempty(q), 'RTB:teach:badarg', 'No joint coordinates provided');
        robot.plot(q, args{:});
    else
        % graphical robot already exists
        %   get the info from its Userdata
        info = get(rhandles(1), 'UserData');
        
        % the handle contains current joint angles (set by plot)
        if isempty(q) && ~isempty(info.q)
            % if no joint coordinates given get from the graphical model
            q = info.q;
        else
            % joint coordiantes were given, make them current
            robot.plot(q, args{:});
        end
    end
    
    assert( ~isempty(q), 'RTB:teach:badarg', 'No joint coordinates provided');
    RTBPlot.install_teach_panel(robot.name, robot, q, opt)
    
end


