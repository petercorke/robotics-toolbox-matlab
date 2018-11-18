%SerialLink.vellipse Velocity ellipsoid for seriallink manipulator
%
% R.vellipse(Q, OPTIONS) displays the velocity ellipsoid for the 
% robot R at pose Q.  The ellipsoid is centered at the tool tip position.
%
% Options::
% '2d'       Ellipse for translational xy motion, for planar manipulator
% 'trans'    Ellipsoid for translational motion (default)
% 'rot'      Ellipsoid for rotational motion
%
% Display options as per plot_ellipse to control ellipsoid face and edge
% color and transparency.
%
% Example::
%  To interactively update the velocity ellipsoid while using sliders
%  to change the robot's pose:
%          robot.teach('callback', @(r,q) r.vellipse(q))
%
% Notes::
% - The ellipsoid is tagged with the name of the robot prepended to
%   ".vellipse".
% - Calling the function with a different pose will update the ellipsoid.
%
% See also SerialLink.jacob0, SerialLink.fellipse, plot_ellipse.

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

function vellipse(robot, q, varargin)
    
    name = [robot.name '.vellipse'];
    
    e = findobj('Tag', name);
    
    if isempty(q)
        delete(e);
        return;
    end
    
    opt.mode = {'trans', 'rot', '2d'};
    opt.deg = false;
    [opt,args] = tb_optparse(opt, varargin);
    
    if opt.deg
        % in degrees mode, scale the columns corresponding to revolute axes
        q = robot.todegrees(q);
    end
    if robot.n == 2
        opt.mode = '2d';
    end
    
    J = robot.jacob0(q);
    
    switch opt.mode
        case'2d'
            J = J(1:2,1:2);
        case 'trans'
            J = J(1:3,:);
        case 'rot'
            J = J(4:6,:);
    end
    
    N = (J*J');

    if det(N) < 100*eps
        warning('RTB:fellipse:badval', 'Jacobian is singular, ellipse cannot be drawn')
        return
    end
    
    t = transl(robot.fkine(q));
    
    switch opt.mode
        case '2d'
            if isempty(e)
                h = plot_ellipse(N, t(1:2), 'edgecolor', 'r', 'Tag', name, args{:});
            else
                plot_ellipse(N, t(1:2), 'alter', e);
            end
        otherwise
            if isempty(e)
                h = plot_ellipse(N, t(1:3), 'edgecolor', 'k', 'fillcolor', 'r', 'alpha', 0.5, 'Tag', name, args{:});
            else
                plot_ellipse(N, t(1:3), 'alter', e);
            end
    end
end
