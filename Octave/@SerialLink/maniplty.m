%SerialLink.MANIPLTY Manipulability measure
%
% M = R.maniplty(Q, OPTIONS) is the manipulability index measure for the robot
% at the joint configuration Q.  It indicates dexterity, how isotropic the
% robot's motion is with respect to the 6 degrees of Cartesian motion.
% The measure is low when the manipulator is close to a singularity.
% If Q is a matrix M is a column vector of  manipulability 
% indices for each pose specified by a row of Q.
%
% Two measures can be selected:
% - Yoshikawa's manipulability measure is based on the shape of the velocity
%   ellipsoid and depends only on kinematic parameters.
% - Asada's manipulability measure is based on the shape of the acceleration
%   ellipsoid which in turn is a function of the Cartesian inertia matrix and
%   the dynamic parameters.  The scalar measure computed here is the ratio of 
%   the smallest/largest ellipsoid axis.  Ideally the ellipsoid would be 
%   spherical, giving a ratio of 1, but in practice will be less than 1.
%
% Options::
% 'T'           compute manipulability for just transational motion
% 'R'           compute manipulability for just rotational motion
% 'yoshikawa'   use Asada algorithm (default)
% 'asada'       use Asada algorithm
%
% Notes::
% - by default the measure includes rotational and translational dexterity, but
%   this involves adding different units.  It can be more useful to look at the
%   translational and rotational manipulability separately.
%
% See also SerialLink.inertia, SerialLink.jacob0.

% Ryan Steindl based on Robotics Toolbox for MATLAB (v6 and v9)
%
% Copyright (C) 1993-2011, by Peter I. Corke
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

function [w,mx] = maniplty(robot, q, varargin)
	n = robot.n;

    opt.yoshikawa = true;
    opt.asada = false;
    opt.axes = {'all', 'T', 'R'};

    opt = tb_optparse(opt, varargin);

	if length(q) == robot.n,
		q = q(:)';
	end

	w = [];
    MX = [];

    if opt.yoshikawa
		for Q = q',
			w = [w; yoshi(robot, Q, opt)];
		end
	elseif opt.asada
		for Q = q',
            if nargout > 1
                [ww,mm] = asada(robot, Q);
                w = [w; ww];
                MX = cat(3, MX, mm);
            else
                w = [w; asada(robot, Q, opt)];
            end
		end
	end

    if nargout > 1
        mx = MX;
    end

function m = yoshi(robot, q, opt)
	J = jacob0(robot, q);
    switch opt.axes
    case 'T'
        J = J(1:3,:);
    case 'R'
        J = J(4:6,:);
    end
	m = sqrt(det(J * J'));

function [m, mx] = asada(robot, q, opt)
	J = jacob0(robot, q);
    
    if rank(J) < 6,
        warning('robot is in degenerate configuration')
        m = 0;
        return;
    end

    switch opt.axes
    case 'T'
        J = J(1:3,:)
    case 'R'
        J = J(4:6,:);
    end
	Ji = inv(J);
	M = inertia(robot, q);
	Mx = Ji' * M * Ji;
	e = eig(Mx(1:3,1:3));
	m = min(e) / max(e);

    if nargout > 1
        mx = Mx;
    end
