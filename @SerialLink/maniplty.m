%SerialLink.MANIPLTY Manipulability measure
%
% M = R.maniplty(Q, OPTIONS) is the manipulability index (scalar) for the
% robot at the joint configuration Q (1xN) where N is the number of robot
% joints.  It indicates dexterity, that is, how isotropic the robot's
% motion is with respect to the 6 degrees of Cartesian motion. The measure
% is high when the manipulator is capable of equal motion in all directions
% and low when the manipulator is close to a singularity.
%
% If Q is a matrix (MxN) then M (Mx1) is a vector of  manipulability 
% indices for each joint configuration specified by a row of Q.
%
% [M,CI] = R.maniplty(Q, OPTIONS) as above, but for the case of the Asada
% measure returns the Cartesian inertia matrix CI.
%
% R.maniplty(Q) displays the translational and rotational manipulability.
%
% Two measures can be computed:
% - Yoshikawa's manipulability measure is based on the shape of the velocity
%   ellipsoid and depends only on kinematic parameters (default).
% - Asada's manipulability measure is based on the shape of the acceleration
%   ellipsoid which in turn is a function of the Cartesian inertia matrix and
%   the dynamic parameters.  The scalar measure computed here is the ratio of 
%   the smallest/largest ellipsoid axis.  Ideally the ellipsoid would be 
%   spherical, giving a ratio of 1, but in practice will be less than 1.
%
% Options::
% 'trans'       manipulability for transational motion only (default)
% 'rot'         manipulability for rotational motion only
% 'all'         manipulability for all motions
% 'dof',D       D is a vector (1x6) with non-zero elements if the
%               corresponding DOF is to be included for manipulability
% 'yoshikawa'   use Yoshikawa algorithm (default)
% 'asada'       use Asada algorithm
%
% Notes::
% - The 'all' option includes rotational and translational dexterity, but
%   this involves adding different units.  It can be more useful to look at the
%   translational and rotational manipulability separately.
% - Examples in the RVC book (1st edition) can be replicated by using the 'all' option
%
% References::
%
% - Analysis and control of robot manipulators with redundancy,
%   T. Yoshikawa,
%   Robotics Research: The First International Symposium (M. Brady and R. Paul, eds.),
%   pp. 735-747, The MIT press, 1984.
% - A geometrical representation of manipulator dynamics and its application to 
%   arm design,
%   H. Asada, 
%   Journal of Dynamic Systems, Measurement, and Control,
%   vol. 105, p. 131, 1983.
% - Robotics, Vision & Control, P. Corke, Springer 2011.
%
% See also SerialLink.inertia, SerialLink.jacob0.





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

%TODO
% return the ellipsoid?

function [w,mx] = maniplty(robot, q, varargin)
    


    opt.method = {'yoshikawa', 'asada'};
    opt.axes = {'all', 'trans', 'rot'};
    opt.dof = [];
    
    opt = tb_optparse(opt, varargin);
    
    if nargout == 0
        opt.axes = 'trans';
        mt = maniplty(robot, q, 'setopt', opt);
        opt.axes = 'rot';
        mr = maniplty(robot, q, 'setopt', opt);
        for i=1:numrows(mt)
        fprintf('Manipulability: translation %g, rotation %g\n', mt(i), mr(i));
        end
        return;
    end
    
    if isempty(opt.dof)
        switch opt.axes
            case 'trans'
                dof = [1 1 1 0 0 0];
            case 'rot'
                dof = [0 0 0 1 1 1];
            case 'all'
                dof = [1 1 1 1 1 1];
        end
    else
        dof = opt.dof;
    end
    
    opt.dof = logical(dof);

    if strcmp(opt.method, 'yoshikawa')
        w = zeros(numrows(q),1);
        for i=1:numrows(q)
            w(i) = yoshi(robot, q(i,:), opt);
        end
    elseif strcmp(opt.method, 'asada')
        w = zeros(numrows(q),1);
        if nargout > 1
            dof = sum(opt.dof);
            MX = zeros(dof,dof,numrows(q));
            for i=1:numrows(q)
                [ww,mm] = asada(robot, q(i,:), opt);
                w(i) = ww;
                MX(:,:,i) = mm;
            end
        else
            for i=1:numrows(q)
                w(i) = asada(robot, q(i,:), opt);
            end
        end
    end

    if nargout > 1
        mx = MX;
    end

function m = yoshi(robot, q, opt)
    J = robot.jacob0(q);
    
    J = J(opt.dof,:);
    m2 = det(J * J');
    m2 = max(0, m2);    % clip it to positive
    m = sqrt(m2);

function [m, mx] = asada(robot, q, opt)
    J = robot.jacob0(q);
    
    if rank(J) < 6
        warning('robot is in degenerate configuration')
        m = 0;
        return;
    end

    Ji = pinv(J);
    M = robot.inertia(q);
    Mx = Ji' * M * Ji;
    d = find(opt.dof);
    Mx = Mx(d,d);
    e = eig(Mx);
    m = min(e) / max(e);

    if nargout > 1
        mx = Mx;
    end
