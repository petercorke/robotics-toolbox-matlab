%SerialLink.IKINE Inverse manipulator kinematics
%
% Q = R.ikine(T) is the joint coordinates corresponding to the robot 
% end-effector pose T which is a homogenenous transform.
%
% Q = R.ikine(T, Q0, OPTIONS) specifies the initial estimate of the joint 
% coordinates.
%
% Q = R.ikine(T, Q0, M, OPTIONS) specifies the initial estimate of the joint 
% coordinates and a mask matrix.  For the case where the manipulator 
% has fewer than 6 DOF the solution space has more dimensions than can
% be spanned by the manipulator joint coordinates.  In this case
% the mask matrix M specifies the Cartesian DOF (in the wrist coordinate 
% frame) that will be ignored in reaching a solution.  The mask matrix 
% has six elements that correspond to translation in X, Y and Z, and rotation 
% about X, Y and Z respectively.  The value should be 0 (for ignore) or 1.
% The number of non-zero elements should equal the number of manipulator DOF.
%
% For example when using a 5 DOF manipulator rotation about the wrist z-axis
% might be unimportant in which case  M = [1 1 1 1 1 0].
%
% In all cases if T is 4x4xM it is taken as a homogeneous transform sequence 
% and R.ikine() returns the joint coordinates corresponding to each of the 
% transforms in the sequence.  Q is MxN where N is the number of robot joints.
% The initial estimate of Q for each time step is taken as the solution 
% from the previous time step.
%
% Options::
%
% Notes::
% - Solution is computed iteratively using the pseudo-inverse of the
%   manipulator Jacobian.
% - The inverse kinematic solution is generally not unique, and 
%   depends on the initial guess Q0 (defaults to 0).
% - Such a solution is completely general, though much less efficient 
%   than specific inverse kinematic solutions derived symbolically.
% - This approach allows a solution to obtained at a singularity, but 
%   the joint angles within the null space are arbitrarily assigned.
%
% See also SerialLink.fkine, tr2delta, SerialLink.jacob0, SerialLink.ikine6s.
 
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

function qt = ikine(robot, tr, varargin)
    %  set default parameters for solution
    opt.ilimit = 100;
    opt.tol = 1e-6;
    opt.alpha = 1;
    opt.plot = false;
    opt.pinv = false;

    [opt,args] = tb_optparse(opt, varargin);

    n = robot.n;

    % robot.ikine(tr, q)
    if length(args) > 0
        q = args{1};
        if isempty(q)
            q = zeros(1, n);
        else
            q = q(:)';
        end
    else
        q = zeros(1, n);
    end

    % robot.ikine(tr, q, m)
    if length(args) > 1
        m = args{2};
        m = m(:);
        if numel(m) ~= 6
            error('Mask matrix should have 6 elements');
        end
        if numel(find(m)) ~= robot.n 
            error('Mask matrix must have same number of 1s as robot DOF')
        end
    else
        if n < 6
            error('For a manipulator with fewer than 6DOF a mask matrix argument must be specified');
        end
        m = ones(6, 1);
    end
    % make this a logical array so we can index with it
    m = logical(m);

    npoints = size(tr,3);    % number of points
    qt = zeros(npoints, n);  % preallocate space for results
    tcount = 0;              % total iteration count
    eprev = Inf;

    save.e = Inf;
    save.q = [];

    history = [];
    for i=1:npoints
        T = tr(:,:,i);
        nm = Inf;
        count = 0;

        optim = optimset('Display', 'iter', 'TolX', 0, 'TolFun', opt.tol, 'MaxIter', opt.ilimit);
        optim
        q = fminsearch(@(x) ikinefunc(x, T, robot, m), q, optim);
    
    end
    qt = q;
end

function E = ikinefunc(q, T, robot, m)
        e = tr2delta(fkine(robot, q'), T);
        E = norm(e(m));
end
