%SerialLink.IKINEM Numerical inverse kinematics by minimization
%
% Q = R.ikinem(T) is the joint coordinates corresponding to the robot
% end-effector pose T which is a homogenenous transform.
%
% Q = R.ikinem(T, Q0, OPTIONS) specifies the initial estimate of the joint
% coordinates.
%
% In all cases if T is 4x4xM it is taken as a homogeneous transform sequence
% and R.ikinem() returns the joint coordinates corresponding to each of the
% transforms in the sequence.  Q is MxN where N is the number of robot joints.
% The initial estimate of Q for each time step is taken as the solution
% from the previous time step.
%
% Options::
% 'pweight',P      weighting on position error norm compared to rotation
%                  error (default 1)
% 'stiffness',S    Stiffness used to impose a smoothness contraint on joint
%                  angles, useful when N is large (default 0)
% 'qlimits'        Enforce joint limits
% 'ilimit',L       Iteration limit (default 1000)
% 'nolm'           Disable Levenberg-Marquadt
%
% Notes::
% - PROTOTYPE CODE UNDER DEVELOPMENT, intended to do numerical inverse kinematics
%   with joint limits
% - The inverse kinematic solution is generally not unique, and
%   depends on the initial guess Q0 (defaults to 0).
% - The function to be minimized is highly nonlinear and the solution is
%   often trapped in a local minimum, adjust Q0 if this happens.
% - The default value of Q0 is zero which is a poor choice for most
%   manipulators (eg. puma560, twolink) since it corresponds to a kinematic
%   singularity.
% - Such a solution is completely general, though much less efficient
%   than specific inverse kinematic solutions derived symbolically, like
%   ikine6s or ikine3.% - Uses Levenberg-Marquadt minimizer LMFsolve if it can be found,
%   if 'nolm' is not given, and 'qlimits' false
% - The error function to be minimized is computed on the norm of the error 
%   between current and desired tool pose.  This norm is computed from distances
%   and angles and 'pweight' can be used to scale the position error norm to
%   be congruent with rotation error norm.
% - This approach allows a solution to obtained at a singularity, but
%   the joint angles within the null space are arbitrarily assigned.
% - Joint offsets, if defined, are added to the inverse kinematics to
%   generate Q.
% - Joint limits become explicit contraints if 'qlimits' is set.
%
% See also fminsearch, fmincon, SerialLink.fkine, SerialLink.ikine, tr2angvec.




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

function qt = ikinem(robot, tr, varargin)
    
    opt.pweight = 1;
    opt.stiffness = 0;
    opt.qlimits = false;
    opt.ilimit = 1000;
    opt.lm = true;
    opt.col = 2;
    
    [opt,args] = tb_optparse(opt, varargin);
    
    % check if optional argument is a valid q
    q0 = args{1};
    if numel(q0) ~= robot.n
        error('q0 length must match number of joints in robot');
    end
    
    
    for i=1:size(tr,3)
        T = tr(:,:,i);
        
        if opt.qlimits
            % constrained optimization to handle joint limits
            options = optimset('MaxIter', opt.ilimit);
            qlim = robot.qlim;
            
            [q, ef, exflag, output] = fmincon( @(x) costfun(x, robot, T, opt), q0, ...
                [], [], [], [], ...
                qlim(:,1), qlim(:,2), ...
                [], options);
            
            if opt.verbose
                fprintf('final error %f, %d iterations, %d evalations\n', ...
                    ef, output.iterations, output.funcCount);
            end
            
        else
            % no joint limits, unconstrained optimization
            if exist('LMFsolve') == 2 && opt.lm
                [q, ef, count] = LMFsolve( @(x) costfun(x, robot, T, opt), q0, 'MaxIter', opt.ilimit);
                q = q';
                if opt.verbose
                    fprintf('final error %f, %d iterations\n', ...
                        ef, count);
                end
            else
                options = optimset('MaxIter', opt.ilimit);
                [q, ef, exflag, output] = fminsearch( @(x) costfun(x, robot, T, opt), q0, options);
                
                if opt.verbose
                    fprintf('final error %f, %d iterations, %d evalations\n', ...
                        ef, output.iterations, output.funcCount);
                end
            end
        end
        
        qt(i,:) = q;
    end
    
    if opt.verbose
        robot.fkine(qt)
    end
end

% The cost function, this is the value to be minimized
function E = costfun(q, robot, T, opt)
    
    Tq = robot.fkine(q);
    % find the pose error in SE(3)
    dT = transl(T) - transl(Tq);
    
    % translation error
    E = norm(dT) * opt.pweight;
    
    % rotation error
    %  find dot product of 
    dd = dot(T(1:3,opt.col), Tq(1:3,opt.col));
    %E = E + (1 - dd)^2*100000 ;
    E = E + acos(dd)^2*1000 ;
    
    
    if opt.stiffness > 0
        % enforce a continuity constraint on joints, minimum bend
        E = E + sum( diff(q).^2 ) * opt.stiffness;
    end
end
    
