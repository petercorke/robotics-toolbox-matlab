%SerialLink.ikine Numerical inverse kinematics
%
% Q = R.ikine(T) are the joint coordinates (1xN) corresponding to the robot 
% end-effector pose T (4x4) which is a homogenenous transform.
%
% Q = R.ikine(T, Q0, OPTIONS) specifies the initial estimate of the joint 
% coordinates.
%
% This method can be used for robots with 6 or more degrees of freedom.
%
% Underactuated robots::
%
% For the case where the manipulator has fewer than 6 DOF the solution 
% space has more dimensions than can be spanned by the manipulator joint 
% coordinates.
%
% Q = R.ikine(T, Q0, M, OPTIONS) similar to above but where M is a mask 
% vector (1x6) which specifies the Cartesian DOF (in the wrist coordinate 
% frame) that will be ignored in reaching a solution.  The mask vector 
% has six elements that correspond to translation in X, Y and Z, and rotation 
% about X, Y and Z respectively.  The value should be 0 (for ignore) or 1.
% The number of non-zero elements should equal the number of manipulator DOF.
%
% For example when using a 3 DOF manipulator rotation orientation might be 
% unimportant in which case  M = [1 1 1 0 0 0].
%
% For robots with 4 or 5 DOF this method is very difficult to use since
% orientation is specified by T in world coordinates and the achievable
% orientations are a function of the tool position.
%
% Trajectory operation::
%
% In all cases if T is 4x4xM it is taken as a homogeneous transform sequence 
% and R.ikine() returns the joint coordinates corresponding to each of the 
% transforms in the sequence.  Q is MxN where N is the number of robot joints.
% The initial estimate of Q for each time step is taken as the solution 
% from the previous time step.
%
% Options::
% 'pinv'         use pseudo-inverse instead of Jacobian transpose (default)
% 'ilimit',L     set the maximum iteration count (default 1000)
% 'tol',T        set the tolerance on error norm (default 1e-6)
% 'alpha',A      set step size gain (default 1)
% 'varstep'      enable variable step size if pinv is false
% 'verbose'      show number of iterations for each point
% 'verbose=2'    show state at each iteration
% 'plot'         plot iteration state versus time
%
% References::
% - Robotics, Vision & Control, Section 8.4,
%   P. Corke, Springer 2011.
%
% Notes::
% - Solution is computed iteratively.
% - Solution is sensitive to choice of initial gain.  The variable
%   step size logic (enabled by default) does its best to find a balance
%   between speed of convergence and divergence.
% - Some experimentation might be required to find the right values of 
%   tol, ilimit and alpha.
% - The pinv option leads to much faster convergence (default)
% - The tolerance is computed on the norm of the error between current
%   and desired tool pose.  This norm is computed from distances
%   and angles without any kind of weighting.
% - The inverse kinematic solution is generally not unique, and 
%   depends on the initial guess Q0 (defaults to 0).
% - The default value of Q0 is zero which is a poor choice for most
%   manipulators (eg. puma560, twolink) since it corresponds to a kinematic
%   singularity.
% - Such a solution is completely general, though much less efficient 
%   than specific inverse kinematic solutions derived symbolically, like
%   ikine6s or ikine3.
% - This approach allows a solution to be obtained at a singularity, but 
%   the joint angles within the null space are arbitrarily assigned.
% - Joint offsets, if defined, are added to the inverse kinematics to 
%   generate Q.
% - Joint limits are not considered in this solution.
%
% See also SerialLink.ikcon, SerialLink.ikunc, SerialLink.fkine, SerialLink.ikine6s.
 

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

function [qt,histout] = ikine(robot, tr, varargin)

    %  set default parameters for solution
    opt.ilimit = 1000;
    opt.tol = 1e-6;
    opt.alpha = 0.9;
    opt.plot = false;
    opt.pinv = true;
    opt.varstep = false;

    [opt,args] = tb_optparse(opt, varargin);

    n = robot.n;

    % robot.ikine(tr, q)
    if ~isempty(args)
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
            error('RTB:ikine:badarg', 'Mask matrix should have 6 elements');
        end
        if numel(find(m)) > robot.n 
            error('RTB:ikine:badarg', 'Number of robot DOF must be >= the same number of 1s in the mask matrix')
        end
    else
        if n < 6
            error('RTB:ikine:badarg', 'For a manipulator with fewer than 6DOF a mask matrix argument must be specified');
        end
        m = ones(6, 1);
    end
    % make this a logical array so we can index with it
    m = logical(m);

    npoints = size(tr,3);    % number of points
    qt = zeros(npoints, n);  % preallocate space for results
    tcount = 0;              % total iteration count

    if ~ishomog(tr)
        error('RTB:ikine:badarg', 'T is not a homog xform');
    end

    J0 = jacob0(robot, q);
    J0 = J0(m, :);
    if cond(J0) > 100
        warning('RTB:ikine:singular', 'Initial joint configuration results in a (near-)singular configuration, this may slow convergence');
    end

    history = [];
    failed = false;
    e = zeros(6,1);
    revolutes = [robot.links.sigma] == 0;

    
    for i=1:npoints
        T = tr(:,:,i);

        nm = Inf;
        % initialize state for the ikine loop
        eprev = [Inf Inf Inf Inf Inf Inf];
        save.e = [Inf Inf Inf Inf Inf Inf];
        save.q = [];
        count = 0;

        while true
            % update the count and test against iteration limit
            count = count + 1;
            if count > opt.ilimit
                warning('ikine: iteration limit %d exceeded (row %d), final err %f', ...
                    opt.ilimit, i, nm);
                failed = true;
                %q = NaN*ones(1,n);
                break
            end

            % compute the error
            Tq = robot.fkine(q');
            
            e(1:3) = transl(T - Tq);
            Rq = t2r(Tq);
            [th,n] = tr2angvec(Rq'*t2r(T));
            e(4:6) = th*n;
            
            % optionally adjust the step size
            if opt.varstep
                % test against last best error, only consider the DOF of
                % interest
                if norm(e(m)) < norm(save.e(m))
                    % error reduced,
                    % let's save current state of solution and rack up the step size
                    save.q = q;
                    save.e = e;
                    opt.alpha = opt.alpha * (2.0^(1.0/8));
                    if opt.verbose > 1
                        fprintf('step %d: raise alpha to %f\n', count, opt.alpha);
                    end
                else
                    % rats!  error got worse,
                    % restore to last good solution and reduce step size
                    q = save.q;
                    e = save.e;
                    opt.alpha = opt.alpha * 0.5;
                    if opt.verbose > 1
                        fprintf('step %d: drop alpha to %f\n', count, opt.alpha);
                    end
                end
            end

            % compute the Jacobian
            J = jacob0(robot, q);

            % compute change in joint angles to reduce the error, 
            % based on the square sub-Jacobian
            if opt.pinv
                %pinv( J(m,:) )
                dq = opt.alpha * pinv( J(m,:) ) * e(m);
            else
                dq = J(m,:)' * e(m);
                dq = opt.alpha * dq;
            end

            % diagnostic stuff
            if opt.verbose > 1
                fprintf('%d/%d: |e| = %f\n', i, count, nm);
                fprintf('       e  = '); disp(e');
                fprintf('       dq = '); disp(dq');
            end
            if opt.plot
                h.q = q';
                h.dq = dq;
                h.e = e;
                h.ne = nm;
                h.alpha = opt.alpha;
                history = [history; h]; %#ok<*AGROW>
            end

            % update the estimated solution
            q = q + dq';
            
            % wrap angles for revolute joints
            k = (q > pi) & revolutes;
            q(k) = q(k) - 2*pi;
            
            k = (q < -pi) & revolutes;
            q(k) = q(k) + 2*pi;
            
            nm = norm(e(m));

            if norm(e(m)) > 2*norm(eprev(m))
                warning('RTB:ikine:diverged', 'solution diverging at step %d, try reducing alpha', count);
            end
            eprev = e;

            if nm <= opt.tol
                break
            end

        end  % end ikine solution for tr(:,:,i)
        qt(i,:) = q';
        tcount = tcount + count;
        if opt.verbose
            fprintf('%d iterations\n', count);
        end
    end
    
    if opt.verbose && npoints > 1
        fprintf('TOTAL %d iterations\n', tcount);
    end

    % plot evolution of variables
    if opt.plot
        figure(1);
        subplot(511)
        plot([history.q]');
        ylabel('q');
        grid

        subplot(512)
        plot([history.dq]');
        ylabel('dq');
        grid

        subplot(513)
        plot([history.e]');
        ylabel('e');
        grid

        subplot(514)
        semilogy([history.ne]);
        ylabel('|e|');
        grid

        subplot(515)
        plot([history.alpha]);
        xlabel('iteration');
        ylabel('\alpha');
        grid

        if nargout > 1
            histout = history;
        end
    end
end
