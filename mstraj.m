%MSTRAJ Multi-segment multi-axis trajectory
%
% TRAJ = MSTRAJ(P, QDMAX, TSEG, Q0, DT, TACC, OPTIONS) is a trajectory
% (KxN) for N axes moving simultaneously through M segment.  Each segment
% is linear motion and polynomial blends connect the segments.  The axes
% start at Q0 (1xN) and pass through M-1 via points defined by the rows of
% the matrix P (MxN), and finish at the point defined by the last row of P.
% The  trajectory matrix has one row per time step, and one column per
% axis.  The number of steps in the trajectory K is a function of the
% number of via points and the time or velocity limits that apply.
%
% - P (MxN) is a matrix of via points, 1 row per via point, one column 
%   per axis.  The last via point is the destination.
% - QDMAX (1xN) are axis speed limits which cannot be exceeded,
% - TSEG (1xM) are the durations for each of the K segments
% - Q0 (1xN) are the initial axis coordinates
% - DT is the time step
% - TACC (1x1) this acceleration time is applied to all segment transitions
% - TACC (1xM) acceleration time for each segment, TACC(i) is the acceleration 
%   time for the transition from segment i to segment i+1.  TACC(1) is also 
%   the acceleration time at the start of segment 1.
%
% TRAJ = MSTRAJ(SEGMENTS, QDMAX, Q0, DT, TACC, QD0, QDF, OPTIONS) as above
% but additionally specifies the initial and final axis velocities (1xN).
%
% Options::
% 'verbose'    Show details.
%
% Notes::
% - Only one of QDMAX or TSEG should be specified, the other is set to [].
% - If no output arguments are specified the trajectory is plotted.
% - The path length K is a function of the number of via points, Q0, DT
%   and TACC.
% - The final via point P(end,:) is the destination.
% - The motion has M segments from Q0 to P(1,:) to P(2,:) ... to P(end,:).
% - All axes reach their via points at the same time.
% - Can be used to create joint space trajectories where each axis is a joint
%   coordinate.
% - Can be used to create Cartesian trajectories where the "axes"
%   correspond to translation and orientation in RPY or Euler angle form.
%
% See also MTRAJ, LSPB, CTRAJ.



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

function [TG, taxis]  = mstraj(segments, qdmax, tsegment, q, dt, Tacc, varargin)


    ns = numrows(segments);
    nj = numcols(segments);

    if ~isempty(qdmax) && ~isempty(tsegment)
        error('Can only specify one of qdmax or tsegment');
    end
    if isempty(qdmax) && isempty(tsegment)
        error('Must specify one of qdmax or tsegment');
    end

    [opt,args] = tb_optparse([], varargin);

    if length(args) > 0
        qd0 = args{1};
    else
        qd0 = zeros(1, nj);
    end
    if length(args) > 1
        qdf = args{2};
    else
        qdf = zeros(1, nj);
    end

    % set the initial conditions
    q_prev = q;
    qd_prev = qd0;

    clock = 0;      % keep track of time
    arrive = [];    % record planned time of arrival at via points

    tg = [];
    taxis = [];

    for seg=1:ns
        if opt.verbose
            fprintf('------------------- segment %d\n', seg);
        end

        % set the blend time, just half an interval for the first segment

        if length(Tacc) > 1
            tacc = Tacc(seg);
        else
            tacc = Tacc;
        end

        tacc = ceil(tacc/dt)*dt;
        tacc2 = ceil(tacc/2/dt) * dt;
        if seg == 1
            taccx = tacc2;
        else
            taccx = tacc;
        end

        % estimate travel time
        %    could better estimate distance travelled during the blend
        q_next = segments(seg,:);    % current target
        dq = q_next - q_prev;    % total distance to move this segment

        %% probably should iterate over the next section to get qb right...
        % while 1
        %   qd_next = (qnextnext - qnext)
        %   tb = abs(qd_next - qd) ./ qddmax;
        %   qb = f(tb, max acceleration)
        %   dq = q_next - q_prev - qb
        %   tl = abs(dq) ./ qdmax;

        if ~isempty(qdmax)
            % qdmax is specified, compute slowest axis

            qb = taccx * qdmax / 2;        % distance moved during blend
            tb = taccx;

            % convert to time
            tl = abs(dq) ./ qdmax;
            %tl = abs(dq - qb) ./ qdmax;
            tl = ceil(tl/dt) * dt;

            % find the total time and slowest axis
            tt = tb + tl;
            [tseg,slowest] = max(tt);
            taxis(seg,:) = tt;

            % best if there is some linear motion component
            if tseg <= 2*tacc
                tseg = 2 * tacc;
            end
        elseif ~isempty(tsegment)
            % segment time specified, use that
            tseg = tsegment(seg);
            slowest = NaN;
        end

        % log the planned arrival time
        arrive(seg) = clock + tseg;
        if seg > 1
            arrive(seg) = arrive(seg) + tacc2;
        end

        if opt.verbose
            fprintf('seg %d, slowest axis %d, time required %.4g\n', ...
                seg, slowest, tseg);
        end

        %% create the trajectories for this segment

        % linear velocity from qprev to qnext
        qd = dq / tseg;

        % add the blend polynomial
        qb = jtraj(q, q_prev+tacc2*qd, 0:dt:taccx, qd_prev, qd);
        tg = [tg; qb(2:end,:)];

        clock = clock + taccx;     % update the clock

        % add the linear part, from tacc/2+dt to tseg-tacc/2
        for t=tacc2+dt:dt:tseg-tacc2
            s = t/tseg;
            q = (1-s) * q_prev + s * q_next;       % linear step
            tg = [tg; q];
            clock = clock + dt;
        end

        q_prev = q_next;    % next target becomes previous target
        qd_prev = qd;
    end
    % add the final blend
    qb = jtraj(q, q_next, 0:dt:tacc2, qd_prev, qdf);
    tg = [tg; qb(2:end,:)];

    % plot a graph if no output argument
    if nargout == 0
        t = (0:numrows(tg)-1)'*dt;
        clf
        plot(t, tg, '-o');
        hold on
        plot(arrive, segments, 'bo', 'MarkerFaceColor', 'k');
        hold off
        grid
        xlabel('time');
        xaxis(t(1), t(end))
    else 
        TG = tg;
    end
