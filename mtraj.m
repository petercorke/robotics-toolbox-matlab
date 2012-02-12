%MTRAJ Multi-axis trajectory between two points
%
% [Q,QD,QDD] = MTRAJ(TFUNC, Q0, QF, M) is a multi-axis trajectory (MxN) varying
% from state Q0 (1xN) to QF (1xN) according to the scalar trajectory function 
% TFUNC in M steps. Joint velocity and acceleration can be optionally returned as 
% QD (MxN) and QDD (MxN) respectively.  The trajectory outputs have one row per 
% time step, and one column per axis.
%
% The shape of the trajectory is given by the scalar trajectory function TFUNC
%      [S,SD,SDD] = TFUNC(S0, SF, M);
% and possible values of TFUNC include @lspb for a trapezoidal trajectory, or
% @tpoly for a polynomial trajectory.
%
% [Q,QD,QDD] = MTRAJ(TFUNC, Q0, QF, T) as above but specifies the trajectory 
% length in terms of the length of the time vector T (Mx1).
%
% Notes::
% - If no output arguments are specified Q, QD, and QDD are plotted.
% - When TFUNC is @tpoly the result is functionally equivalent to JTRAJ except 
%   that no initial velocities can be specified. JTRAJ is computationally a little
%   more efficient.
%
% See also JTRAJ, MSTRAJ, LSPB, TPOLY.

% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
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

function [S,Sd,Sdd] = mtraj(tfunc, q0, qf, M)

    if ~isa(tfunc, 'function_handle')
        error('first argument must be a function handle');
    end

    M0 = M;
    if ~isscalar(M)
        M = length(M);
    end
    if numcols(q0) ~= numcols(qf)
        error('must be same number of columns in q0 and qf')
    end

    s = zeros(M, numcols(q0));
    sd = zeros(M, numcols(q0));
    sdd = zeros(M, numcols(q0));

    for i=1:numcols(q0)
        % for each axis
        [s(:,i),sd(:,i),sdd(:,i)] = tfunc(q0(i), qf(i), M);
    end

% - If no output arguments are specified S, SD, and SDD are plotted 
%   against time.

    switch nargout
        case 0
            clf

            if isscalar(M0)
                t = [1:M0]';
            else
                t = M0;
            end
            subplot(311)
            plot(t, s); grid; ylabel('s');

            subplot(312)
            plot(t, sd); grid; ylabel('sd');
            
            subplot(313)
            plot(t, sdd); grid; ylabel('sdd');
            if ~isscalar(M0)
                xlabel('time')
            else
                for c=get(gcf, 'Children');
                    set(c, 'XLim', [1 M0]);
                end
            end
            shg
        case 1
            S = s;
        case 2
            S = s;
            Sd = sd;
        case 3
            S = s;
            Sd = sd;
            Sdd = sdd;
    end
