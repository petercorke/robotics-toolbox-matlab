%LSPB  Linear segment with parabolic blend
%
% [S,SD,SDD] = LSPB(S0, SF, M) is a scalar trajectory (Mx1) that varies
% smoothly from S0 to SF in M steps using a constant velocity segment and
% parabolic blends (a trapezoidal velocity profile).  Velocity and
% acceleration can be optionally returned as SD (Mx1) and SDD (Mx1)
% respectively.
%
% [S,SD,SDD] = LSPB(S0, SF, M, V) as above but specifies the velocity of 
% the linear segment which is normally computed automatically.
%
% [S,SD,SDD] = LSPB(S0, SF, T) as above but specifies the trajectory in 
% terms of the length of the time vector T (Mx1).
%
% [S,SD,SDD] = LSPB(S0, SF, T, V) as above but specifies the velocity of 
% the linear segment which is normally computed automatically and a time
% vector.
%
% LSPB(S0, SF, M, V) as above but plots S, SD and SDD versus time in a single
% figure.
%
% Notes::
% - If M is given
%   - Velocity is in units of distance per trajectory step, not per second.
%   - Acceleration is in units of distance per trajectory step squared, not
%     per second squared. 
% - If T is given then results are scaled to units of time.
% - The time vector T is assumed to be monotonically increasing, and time
%   scaling is based on the first and last element.
% - For some values of V no solution is possible and an error is flagged.
%
% References::
% - Robotics, Vision & Control, Chap 3,
%   P. Corke, Springer 2011.
%
% See also TPOLY, JTRAJ.



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
% add a 'dt' option, to convert to everything to units of seconds

function [s,sd,sdd] = lspb(q0, q1, t, V)

    t0 = t;
    if isscalar(t)
        t = (0:t-1)';
    else
        t = t(:);
    end
    plotsargs = {'Markersize', 16};

    tf = max(t(:));

    if nargin < 4
        % if velocity not specified, compute it
        V = (q1-q0)/tf * 1.5;
    else
        V = abs(V) * sign(q1-q0);
        if abs(V) < abs(q1-q0)/tf
            error('V too small');
        elseif abs(V) > 2*abs(q1-q0)/tf
            error('V too big');
        end
    end

    if q0 == q1
        s = ones(size(t)) * q0;
        sd = zeros(size(t));
        sdd = zeros(size(t));
        return
    end

    tb = (q0 - q1 + V*tf)/V;
    a = V/tb;

    p = zeros(length(t), 1);
    pd = p;
    pdd = p;
    
    for i = 1:length(t)
        tt = t(i);

        if tt <= tb
            % initial blend
            p(i) = q0 + a/2*tt^2;
            pd(i) = a*tt;
            pdd(i) = a;
        elseif tt <= (tf-tb)
            % linear motion
            p(i) = (q1+q0-V*tf)/2 + V*tt;
            pd(i) = V;
            pdd(i) = 0;
        else
            % final blend
            p(i) = q1 - a/2*tf^2 + a*tf*tt - a/2*tt^2;
            pd(i) = a*tf - a*tt;
            pdd(i) = -a;
        end
    end

    switch nargout
        case 0
            if isscalar(t0)
                % for scalar time steps, axis is labeled 1 .. M
                xt = t+1;
            else
                % for vector time steps, axis is labeled by vector M
                xt = t;
            end

            clf
            subplot(311)
            % highlight the accel, coast, decel phases with different
            % colored markers
            hold on
            %plot(xt, p);
            k = t<= tb;
            plot(xt(k), p(k), 'r.-', plotsargs{:});
            k = (t>=tb) & (t<= (tf-tb));
            plot(xt(k), p(k), 'b.-', plotsargs{:});
            k = t>= (tf-tb);
            plot(xt(k), p(k), 'g.-', plotsargs{:});
            grid; ylabel('$s$', 'FontSize', 16, 'Interpreter','latex');

            hold off

            subplot(312)
            plot(xt, pd, '.-', plotsargs{:});
            grid;
            if isscalar(t0)
                ylabel('$ds/dk$', 'FontSize', 16, 'Interpreter','latex');
            else
                ylabel('$ds/dt$', 'FontSize', 16, 'Interpreter','latex');
            end
            
            subplot(313)
            plot(xt, pdd, '.-', plotsargs{:});
            grid;
            if isscalar(t0)
                ylabel('$ds^2/dk^2$', 'FontSize', 16, 'Interpreter','latex');
            else
                ylabel('$ds^2/dt^2$', 'FontSize', 16, 'Interpreter','latex');
            end
            
            if ~isscalar(t0)
                xlabel('t (seconds)')
            else
                xlabel('k (step)');
                for c=findobj(gcf, 'Type', 'axes')
                    set(c, 'XLim', [1 t0]);
                end
            end
            shg
        case 1
            s = p;
        case 2
            s = p;
            sd = pd;
        case 3
            s = p;
            sd = pd;
            sdd = pdd;
    end
