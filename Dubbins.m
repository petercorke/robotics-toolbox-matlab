% Dubbins path planner sample code
%
% P = Dubbins(q0, qf, maxc, dl) finds the shortest path between configurations
% q0 and qf where each is a vector [x y theta].  maxc is the maximum curvature
%
% The robot can only move forwards and the path consists of 3 segments
% which have zero or maximum curvature maxc.  There are discontinuities in 
% velocity and steering commands (cusps) at the transitions between the
% segments.
%
% Example::
%          q0 = [1 1 pi/4]'; qf = [1 1 pi]';
%          p = Dubbins(q0, qf, 1, 0.05)            
%          p.plot('circles', 'k--', 'join', {'Marker', 'o', 'MarkerFaceColor', 'k'});
%
% or alternatively
%
%        Dubbins.test
%
% References::
% - Dubins, L.E.
%   On Curves of Minimal Length with a Constraint on Average Curvature, and with Prescribed Initial and Terminal Positions and Tangents
%   American Journal of Mathematics. 79(3), July 1957, pp497?516.
%   doi:10.2307/2372560.
%
% Acknowledgement::
% - Based on python code from Python Robotics by Atsushi Sakai
%   https://github.com/AtsushiSakai/PythonRobotics
%
% See also Navigation, ReedsShepp

% each path is described by a 3-letter word.
% the algorithm finds a bunch of possible paths, then chooses the shortest
% one.  Each word is represented by a structure with fields:
% - word      a 3-letter sequence drawn from the letters LRLS
% - L         total path length
% - lengths   a 3-vector of lengths, signed to indicate the direction of
%             curvature
% - traj      a cell array of 3xN matrices giving the path for each segment


classdef Dubbins < handle
    properties
        best  % the best path
        words
        maxc
    end
    
    methods
        function obj = Dubbins(q0, qf, maxcurv, dl)
            
            obj.maxc = maxcurv;
            
            % return the word describing the shortest path
            obj.words = generate_path(q0, qf, maxcurv);
            
            if isempty(obj.words)
                error('no path');
            end
            
            % find shortest path
            [~,k] = min( [obj.words.L] );
            
            obj.best = obj.words(k);
            
            % add the trajectory
            obj.best = generate_trajectories(obj.best, maxcurv, dl, q0);
        end
        
        function p = path(obj)
            p = [obj.best.traj{:}]';
        end
        
        function show(obj)
            for w=obj.words
                fprintf('%s (%g): [%g %g %g]\n', w.word, w.L, w.lengths);
            end
        end
        
        function n = length(obj)
            n = length(obj.words);
        end
        
        
        
        function plot(obj, varargin)
            %DUBBINS.PLOT Plot Dubbins path
            %
            % DP.plot(OPTIONS) plots the optimal Dubbins path.
            %
            % Options::
            % 'circle',LS    Plot the full circle corresponding to each curved segment
            % 'join',LS      Plot a marker at the intermediate segment boundaries
            %
            % Notes::
            % - LS can be a simple LineSpec string or a cell array of Name,Value pairs.
            
            opt.circles = [];
            opt.join = [];
            
            opt = tb_optparse(opt, varargin);
            
            if ~ishold
                clf
            end
            hold on
            word = obj.best;
            
            for i=1:3
                
                color = 'b';
                if i == 1
                    x = word.traj{i}(1,:);
                    y = word.traj{i}(2,:);
                else
                    % ensure we join up the lines in the plot
                    x = [x(end) word.traj{i}(1,:)];
                    y = [y(end) word.traj{i}(2,:)];
                end
                
                if ~isempty(opt.join) && i<3
                    plot(x(end), y(end), opt.join{:});
                end
                if ~isempty(opt.circles)
                    T = SE2(word.traj{i}(:,1));
                    R = 1/obj.maxc;
                    switch word.word(i)
                        case 'L'
                            c = T*[0; R];
                        case 'R'
                            c = T*[0; -R];
                        case 'S'
                            continue
                    end
                    
                    plot_circle(c, R, opt.circles)
                    plot_point(c, 'k+')
                end
                
                plot(x, y, color, 'LineWidth', 2);
            end
            grid on; xlabel('X'); ylabel('Y')
            hold off
            axis equal
            title('Dubbins path')
        end
        
        function s = char(obj)
            s = '';
            s = strvcat(s, sprintf('Dubbins path:  %s, length %f', obj.best.word, obj.best.L));
            s = strvcat(s, sprintf(' segment lengths:  %f %f %f', obj.best.lengths));
        end
        
        function display(obj)
            disp( char(obj) );
        end
    end
    
    methods(Static)
        function test()
            maxcurv = 1;
            dl = 0.05;
            q0 = [1 1 pi/4]'; qf = [1 1 pi]';
            p = Dubbins(q0, qf, maxcurv, dl)
            
            p.plot('circles', 'k--', 'join', {'Marker', 'o', 'MarkerFaceColor', 'k'});
            hold on
            plot_vehicle(q0, 'r');
            plot_vehicle(qf, 'b');
            hold off
        end
    end
end

function out = generate_trajectories(word, maxc, d, q0)
    
    % initialize the configuration
    p0 = q0;
    
    % output struct is same as input struct, but we will add:
    %  - a cell array of trajectories
    %  - a vector of directions -1 or +1
    out = word;
    
    for i=1:3
        m = word.word(i);
        l = word.lengths(i);
        
        x = 0:d:abs(l);
        if x(end) ~= abs(l)
            x = [x abs(l)];
        end
        
        p = pathseg(x, m, maxc, p0);
        
        % add new fields to the struct
        
        if i == 1
            out.traj{i} = p;
        else
            % for subsequent segments skip the first point, same as last
            % point of previous segment
            out.traj{i} = p(:,2:end);
        end
        out.dir(i) = sign(l);
        
        % initial state for next segment is last state of this segment
        p0 = p(:,end);
    end
end

function q = pathseg(l, m, maxc, p0)
    q0 = p0(:);
    switch m
        case 'S'
            f = @(t,q) [cos(q(3)), sin(q(3)), 0]';
        case 'L'
            f = @(t,q) [cos(q(3)), sin(q(3)), maxc]';
        case 'R'
            f = @(t,q) [cos(q(3)), sin(q(3)), -maxc]';
    end
    [t,q] = ode45(f, l, q0);
    q = q';  % points are column vectors
end



function words = generate_path(q0, q1, maxc)
    % return a list of all possible words
    q0 = q0(:); q1 = q1(:);
    dq = q1 - q0;
    dth = dq(3);
    
    xy = rot2(q0(3))' * dq(1:2);
    x = xy(1); y = xy(2);
    
    
    d = norm([x y]) * maxc;
    %  [x y d]
    
    theta = mod2pi(atan2(y, x));
    alpha = mod2pi(-theta);
    beta = mod2pi(dth - theta);
    
    words = [];
    words = LSL(alpha, beta, d, 'LSL', words);
    words = RSR(alpha, beta, d, 'RSR', words);
    words = LSR(alpha, beta, d, 'LSR', words);
    words = RSL(alpha, beta, d, 'RSL', words);
    words = RLR(alpha, beta, d, 'RLR', words);
    words = LRL(alpha, beta, d, 'LRL', words);
    
    % account for non-unit curvature
    for i=1:numel(words)
        words(i).lengths = words(i).lengths / maxc;
        words(i).L = words(i).L / maxc;
    end
    
end % class Dubbins


%%
function owords = LSL(alpha, beta, d, word, words)
    sa = sin(alpha);
    sb = sin(beta);
    ca = cos(alpha);
    cb = cos(beta);
    c_ab = cos(alpha - beta);
    
    tmp0 = d + sa - sb;
    
    p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sa - sb));
    if p_squared < 0
        t = NaN; p = NaN; q = NaN;
    else
        
        tmp1 = atan2((cb - ca), tmp0);
        t = mod2pi(-alpha + tmp1);
        p = sqrt(p_squared);
        q = mod2pi(beta - tmp1);
    end
    
    owords = addpath(words, [t, p, q], word);
end

function owords = RSR(alpha, beta, d, word, words)
    sa = sin(alpha);
    sb = sin(beta);
    ca = cos(alpha);
    cb = cos(beta);
    c_ab = cos(alpha - beta);
    
    tmp0 = d - sa + sb;
    
    p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sb - sa));
    if p_squared < 0
        t = NaN; p = NaN; q = NaN;
    else
        tmp1 = atan2((ca - cb), tmp0);
        t = mod2pi(alpha - tmp1);
        p = sqrt(p_squared);
        q = mod2pi(-beta + tmp1);
    end
    
    owords = addpath(words, [t, p, q], word);
end

function owords = LSR(alpha, beta, d, word, words)
    sa = sin(alpha);
    sb = sin(beta);
    ca = cos(alpha);
    cb = cos(beta);
    c_ab = cos(alpha - beta);
    
    p_squared = -2 + (d * d) + (2 * c_ab) + (2 * d * (sa + sb));
    if p_squared < 0
        t = NaN; p = NaN; q = NaN;
    else
        p = sqrt(p_squared);
        tmp2 = atan2((-ca - cb), (d + sa + sb)) - atan2(-2.0, p);
        t = mod2pi(-alpha + tmp2);
        q = mod2pi(-mod2pi(beta) + tmp2);
    end
    
    owords = addpath(words, [t, p, q], word);
end

function owords = RSL(alpha, beta, d, word, words)
    sa = sin(alpha);
    sb = sin(beta);
    ca = cos(alpha);
    cb = cos(beta);
    c_ab = cos(alpha - beta);
    
    p_squared = (d * d) - 2 + (2 * c_ab) - (2 * d * (sa + sb));
    
    if p_squared < 0
        t = NaN; p = NaN; q = NaN;
    else
        p = sqrt(p_squared);
        tmp2 = atan2((ca + cb), (d - sa - sb)) - atan2(2.0, p);
        t = mod2pi(alpha - tmp2);
        q = mod2pi(beta - tmp2);
    end
    
    owords = addpath(words, [t, p, q], word);
end


function owords = RLR(alpha, beta, d, word, words)
    sa = sin(alpha);
    sb = sin(beta);
    ca = cos(alpha);
    cb = cos(beta);
    c_ab = cos(alpha - beta);
    
    tmp_rlr = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0;
    
    if abs(tmp_rlr) > 1
        t = NaN; p = NaN; q = NaN;
    else
        p = mod2pi(2 * pi - acos(tmp_rlr));
        t = mod2pi(alpha - atan2(ca - cb, d - sa + sb) + mod2pi(p / 2.0));
        q = mod2pi(alpha - beta - t + mod2pi(p));
        owords = addpath(words, [t, p, q], word);
    end
end

function owords = LRL(alpha, beta, d, word, words)
    sa = sin(alpha);
    sb = sin(beta);
    ca = cos(alpha);
    cb = cos(beta);
    c_ab = cos(alpha - beta);
    
    tmp_lrl = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (- sa + sb)) / 8.0;
    if abs(tmp_lrl) > 1
        t = NaN; p = NaN; q = NaN;
    else
        p = mod2pi(2 * pi - acos(tmp_lrl));
        t = mod2pi(-alpha - atan2(ca - cb, d + sa - sb) + p / 2.0);
        q = mod2pi(mod2pi(beta) - alpha - t + mod2pi(p));
    end
    
    owords = addpath(words, [t, p, q], word);
    
end

function owords = addpath(words, lengths, ctypes)
    owords = words;
    
    % create a struct to represent this segment
    word.word = ctypes;
    word.lengths = lengths;
    
    if any(isnan(lengths))
        return;
    end
    
    word.L = sum(abs(lengths));
    
    owords = [owords word];
    
end

function v = mod2pi(theta)
    %v = theta - 2.0 * pi * floor(theta / 2.0 / pi)
    v = mod(theta, 2*pi);
end

function v = pi_2_pi(angle)
    %v = (angle + pi) % (2 * math.pi) - math.pi
    v = mod(angle + pi, 2*pi) - pi;
end
