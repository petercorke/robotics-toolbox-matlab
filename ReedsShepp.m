% Reeds Shepp path planner sample code
%
% based on python code from Python Robotics by Atsushi Sakai(@Atsushi_twi)
%
% Peter 3/18
%
% Finds the shortest path between 2 configurations:
% - robot can move forward or backward
% - the robot turns at zero or maximum curvature
% - there are discontinuities in velocity and steering commands (cusps)
% to see what it does run
%
% >> ReedsShepp.test
%
% References::
% - Reeds, J. A.; Shepp, L. A.
%   Optimal paths for a car that goes both forwards and backwards.
%   Pacific J. Math. 145 (1990), no. 2, 367--393.
%   https://projecteuclid.org/euclid.pjm/1102645450

% each path is described by a 3-letter word.
% the algorithm finds a bunch of possible paths, then chooses the shortest
% one.  Each word is represented by a structure with fields:
% - word      a 3-letter sequence drawn from the letters LRLS
% - L         total path length
% - lengths   a 3-vector of lengths, signed to indicate the direction of
%             curvature
% - traj      a cell array of 3xN matrices giving the path for each segment
% - dir       the direction of travel: +1 or -1
%
% TODO: display all the solutions in one figure, as subplots

classdef ReedsShepp < handle
    properties
        best  % the best path
        words
        maxc
    end
    
    methods
        function obj = ReedsShepp(q0, qf, maxcurv, dl)
            
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
        
        function plot(obj, varargin)
            
            opt.circles = [];
            opt.join = [];
            
            opt = tb_optparse(opt, varargin);
            
            if ~ishold
                clf
            end
            hold on
            word = obj.best;
            
            for i=1:3
                
                if word.dir(i) > 0
                    color = 'b';
                else
                    color = 'r';
                end
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
                    c = T*[0; word.dir(i)*R];
                    
                    plot_circle(c, R, opt.circles)
                    plot_point(c, 'k+')
                end
                
                plot(x, y, color, 'LineWidth', 2);
            end
            grid on; xlabel('X'); ylabel('Y')
            hold off
            axis equal
            title('Reeds-Shepp path');
        end
        
        function s = char(obj)
            s = '';
            s = strvcat(s, sprintf('Reeds-Shepp path:  %s, length %f', obj.best.word, obj.best.L));
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
            q0 = [0 0 pi/4]'; qf = [0 0 pi]';
            p = ReedsShepp(q0, qf, maxcurv, dl)
            
            p.plot('circles', 'k--', 'join', {'Marker', 'o', 'MarkerFaceColor', 'k'});
        end
    end
end % class ReedsShepp

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
        
        x = [0:d:abs(l) abs(l)];
        
        p = pathseg(x, sign(l), m, maxc, p0);
        
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

function q = pathseg(l, dir, m, maxc, p0)
    q0 = p0(:);
    switch m
        case 'S'
            f = @(t,q) dir*[cos(q(3)), sin(q(3)), 0]';
        case {'L', 'R'}
            f = @(t,q) dir*[cos(q(3)), sin(q(3)), dir*maxc]';
    end
    [t,q] = ode45(f, l, q0);
    q = q';  % points are column vectors
end


function words = generate_path(q0, q1, maxc)
    % return a list of all possible words
    q0 = q0(:); q1 = q1(:);
    dq = q1 - q0;
    dth = dq(3);
    
    xy = rot2(q0(3))' * dq(1:2) * maxc;
    x = xy(1); y = xy(2);
    
    words = [];
    words = SCS(x, y, dth, words);
    words = CSC(x, y, dth, words);
    words = CCC(x, y, dth, words);
    
    % account for non-unit curvature
    for i=1:numel(words)
        words(i).lengths = words(i).lengths / maxc;
        words(i).L = words(i).L / maxc;
    end
    
end

%%
function owords = SCS(x, y, phi, words)
    
    words = SLS([ x  y  phi], 1, 'SLS', words);
    words = SLS([ x -y -phi], 1, 'SRS', words);
    
    owords = words;
end

function owords = CCC(x, y, phi, words)
    
    words = LRL([ x  y  phi],  1, 'LRL', words);
    words = LRL([-x  y -phi], -1, 'LRL', words);
    words = LRL([ x -y -phi],  1, 'RLR', words);
    words = LRL([-x -y  phi], -1, 'RLR', words);
    
    % backwards
    xb = x * cos(phi) + y * sin(phi);
    yb = x * sin(phi) - y * cos(phi);
    
    flip = [0 1 0; 1 0 0; 0 0 1];  % flip u and v
    
    words = LRL([ xb  yb  phi],  flip, 'LRL', words);
    words = LRL([-xb  yb -phi], -flip, 'LRL', words);
    words = LRL([ xb -yb -phi],  flip, 'RLR', words);
    words = LRL([-xb -yb  phi], -flip, 'RLR', words);
    
    owords = words;
end

function owords = CSC(x, y, phi, words)
    
    words = LSL([ x  y  phi],  1, 'LSL', words);
    words = LSL([-x  y -phi], -1, 'LSL', words);
    words = LSL([ x -y -phi],  1, 'RSR', words);
    words = LSL([-x -y  phi], -1, 'RSR', words);
    words = LSR([ x  y  phi],  1, 'LSR', words);
    words = LSR([-x  y -phi], -1, 'LSR', words);
    words = LSR([ x -y -phi],  1, 'RSL', words);
    words = LSR([-x -y  phi], -1, 'RSL', words);
    
    owords = words;
end

% requires LSL, LSR, SLS, LRL

%%


function owords = SLS(q, sign, word, words)
    x = q(1); y = q(2); phi = mod(q(3), 2*pi);
    
    if y > 0.0 && phi > 0.0 && phi < pi * 0.99
        xd = - y / tan(phi) + x;
        t = xd - tan(phi / 2.0);
        u = phi;
        v = norm( [(x - xd) y]) - tan(phi / 2.0);
        owords = addpath(words, sign*[t, u, v], word);
    elseif y < 0.0 && phi > 0.0 && phi < pi * 0.99
        xd = - y / tan(phi) + x;
        t = xd - tan(phi / 2.0);
        u = phi;
        v = -norm([(x - xd) y]) - tan(phi / 2.0);
        owords = addpath(words, sign*[t, u, v], word);
    else
        owords = words;
    end
end

function owords = LSL(q, sign, word, words)
    x = q(1); y = q(2); phi = mod(q(3), 2*pi);
    
    [t,u] = cart2pol(x - sin(phi), y - 1.0 + cos(phi));
    if t >= 0.0
        v = angdiff(phi - t);
        if v >= 0.0
            owords = addpath(words, sign*[t, u, v], word);
            return
        end
    end
    
    owords = words;
end

function owords = LRL(q, sign, word, words)
    x = q(1); y = q(2); phi = mod(q(3), 2*pi);
    
    [t1,u1] = cart2pol(x - sin(phi), y - 1.0 + cos(phi));
    
    if u1 <= 4.0
        u = -2.0 * asin(0.25 * u1);
        t = angdiff(t1 + 0.5 * u + pi);
        v = angdiff(phi - t + u);
        
        if t >= 0.0 && u <= 0.0
            owords = addpath(words, [t, u, v]*sign, word);
            return
        end
    end
    
    owords = words;
end

function owords = LSR(q, sign, word, words)
    x = q(1); y = q(2); phi = mod(q(3), 2*pi);
    
    [t1,u1] = cart2pol(x + sin(phi), y - 1.0 - cos(phi));
    u1 = u1^2;
    if u1 >= 4.0
        u = sqrt(u1 - 4.0);
        theta = atan2(2.0, u);
        t = angdiff(t1 + theta);
        v = angdiff(t - phi);
        
        if t >= 0.0 && v >= 0.0
            owords = addpath(words, sign*[t, u, v], word);
            return
        end
    end
    
    owords = words;
end


%%
function owords = addpath(words, lengths, ctypes)
    
    % create a struct to represent this segment
    word.word = ctypes;
    word.lengths = lengths;
    
    % check same path exist
    for p = words
        if strcmp(p.word, word.word)
            if sum(p.lengths) - sum(word.lengths) <= 0.01
                owords = words;
                return %not insert path
            end
        end
    end
    
    word.L = sum(abs(lengths));
    
    % long enough to add?
    if word.L >= 0.01
        owords = [words word];
    end
end
