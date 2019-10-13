%IKINE_SYM  Symbolic inverse kinematics
%
% Q = R.IKINE_SYM(K, OPTIONS) is a cell array (Cx1) of inverse kinematic
% solutions of the SerialLink object ROBOT.  The cells of Q represent the
% solutions for each joint, ie. Q{1} is the solution for joint 1.  A
% cell may contain an array of solutions. The solution is expressed in terms
% of other joint angles and elements of the desired end-point pose which is
% represented by the symbolic matrix (3x4) with elements
%      nx ox ax tx
%      ny oy ay ty
%      nz oz az tz
% where the first three columns specify orientation and the last column
% specifies translation.
%
% K <= N is the number of joint angles solved for.
%
% Options::
%
% 'file',F    Write the solution to an m-file named F
% 'Tpost',T   Add a symbolic 4x4 matrix T to the end of the chain
%
% Example::
%
%         mdl_planar2
%         sol = p2.ikine_sym(2);
%         length(sol)
%
%         q1 = sol{1}   % are the solution for joint 1
%         q2 = sol{2}   % is the solution for joint 2
%         length(q1)
%         ans =
%               2     % there are 2 solutions for this joint
%         q1(1)       % one solution for q1
%         q1(2);      % the other solution for q1
%
% Notes::
% - ignores tool and base transforms.
%
% References::
% - Robot manipulators: mathematics, programming and control
%   Richard Paul, MIT Press, 1981.
% - The kinematics of manipulators under computer control, 
%   D.L. Pieper, Stanford report AI 72, October 1968.
%
% Notes::
% - Requires the MATLAB Symbolic Math Toolbox.
% - This code is experimental and has a lot of diagnostic prints.
% - Based on the classical approach using Pieper's method.


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

function out = ikine_sym(robot, N, varargin)
    
    %
    % Given a robot model the following steps are performed:
    % 1. Convert model to symbolic form
    % 2. Find relevant trig equations and solve them for joint angles
    % 3. Write an M-file to implement the solution
    %      xikine(T)
    %      xikine(T, S) where S is a 3 vector with elements 1 or 2 to select
    %       the first or second solution for the corresponding joint.
    %
    % TODO:
    %  - handle the wrist joints, only first 3 joints so far
    %  - handle base and tool transforms
    
    opt.file = [];
    opt.Tpost = [];
    opt = tb_optparse(opt, varargin);
    
    % make a symbolic representation of the passed robot
    srobot = SerialLink(robot);  % make a deep copy
    srobot = sym(srobot);  % convert to symbolic
    q = srobot.gencoords();

    
    % define symbolic elements of the homogeneous transform
    syms nx ox ax tx real
    syms ny oy ay ty real
    syms nz oz az tz real
    syms d4 real
    
    % inits
    Q = {};
    trigsubOld = [];
    trigsubNew = [];

    % loop over each joint variable
    for j=1:N
        fprintf('----- solving for joint %d\n', j);
        
        % create some equations to sift through
        [left,right] = pieper(srobot, j, 'left', opt.Tpost);
        
        % decide which equations to look at
        if j <= 3
            % for first three joints only focus on translational part
            left = left(1:3, 4); left = left(:);
            right = right(1:3, 4); right = right(:);
        else
            % for last three joints only focus on rotational part
            left = left(1:3, 1:3); left = left(:);
            right = right(1:3, 1:3); right = right(:);
        end
        
        % substitute sin/cos for preceding joint as Sj/Cj, essentially removes
        % the joint variables from the equations and treats them as constants.
        if ~isempty(trigsubOld)
            left = subs(left, trigsubOld, trigsubNew);
            right = subs(right, trigsubOld, trigsubNew);
        end
        
        % then simplify the LHS
        %   do it after the substitution to prevent sum of angle terms being introduced
        left = simplify(left);
        
        % search for a solveable equation:
        %    function of current joint variable on the LHS
        %    constant element on the RHS
        k = NaN;
        for i=1:length(left)
            if hasonly(left(i), j) && isconstant(right(i))
                k = i;
                break;
            end
        end
        
        eq = [];
        
        if ~isnan(k)
            % create the equation to solve: LHS-RHS == 0
            eq = left(k) - right(k);
        else
            % ok, we weren't lucky, try another strategy
            
            % find all equations:
            %    function of current joint variable on the LHS
            
            k = [];
            for i=1:length(left)
                % has qj on the left
                if hasonly(left(i), j)
                    k = [k i];
                end
            end
            
            % hopefully we found at least two of them
            if length(k) < 2
                continue;
            end
            
            % we did, lets see if the sum square RHS is constant
            for kk = nchoosek(k, 2)'
                fprintf('let''s square and add RHS %d %d\n', kk);
                rhs = simplify(right(kk(1))^2 + right(kk(2))^2); % was simple
                if isconstant( rhs )
                    eq = simplify( expand( left(kk(1))^2 + left(kk(2))^2 ) ) - rhs;
                    break
                end
            end
            if isempty(eq)
                fprintf('** can''t solve this equation, out of options');
                k
                left(k)==right(k)
                error('can''t solve this equation');
            end
        end
        
        % expand the list of joint variable subsitutions
        fprintf('subs sin/cos q%d for S/C\n', j);
        trigsubOld = [trigsubOld mvar('sin(q%d)', j) mvar('cos(q%d)', j)];
        trigsubNew = [trigsubNew mvar('S%d', j) mvar('C%d', j)];
        
        % now solve the equation
        if srobot.links(j).isrevolute()
            % for revolute joint it will be a trig equation, do we know how to solve it?
            Q{j} = solve_joint(eq, j );
            if isempty(Q)
                warning('RTB:ikine_sym', 'can''t solve this kind of equation');
            end
        else
            fprintf('prismatic case\n')
            q = sym( sprintf('q%d', j) );
            Q{j} = solve( eq == 0, q);
        end
    end
    
    % final simplification
    %  get rid of C^2+S^2 and C^4, S^4 terms
    fprintf('**final simplification pass\n')
    
    Q = simplify_powers(Q, N, trigsubOld, trigsubNew);

    % Q is a cell array of equations for joint variables
    if nargout > 0
        out = Q;
    end
    
    if ~isempty(opt.file)
        fprintf('**generate MATLAB code\n')
        gencode(Q);
    end
end


%PIEPER Return a set of equations using Pieper's method
%
% [L,R] = pieper(robot, n, which)
%
% If robot has link matrix A1 A2 A3 A4 then returns 12 equations from equating the coefficients of
%
%  A1' T = A2 A3 A4     n=1, which='left'
%  A2' A1' T = A3 A4    n=2, which='left'
%  A3' A2' A1' T = A4   n=3, which='left'
%
%  T A4' = A1 A2 A3     n=1, which='right'
%  T A4' A3' = A1 A2    n=2, which='right'
%  T A4' A3' A2' = A1   n=3, which='right'
%
% A' denotes inversion not transposition
%
% Judicious choice of the equations can lead to joint solutions

function [L,R] = pieper(robot, n, which, Tpost)
    
    if nargin < 3
        which = 'left';
    end
    if nargin < 4 || isempty(Tpost)
        Tpost = transl(zeros(robot.n, 3));
    end
    assert(n <= robot.n, 'RTB:ikine_sym:badarg', 'N is greater than number of joints');
    
    syms nx ox ax tx real
    syms ny oy ay ty real
    syms nz oz az tz real
        
    T = [nx ox ax tx
        ny oy ay ty
        nx oz az tz
        0  0  0  1 ];
    
    T = inv(robot.base.T) * T * inv(robot.tool.T);
    
    q = robot.gencoords();
    
    
    % Create the symbolic A matrices
    for j=1:robot.n
        A{j} = robot.links(j).A(q(j)).T * Tpost(:,:,j);
    end
    
    switch which
        case 'left'
            left = T;
            for j=1:n
                left = inv(A{j}) * left ;
            end
            
            right = eye(4,4);
            for j=n+1:robot.n
                right = right * A{j};
            end
            
        case 'right'
            left = T;
            for j=1:n
                left = left * inv(A{robot.n-j+1});
            end
            
            right = eye(4,4);
            for j=1:(robot.n-n)
                right = right * A{j};
            end
    end
    
    %     left = simple(left);
    %     right = simple(right);
    
    if nargout == 0
        left == right
    elseif nargout == 1
        L = left;
    elseif nargout == 2
        L = left;
        R = right;
    end
end

%SOLVE_JOINT Solve a trigonometric equation
%
% S = SOLVE_JOINT(EQ, J) solves the equation EQ=0 for the joint variable qJ.
% The result is a vector of symbolic solutions.
%
% The equations must be of the form:
%  A cos(qJ) + B sin(qJ) = 0
%  A cos(qJ) + B sin(qJ) = C
%
% where A, B, C are arbitrarily complex expressions.  qJ can be the only
% joint variable in the expression.
%
% Notes::
% - In general there are two solutions, but if A^2+B^2-C^2 = 0, then only one
%   solution is returned.
% - The one solution case may not be detected symbolically, in which case
%   the two returned solutions will have the same value after numerical
%   substitution
% - The symbolic solution may not be evaluteable numerically with
%   certain parameter values, ie. if A^2+B^2-C^2 < 0

function s = solve_joint(eq, j)
    
    % see http://petercorke.com/wordpress/solving-trigonometric-equations
    
    sinj = mvar('sin(q%d)', j);
    cosj = mvar('cos(q%d)', j);
    
    A = getcoef(eq, cosj);
    B = getcoef(eq, sinj);
    
    if isempty(A) || isempty(B)
        warning('don''t know how to solve this kind of equation');
    end
    
    C = -simplify(eq - A*cosj - B*sinj);  % was simple
    
    if C == 0
        fprintf('C == 0\n');
        % A cos(q) + B sin(q) = 0
        s(2) = atan2(A, -B);
        s(1) = atan2(-A, B);
    else
        fprintf('C != 0\n');
        % A cos(q) + B sin(q) = C
        r = sqrt(A^2 + B^2 - C^2);
%         phi = atan2(A, B);
%         
%         s(2) = atan2(C, r) - phi;
%         s(1) = atan2(C, -r) - phi;
        if r == 0
            s = atan2(B*C, A*C) 
        else
            s(1) = atan2(B*C + A*r, A*C - B*r);
            s(2) = atan2(B*C + A*r, A*C - B*r);
        end
    end
    
    if nargout == 0
        try
            eval(s)
        catch
            s
        end
    end
end

function Qout = simplify_powers(Q, N, trigsubOld, trigsubNew)
        % create a list of simplifications
    %  substitute S^2 = 1-C^2, S^4=(1-C^2)^2
    fprintf('power simplification');
    tsubOld = [];
    tsubNew = [];
    for j=1:N
        tsubOld = [tsubOld mvar('S%d', j)^2 mvar('S%d', j)^4];
        tsubNew = [tsubNew 1-mvar('C%d', j)^2 (1-mvar('C%d', j)^2)^2];
    end
    
    for j=1:length(Q)
        for k=1:5
            % seem to need to iterate this, not quite sure why
            Q{j} = simplify( expand( subs(Q{j}, tsubOld, tsubNew) ) );
        end
        % subs Sx, Cx to sin(qx), cos(qx)
        Qout{j} = simplify( subs(Q{j}, trigsubNew, trigsubOld) );
    end
end


%MVAR Create a symbolic variable
%
% V = MVAR(FMT, ARGS) is a symbolic variable created using SPRINTF
%
% eg. mvar('q%d', j)
%
% The symbolic is explicitly declared to be real.

function v = mvar(fmt, varargin)

    if isempty(strfind(fmt, '('))
        % not a function
        v = sym( sprintf(fmt, varargin{:}), 'real' );
    else
        v = str2sym( sprintf(fmt, varargin{:}) );
        
    end
end

%HASONLY Determine if an expression contains only certain joint variables
%
% S = HASONLY(E L) is true if the joint variables (q1, q2 etc.) in the expression E
% are listed in the vector L.
%
% Eg. hasonly('sin(q1)*cos(q2)*cos(q4)', [1 2 3]) -> true
% Eg. hasonly('sin(q1)*cos(q2)*cos(q4)', [1]) -> false

function s = hasonly(eq, j)
    
    q = findq(eq);
    if isempty(q)
        s = false;
    else
        s = all(ismember(j, findq(eq)));
    end
end

%ISCONSTANT Determine if an expression is free of joint variables
%
% S = ISCONSTANT(E) is true if the expression E contains no joint variables such
% q1, q2 etc.

function s = isconstant(eq)
    s = isempty(findq(eq));
end

%FINDQ Find the joint variables in expression
%
% Q = FINDQ(E) returns a list of integers indicating the joint variables found
% in the expression E.  For instance an instance of 'q1' would cause a 1 to be
% returned and so on.
%
% Eg. findq('sin(q1)*cos(q2)+S3') -> [1 2]

function q = findq(s)
    
    q = [];
    
    for var=symvar(s)
        if isempty(var)
            break
        end
        varname = char(var);
        if varname(1) == 'q'
            q = [q str2num(varname(2:end))];
        end
    end
end

function coef = getcoef(eq, trig)
    z = children( collect(eq, trig) );
    z = children( z(1) );
    coef = z(1);
end

% Output a joint expression to a file

function s = gencode(Q, filename)
    
    function s = G(s, fmt, varargin)
        s = strvcat(s, sprintf(fmt, varargin{:}));
    end

    s = 'function q = xikine(T, sol)';
    s = G(s, '  if nargin < 2; sol = ones(1, %d); end', length(Q));
    s = G(s, '  px = T(1,4); py = T(2,4); pz = T(3,4);');
    
    for j=1:3
        Qj = Q{j};   % cast it to subclass
        if length(Qj) == 1
            s = G(s, '  q(%d) = %s', j, matgen2(Qj));
        elseif length(Qj) == 2
            s = G(s, '  if sol(%d) == 1', j);
            s = G(s, '    q(%d) = %s', j, matgen2(Qj(1)));
            s = G(s, '  else');
            s = G(s, '    q(%d) = %s', j, matgen2(Qj(2)));
            s = G(s, '  end');
            
            
        end
        
        
        s = G(s, '  S%d = sin(q(%d));', j, j);
        s = G(s, '  C%d = cos(q(%d));', j, j);
        s = G(s, ' ');
        
        
    end
    s = G(s, 'end');
    
    fp = fopen(filename, 'w');
    for i=1:numrows(s)
        fprintf(fp, '%s\n', deblank(s(i,:)));
    end
    fclose(fp);
    
end

% Generate MATLAB code from an expression
%
% Requires a bit of a hack, a subclass of sym (sym2) to do this

function s = matgen2(e)
    
    s = matgen(sym2(e));
    
    k = strfind(s, '=');
    s = deblank( s(k+2:end) );
end
