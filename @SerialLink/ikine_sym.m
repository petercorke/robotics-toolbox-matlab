%IKINE_SYM  Symbolic inverse kinematics
%
% Q = R.IKINE_SYM(K, OPTIONS) is a cell array (Cx1) of inverse kinematic
% solutions of the SerialLink object ROBOT.  The cells of Q represent the
% different possible configurations.  Each cell of Q is a vector (Nx1), and
% element J is the symbolic expressions for the J'th joint angle.  The
% solution is in terms of the desired end-point pose of the robot which is
% represented by the symbolic matrix (3x4) with elements
%      nx ox ax tx
%      ny oy ay ty
%      nz oz az tz
% where the first three columns specify orientation and the last column
% specifies translation.
%
% K <= N can have only specific values:
%  - 2 solve for translation tx and ty
%  - 3 solve for translation tx, ty and tz
%  - 6 solve for translation and orientation
%
% Options::
%
% 'file',F    Write the solution to an m-file named F
%
% Example::
%
%         mdl_planar2
%         sol = p2.ikine_sym(2);
%         length(sol)
%         ans = 
%               2       % there are 2 solutions
%         s1 = sol{1}  % is one solution
%         q1 = s1(1);      % the expression for q1
%         q2 = s1(2);      % the expression for q2
%
% Notes::
% - Requires the Symbolic Toolbox for MATLAB.
% - This code is experimental and has a lot of diagnostic prints.
% - Based on the classical approach using Pieper's method.

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

function out = ikine_sym(srobot, N, varargin)
    
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
    opt = tb_optparse(opt, varargin);
    
    % make a symbolic representation of the passed robot
    srobot = sym(srobot);
    q = srobot.gencoords();

    % test N DOF has an allowable value
    switch N
        case 2
        case 3
        case 6
        otherwise
            error('RTB:ikine_sym:badarg', 'Can only solve for 2,3,6 DOF');
    end
    
    % define symbolic elements of the homogeneous transform
    syms nx ox ax tx
    syms ny oy ay ty
    syms nz oz az tz
    syms d3
    
    % inits
    Q = {};
    trigsubOld = [];
    trigsubNew = [];

    % loop over each joint variable
    for j=1:N
        fprintf('----- solving for joint %d\n', j);
        
        % create some equations to sift through
        [left,right] = pieper(srobot, j, 'left');
        
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
        
        % substitute sin/cos for preceding joint as S/C, essentially removes
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
                % has qj on the left and constant on the right
                if hasonly(left(i), j)
                    k = [k i];
                end
            end
            
            % hopefully we found two of them
            if length(k) < 2
                continue;
            end
            
            % we did, lets see if the sum square RHS is constant
            rhs = simplify(right(k(1))^2 + right(k(2))^2); % was simple
            if isconstant( rhs )
                % it is, let's sum and square the LHS
                fprintf('lets square and add %d %d\n', k);
                
                eq = simplify( expand( left(k(1))^2 + left(k(2))^2 ) ) - rhs; % was simple
            end
        end
        
        % expand the list of joint variable subsitutions
        fprintf('subs sin/cos q%d for S/C\n', j);
        trigsubOld = [trigsubOld mvar('sin(q%d)', j) mvar('cos(q%d)', j)];
        trigsubNew = [trigsubNew mvar('S%d', j) mvar('C%d', j)];
        
        if isempty(eq)
            fprintf('cant solve this equation');
            k
            left(k)==right(k)
            error('cant solve');
        end
        % now solve the equation
        if srobot.links(j).isrevolute()
            % for revolute joint it will be a trig equation, do we know how to solve it?
            Q{j} = solve_joint(eq, j );
            if isempty(Q)
                warning('cant solve this kind of equation');
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
    
    % create a list of simplifications
    %  substitute S^2 = 1-C^2, S^4=(1-C^2)^2
    tsubOld = [];
    tsubNew = [];
    for j=1:N
        tsubOld = [tsubOld mvar('S%d', j)^2 mvar('S%d', j)^4];
        tsubNew = [tsubNew 1-mvar('C%d', j)^2 (1-mvar('C%d', j)^2)^2];
    end
    
    for j=1:N
        for k=1:5
            % seem to need to iterate this, not quite sure why
            Q{j} = simplify( expand( subs(Q{j}, tsubOld, tsubNew) ) );
        end
    end

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
% Judicious choice of the equations can lead to joint solutions

function [L,R] = pieper(robot, n, which)
    
    if nargin < 3
        which = 'left';
    end
    
    syms nx ox ax tx real
    syms ny oy ay ty real
    syms nz oz az tz real
        
    T = [nx ox ax tx
        ny oy ay ty
        nx oz az tz
        0  0  0  1 ];
    
    T = inv(robot.base) * T * inv(robot.tool);
    
    q = robot.gencoords();
    
    
    % Create the symbolic A matrices
    for j=1:robot.n
        A{j} = robot.links(j).A(q(j));
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
% The result is a cell array of solutions.
%
% The equations must be of the form:
%  A cos(qJ) + B sin(qJ) = 0
%  A cos(qJ) + B sin(qJ) = C
%
% where A, B, C are arbitrarily complex expressions.  qJ can be the only
% joint variable in the expression.

function s = solve_joint(eq, j)
    
    sinj = mvar('sin(q%d)', j);
    cosj = mvar('cos(q%d)', j);
    
    A = getcoef(eq, cosj);
    B = getcoef(eq, sinj);
    
    if isempty(A) || isempty(B)
        warning('don''t know how to solve this kind of equation');
    end
    
    C = -simplify(eq - A*cosj - B*sinj);  % was simple
    
    if C == 0
        % A cos(q) + B sin(q) = 0
        s(2) = atan2(A, -B);
        s(1) = atan2(-A, B);
    else
        % A cos(q) + B sin(q) = C
        r = sqrt(A^2 + B^2 - C^2);
        phi = atan2(A, B);
        
        s(2) = atan2(C, r) - phi;
        s(1) = atan2(C, -r) - phi;
    end
    if nargout == 0
        try
            eval(s)
        catch
            s
        end
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
        v = sym( sprintf(fmt, varargin{:}) );
        
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
