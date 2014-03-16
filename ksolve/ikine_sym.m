function Q = ikine_sym(robot)
    
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
    
    % solve inverse kinematics
    syms nx ox ax px
    syms ny oy ay py
    syms nz oz az pz
    syms d3
    
    
    robot = sym(robot);
    
    q = robot.gencoords();
    
    
    Q = {};
    
    trigsubOld = [];
    trigsubNew = [];
    %for j=1:robot.n
    for j=1:3
        
        
        fprintf('----- joint %d\n', j);
        
        % create some equations to sift through
        [left,right] = pieper(robot, j, 'left');
        
        % for first three joints only focus on translational part
        left = left(1:3, 4); left = left(:);
        right = right(1:3, 4); right = right(:);
        
        
        
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
            rhs = simple(right(k(1))^2 + right(k(2))^2);
            if isconstant( rhs )
                % it is, let's sum and square the LHS
                fprintf('lets square and add %d %d\n', k);
                
                eq = simple( expand( left(k(1))^2 + left(k(2))^2 ) ) - rhs;
            end
        end
        
        % expand the list of joint variable subsitutions
        fprintf('subs sin/cos q%d for S/C\n', j);
        trigsubOld = [trigsubOld mvar('sin(q%d)', j) mvar('cos(q%d)', j)];
        trigsubNew = [trigsubNew mvar('S%d', j) mvar('C%d', j)];
        
        % now solve the equation
        if robot.links(j).isrevolute()
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
    for j=1:3
        tsubOld = [tsubOld mvar('S%d', j)^2 mvar('S%d', j)^4];
        tsubNew = [tsubNew 1-mvar('C%d', j)^2 (1-mvar('C%d', j)^2)^2];
    end
    
    for j=1:3
        for k=1:5
            % seem to need to iterate this, not quite sure why
            Q{j} = simplify( expand( subs(Q{j}, tsubOld, tsubNew) ) );
        end
    end
    
    
    % Q is a cell array of equations for joint variables
    
    % create xikine.m
    fprintf('**generate MATLAB code\n')
    
    gencode(Q);
    
