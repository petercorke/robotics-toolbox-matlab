


function sol = solve1(j, robot, left, right)
    % look for a constant element on the right
        which = NaN;
        for i=1:12
            if any(ismember(findq(left(i)), j)) && isempty(findq(right(i)))
                which = i;
                break;
            end
        end
        which
        if isnan(which)
            sol = [];
            return;
        end
        eq = left(which) - right(which);
        
        if robot.links(j).isrevolute()
            if hasCj(eq, j) && hasSj(eq, j)
                sol = solve_joint(eq, j );
            else
                warning('eq does not have both Sj and Cj');
            end
        else
            disp('prismatic case')
            q = sym( sprintf('q%d', j) );
            sol = solve( eq == 0, q);
        end
        
        
end

function sol = solve2(j, robot, left, right)
    % look for a simple trig term on the right
    
    [bs,ss] = isSj(right, j);
    [bc,sc] = isCj(right, j);
    
    if ~(any(bs) && any(bc))
        sol = [];
        return;
    end
    
    ks = find(bs, 1);
    kc = find(bc, 1);
    
    eq_sin = ss(ks) * left(ks);
    eq_cos = sc(kc) * left(kc);
    
    sol = atan2(eq_sin, eq_cos);
    
    disp('solve2')
    sol
    
end

function s = solve_joint(eq, j)
    % solve an equation for q_j which is implicitly equal to zero
    
    eq
    
    sinj = sym( sprintf('sin(q%d)', j) );
    cosj = sym( sprintf('cos(q%d)', j) );

    A = getcoef(eq, sinj);
    B = getcoef(eq, cosj);
    
    C = simple(eq - A*sinj - B*cosj);
    
    if C == 0
        % A sin(q) + B cos(q) = 0
        s = atan2(-B, A);
    else
        % A sin(q) + B cos(q) + C = 0
        r = sqrt(A^2 + B^2);
        phi = atan2(B, A);
        sin_sum = -C/r;
        cos_sum = sqrt(1- sin_sum^2);
        
        s(1) = atan2(-C, sqrt(A^2 + B^2 - C^2)) - phi;
        s(2) = atan2(-C, -sqrt(A^2 + B^2 - C^2)) - phi;
    end

end

function coef = getcoef(eq, trig)
    z = children( collect(eq, trig) );
    z = children( z(1) );
    coef = z(1);
end


function b = hasCj(eq, j)
    %hasCj Test if expression contains cos(q_j)
%
% works for an array of equations
    
    Cj = sym( sprintf('cos(q%d)', j) );
    
    for i=1:numel(eq)
        
        e = children( collect(eq(i), Cj) );
        e = children(e(1));
        b(i) = numel(e) == 2 && isequaln(e(2), Cj);
    end
end

function b = hasSj(eq, j)
    %hasSj Test if expression contains sin(q_j)
%
% works for an array of equations
    
    Sj = sym( sprintf('sin(q%d)', j) );
    
    for i=1:numel(eq)
        
        e = children( collect(eq(i), Sj) );
        e = children(e(1));
        b(i) = numel(e) == 2 && isequaln(e(2), Sj);
    end
end


function [b,sgn] = isCj(eq, j)
    %isCj Test if expression is cos(q_j)
%
% works for an array of equations
    
    Cj = sym( sprintf('cos(q%d)', j) );
    
    for i=1:numel(eq)
        
        if isequaln(eq(i), Cj);
            b(i) = true;
            sgn(i) = 1;
        elseif isequaln(eq(i), -Cj);
            b(i) = true;
            sgn(i) = -1;
        else
            b(i) = false;
            sgn(i) = 0;
        end
    end
end

function [b,sgn] = isSj(eq, j)
    %isSj Test if expression is sin(q_j)
%
% works for an array of equations
    
    Sj = sym( sprintf('sin(q%d)', j) );
    
    for i=1:numel(eq)
        
        if isequaln(eq(i), Sj);
            b(i) = true;
            sgn(i) = 1;
        elseif isequaln(eq(i), -Sj);
            b(i) = true;
            sgn(i) = -1;
        else
            b(i) = false;
            sgn(i) = 0;
        end
    end
end
    
% function solve_sincos(j, eqns)
%     for i=1:2
%         t = trigvar(eqns(i));
%         if t(1) == 'S'
%             s = solve(eqns(i), t);
%         elseif t(1) == 'C'
%             c = solve(eqns(i), t);
%         end
%     end
%     
%     s/c
% end
% 
% function trigvar(eq)
%     for var=symvar(s)
%         varname = char(var);
%         if varname(1) == 'S' || varname(1) == 'C'
%             q = [q str2num(varname(2:end))];
%         end
%     end
% end
% 
% 
% function c = allkids(eq)
%     c = [];
%     kids = children(eq);
%     if length(kids) == 1
%         c = kids;
%         return;
%     else
%         for i=1:length(kids)
%             c = [c allkids(kids(i))];
%         end
%     end
% end
%    
% 
% function out = removeTrig(eq, j)
%     old = [sym(sprintf('sin(q%d)', j)), sym(sprintf('cos(q%d)', j))];
%     new = [sym(sprintf('S%d', j)), sym(sprintf('C%d', j))];
%     
%     out = subs(eq, old, new);
% end
% 
% function out = addTrig(eq)
% end
% 
%     
% %     % simplify list, remove repeats
% %     eqns2(1) = eqns(1);
% %     
% %     for i=2:length(eqns)
% %         for j=1:length(eqns2)
% %             if any(logical( eqns(i) == eqns2)) == 0
% %                 eqns2 = [eqns2; eqns(i)];  % add it to the list
% %                 break;
% %             end
% %         end
% %     end
% %     
% %     fprintf('%d unique equations\n', length(eqns2));
% %     eqns = eqns2;
% %     
% %     eqns
% %     
% %     
% %     for k=1:length(eqns)
% %         eqn(k).e = eqns(k);
% %         eqn(k).qs = findq( eqns(k) );
% %     end
% %     for k=1:length(qs)
% %         fprintf('%d:  ', k);
% %         fprintf('%d, ', eqn(k).qs);
% %         fprintf('\n');
% %     end
% %    
% % 
% %         complexity(eqn)
% % 
% %         
% %     s1 = jointsolve(eqn, 1);
% %     
% %     s2 = jointsolve(eqn, 2, false);
% %     
% %     
% %     s3 = jointsolve(eqn, 3);
% % 
% % 
% %     %matlabFunction([q1 = s1, q2 = s2, q3 = s3], 'File', 'ik.m');
% %     
% % 
% %     
% %     
% %     %k = upto(qs, 3);
% %     %eee = eqns(k)
% %    
% % 
% % end
% 
% function soln = jointsolve(eqn, j, require_p)
%     
%     if nargin < 3
%         require_p = true;
%     end
%     
%     fprintf('----------- joint %d ------------\n', j);
%     
%     
%     % get all equations involving upto joint j
%     k = upto(eqn, j);
%     eqn = eqn(k);
%     
%     if require_p
%         ps = [];
%         for k=1:length(eqn)
%             z = findp( eqn(k).e );
%             if ~isempty(z)
%                 ps = eqn(k).e;
%                 break;
%             end
%         end
%     else
%         complexity(eqn)
%         ps = eqn(1).e
%     end
%     
%     if isempty(ps)
%         warning('no good equation found to solve for joint %d', j);
%     end
%     
%     soln = solve(ps, sprintf('q%d', j), 'Real', true);
%     %simplify([soln)
% end
% 
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
% 
% function p = findp(s)
%     
%     p = [];
%     
%     for var=symvar(s)
%         varname = char(var);
%         if varname(1) == 'p'
%             p = [p varname(2:end)];
%         end
%     end
% end
% 
% 
% 
% function k = upto(eqn, n)
%     k = [];
%     for i=1:length(eqn)
%         if max(eqn(i).qs) == n
%             k = [k i];
%         end
%     end
% end
% 
% 
% function c = complexity(eqn)
%     for i=1:length(eqn)
%         c(i) = complexity2(eqn(i).e);
%     end
% end
% 
% function c = complexity2(e)
%     kids = children(e);
%     if length(kids) == 1
%         c = 1;
%         return
%     else
%         c = 0;
%         for kid=kids
%             c = c + complexity2(kid);
%         end
%     end
% end
   
