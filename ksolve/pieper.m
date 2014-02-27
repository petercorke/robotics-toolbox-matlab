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
    
    syms nx ox ax px real
    syms ny oy ay py real
    syms nz oz az pz real
    
    T = [nx ox ax px
        ny oy ay py
        nx oz az pz
        0  0  0  1 ];
    
    T = inv(robot.base) * T * inv(robot.tool)
    
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