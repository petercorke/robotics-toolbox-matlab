%TREXP2 matrix exponential for so(2) and se(2)
%
% R = trexp(so2) is the matrix exponential (2x2) that yields 
%
% R = trexp(wth) as above, rotation of |wth| about the vector wth
% R = trexp(w, theta) as above, rotation of theta about the unit-vector w
%
% T = trexp(se3) is
% T = trexp(Sth) as above
% T = trexp(S, theta) as above
% T = trexp(twist, theta)
%
% Notes::
% - Efficient closed-form solution of the matrix exponential for arguments that are
%   so(3) or se(3).
% - If theta is given then the first argument must be a unit vector or a
%   skew-symmetric matrix from a unit vector.
function T = trexp2(S, theta)

    
    if ishomog2(S) || isvec(S,3)
        % input is se(2)
        if nargin == 1
            if isvec(S,3)
                S = [skew(S(3)) S(1:2)'; 0 0 0];
            end
            T = expm(S);
        else
            if ishomog2(S)
                v = S(1:2,3);
                skw = S(1:2,1:2);
            else
                v = S(1:2)';
                skw = skew(S(3));
            end
            
            R = trexp2(skw, theta);
            
            t = (eye(2,2)*theta + (1-cos(theta))*skw + (theta-sin(theta))*skw^2)*v;
            
            T = rt2tr(R,t);
            %T = expm([S v; 0 0 0]*theta);
        end
    else
        % input is so(2)
        if isrot2(S)
            % input is 2x2 skew symmetric
            w = vex(S);
        elseif isvec(S,2)
            % input is a 3-vector
            w = S;
        end
        
        if nargin == 1
            %  theta is not given, extract it
            if norm(w) < 10*eps
                T = eye(2,2);
                return;
            end
            theta = norm(w);
            S = skew(unit(w));
        else
            if theta < 10*eps
                T = eye(2,2);
                return;
            end
            if ~isunit(w)
                error('RTB:trexp: angular velocity must be a unit vector');
            end
            S = skew(w);
        end
        
        T = eye(2,2) + sin(theta)*S + (1-cos(theta))*S^2;
        
    end
end