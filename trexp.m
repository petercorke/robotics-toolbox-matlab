%TREXP matrix exponential for so(3) and se(3)
%
% R = trexp(so3) is the matrix exponential (3x3) that yields 
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
function T = trexp(S, theta)

    if ishomog(S) || isvec(S,6)
        % input is se(3)
        
        if nargin == 1
            if isvec(S,6)
                S = [skew(S(4:6)) S(1:3)'; 0 0 0 0];
            end
            T = expm(S);
        else
            if ishomog(S)
                [skw,v] = tr2rt(S);
            else
                v = S(1:3)';
                skw = skew(S(4:6));
            end
            
            R = trexp(skw, theta);
            t = (eye(3,3)*theta + (1-cos(theta))*skw + (theta-sin(theta))*skw^2)*v;
            
            T = rt2tr(R,t);
        end         
    else
        if isrot(S)
            % input is 3x3 skew symmetric
            w = vex(S);
        elseif isvec(S)
            % input is a 3-vector
            w = S;
        end
        
        if nargin == 1
            %  theta is not given, extract it
            theta = norm(w);
            S = skew(unit(w));
        else
            if norm(w) < 10*eps
                T = eye(3,3);
                return;
            end
            if ~isunit(w)
                error('RTB:trexp: angular velocity must be a unit vector');
            end
            S = skew(w);
        end
        
        T = eye(3,3) + sin(theta)*S + (1-cos(theta))*S^2;
        
    end
end