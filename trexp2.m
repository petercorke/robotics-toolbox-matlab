%TREXP2 matrix exponential for so(2) and se(2)
%
% R = TREXP2(S) is the matrix exponential (2x2) of the so(2) element S that
% yields a rotation matrix (2x2).
%
% R = TREXP2(S, THETA) as above, but so(2) rotation of S*THETA, S must be
% unit norm.
%
% R = TREXP2(THETA) as above, but the so(2) value is expressed as a scalar. 
%
% T = TREXP2(SIGMA) is the matrix exponential (3x3) of the se(2) element
% SIGMA that yields a homogeneous transformation  matrix (3x3).
%
% T = TREXP2(SIGMA, THETA) as above, but se(2) rotation of SIGMA*THETA, the
% rotation part of SIGMA must be unit norm.
%
% T = TREXP2(VW) as above, but the se(2) value is expressed as a vector VW
% (1x3).
%
% T = TREXP(VW, THETA) as above, but se(2) rotation of VW*THETA, the
% rotation part of VW must be unit norm.
%
% Notes::
% - Efficient closed-form solution of the matrix exponential for arguments that are
%   so(2) or se(2).
% - If theta is given then the first argument must be a unit vector or a
%   skew-symmetric matrix from a unit vector.
%
% References::
% - "Mechanics, planning and control"
%   Park & Lynch, Cambridge, 2016.
%
% See also trlog, trexp, Twist.

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
        elseif isvec(S,1)
            % input is a 1-vector
            w = S;
        else
            error('RTB:trexp2:badarg', 'expecting scalar or 2x2');
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