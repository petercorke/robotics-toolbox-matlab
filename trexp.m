%TREXP matrix exponential for so(3) and se(3)
%
% R = TREXP(S) is the matrix exponential (3x3) of the so(3) element S that
% yields a rotation matrix (3x3). 
%
% R = TREXP(S, THETA) as above, but so(3)
% rotation of S*THETA, S must be unit norm.
%
% R = TREXP(W) as above, but the so(3) value is expressed as a vector W
% (1x3). 
%
% R = TREXP(W, THETA) as above, but so(3) rotation of W*THETA, wW
% must be unit norm.
%
% T = TREXP(SIGMA) is the matrix exponential (4x4) of the se(3) element SIGMA that
% yields a homogeneous transformation  matrix (4x4). 
%
% T = TREXP(SIGMA, THETA) as above, but se(3)
% rotation of SIGMA*THETA, the rotation part of SIGMA must be unit norm.
%
% T = TREXP(VW) as above, but the se(3) value is expressed as a vector VW
% (1x6). 
%
% T = TREXP(VW, THETA) as above, but se(3) rotation of VW*THETA, the
% rotation part of VW
% must be unit norm.
%
% Notes::
% - Efficient closed-form solution of the matrix exponential for arguments that are
%   so(3) or se(3).
% - If theta is given then the first argument must be a unit vector or a
%   skew-symmetric matrix from a unit vector.
%
% References::
% - "Mechanics, planning and control"
%   Park & Lynch, Cambridge, 2016.
%
% See also trlog, trexp2, Twist.
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
        else
            error('RTB:trexp:badarg', 'expecting 1x3 or 3x3');
        end
        
        if nargin == 1
            %  theta is not given, extract it
            if norm(w) < 10*eps
                T = eye(3,3);
                return;
            end
            theta = norm(w);
            S = skew(unit(w));
        else
            if theta < 10*eps
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