%GENCOORDS Vector of symbolic generalized coordinates
%
% Q = R.GENCOORDS is a vector (1xN) of symbols [q1 q2 ... qN].
%
% [Q,QD] = R.GENCOORDS as above but QD is a vector (1xN) of 
% symbols [qd1 qd2 ... qdN].
%
% [Q,QD,QDD] = R.GENCOORDS as above but QDD is a vector (1xN) of 
% symbols [qdd1 qdd2 ... qddN].
%
%
function [q,qd,qdd] = gencoords(r)

    if nargout > 0
        for j=1:r.n
            q(j) = sym( sprintf('q%d', j), 'real' );
        end
    end

    if nargout > 1
        for j=1:r.n
            qd(j) = sym( sprintf('qd%d', j), 'real' );
        end
    end

    if nargout > 2
        for j=1:r.n
            qdd(j) = sym( sprintf('qdd%d', j), 'real' );
        end
    end
