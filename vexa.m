% See also skewa, vex.

function s = vexa(Omega)
        if ~all(size(Omega) == [4 4])
        error('RTB:vexa:badarg', 'expecting a 4x4 matrix');
        end
    
        s = [transl(Omega); vex(Omega(1:3,1:3))];
end