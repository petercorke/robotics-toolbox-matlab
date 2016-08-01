% See also vexa, skew.
function Omega = skewa(s)
    s  = s(:);
    switch length(s)
        case 3
            Omega = [skew(s(3)) s(1:2); 0 0 0];
            
        case 6
            Omega = [skew(s(4:6)) s(1:3); 0 0 0 0];
            
        otherwise
            error('RTB:skewa:badarg', 'expecting a 3- or 6-vector');
    end
end