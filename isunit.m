%ISUNIT Test if vector has unit length
%
% ISUNIT(V) is true if the vector has unit length.
%
% Notes::
% - A tolerance of 100eps is used.
function s = isunit(v)
    s = abs(norm(v)-1) < 100*eps;
end
