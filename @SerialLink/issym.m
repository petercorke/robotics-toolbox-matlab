%SerialLink.issym Check if Link or SerialLink object is a symbolic model
%
% res = L.issym() is true if the Link L has symbolic parameters.
%
% res = R.issym() is true if the SerialLink manipulator R has symbolic parameters
%
%
% Authors::
% Jörn Malzahn   
% 2012 RST, Technische Universität Dortmund, Germany
% http://www.rst.e-technik.tu-dortmund.de    
function res = issym(l)

	res = issym(l.links(1));
	
end