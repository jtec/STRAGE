% Normalizes a given vector.
%
% \author Jan Bolting, ONERA/ISAE, 2014, j.bolting@isae.fr
%
function n = fflib_normalize( nin )
n = nin ./ norm(nin);
end

