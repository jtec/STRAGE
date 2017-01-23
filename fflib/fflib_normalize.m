% Normalizes a given vector.
%
% \author Jan Bolting, ONERA/ISAE, 2014, j.bolting@isae.fr
%
function n = fflib_normalize( nin )
n = zeros(size(nin));
% Don't normalize zero vectors:
if max(abs(nin))>eps
    n = nin ./ norm(nin);
end
end

