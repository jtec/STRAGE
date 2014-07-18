% Sets matrix elements that are smaller than the machine precision eps to
% zero.
% \author Jan Bolting, ONERA/ISAE, 2014, j.bolting@isae.fr
%
function aOut = fflib_removeSubEps(a)
a(abs(a)<eps) = 0;
aOut = a;
end
