function spline = trajectory_parameterizeWithArcLength( spline )
% Approximates the function relating arc length to spline parameter t
% [0,1].

% Approximate arc length if necessary:
if ~isfield(spline, 'discrete')
    spline = trajectory_approximateSplineLength(spline);
end
% Get points equidistant in t:
% Fit spline to arclength(t):
spline.SofT = csapi(spline.discrete.ts, spline.discrete.arclength);
% Fit spline to t(arcLength):
spline.TofS = csapi(spline.discrete.arclength, spline.discrete.ts);

end