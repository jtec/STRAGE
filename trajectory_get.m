% Computes 
% - position 
% - a unit vector tangent to the spline
% - the curvature vector 
%  w.r.t. the  given total arc length.
function [p, uTangent, Kappa] = trajectory_get( traj, s)

% Figure out on which spline we are:
sTotal = 0;
spline = [];
sLocal = 0;
for i=1:length(traj.splines)
    sTotal = sTotal + traj.splines{i}.discrete.arclength(end);
    if sTotal > s
        spline = traj.splines{i};
        sLocal = s - (sTotal -  traj.splines{i}.discrete.arclength(end));
        break;
    end
end

if isempty(spline)
    error([mfilename '>> Given arc length exceeds the maximum arc length of this trajectory, returning end point'])
    spline = traj.splines{end};
    sLocal = spline.discrete.arclength(end);
end

% Get spline parameter t
t = fnval(spline.TofS, sLocal);
% Compute point, first and second derivative w.r.t. s:
[p, dpdt, dpddt] = trajectory_evaluateBezier(spline, t);

kappa = cross(dpdt, dpddt) / norm(dpdt)^3;
Kappa = cross(kappa, unit(dpdt));

uTangent = unit(dpdt);

% Figure out sign of curvature in NED frame. 
end
