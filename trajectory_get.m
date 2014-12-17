% Computes position and position derivative on the trajectory w.r.t. the
% given total arc length.
function [p, dpds, ddpdds, ddpdds_sign, kappa] = trajectory_get( traj, s)

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

TperS = fnval(spline.TPerS, sLocal);
dpds = TperS * dpdt;
ddpdds = TperS * dpddt;

% Figure out sign of curvature in NED frame. 
ddpdds_sign = trajectory_getHorizontalCurvatureSign(dpds, ddpdds);
end
