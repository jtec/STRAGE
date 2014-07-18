% Computes position and position derivative on the trajectory w.r.t. the
% given total arc length.
function [p, dpds] = trajectory_get( traj, s)

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

% Get spline parameter t
t = fnval(spline.TofS, sLocal);
% Compute point
p = trajectory_evaluateBezier(spline, t);
% Do quick and dirty finite difference approximation of spline derivative
% w.r.t. s:
% Decide where to step:
ds = traj.resolution;
if spline.discrete.arclength(end) - sLocal > ds
   ds = traj.resolution;
else
   ds = - traj.resolution;
end
t1 = fnval(spline.TofS, sLocal + ds);
p1 = trajectory_evaluateBezier(spline, t1);
dpds = (p1-p)/ds;
end
