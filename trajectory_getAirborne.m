% Re-parameterizes a given trajectory with the
% trajectory distance s.
function traj = trajectory_getAirborne( traj)

if isfield(traj, 'splines')
    for i=1:length(traj.splines)
        traj.splines{i} = trajectory_parameterizeWithArcLength(traj.splines{i});
    end
    
    % Compute total spline length:
    traj.sTotal = 0;
    for i=1:length(traj.splines)
        traj.sTotal = traj.sTotal + trajectory_getSplineLength(i, traj);
    end
end
end