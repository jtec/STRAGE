% Re-parameterizes a given trajectory with the
% trajectory distance s.
function traj = trajectory_getAirborne( traj)

output_verbose = strcmpi(traj.news, 'verbose');
output_minimal = strcmpi(traj.news, 'minimal');


if isfield(traj, 'splines')
    for i=1:length(traj.splines)
        traj.splines{i} = trajectory_parameterizeWithArcLength(traj.splines{i});
        if output_verbose
            disp([mfilename '>> Parameterizing with arc length: spline ' num2str(i) '/' num2str(length(traj.splines))]);
        end
    end
    
    % Compute total spline length:
    traj.sTotal = 0;
    for i=1:length(traj.splines)
        traj.sTotal = traj.sTotal + trajectory_getSplineLength(i, traj);
    end
    if output_verbose || output_minimal
        disp([mfilename '>> Computed total spline length: ' num2str(traj.sTotal) ' m']);
    end
end
end