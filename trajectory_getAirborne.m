% Re-parameterizes a given trajectory with the
% trajectory distance s.
function traj = trajectory_getAirborne( traj)
for i=1:length(traj.splines)
    traj.splines{i} = trajectory_parameterizeWithArcLength(traj.splines{i});
end

end
