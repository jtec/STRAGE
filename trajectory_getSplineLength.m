function l = trajectory_getSplineLength( i, traj )
    l = traj.splines{i}.discrete.arclength(end);
end

