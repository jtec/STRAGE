function l = trajectory_getCumSplineLength( i, traj )
l = 0;
for k=1:i
    l = l + traj.splines{k}.discrete.arclength(end);
end
end

