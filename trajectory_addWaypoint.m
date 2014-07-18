function traj = trajectory_addWaypoint( wp, traj)
%TRAJECTORY_ADDWAYPOINT Adds the given waypoint to the given trajectory and
%returns it.
traj.waypoints{end+1} = wp;
end