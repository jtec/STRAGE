function traj = trajectory_concatenateWaypoint( wp, traj)
%TRAJECTORY_ADDWAYPOINT Adds the given waypoint to the given trajectory and
%returns it.
% Note that the position is given as relative to the last waypoint.
traj.waypoints{end+1} = traj.waypoints{end} + wp;
end