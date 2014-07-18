function traj = trajectory_build( wp0 )
%TRAJECTORY_BUILD Returns an unsmoothed trajectory, containing only one
%waypoint at the given p0.
%   Use this function to obtain a trajectory container structure. The next
%   steps would be to add waypoints to this trajectory and to smooth it.
%   Waypoints are defined in a local NED system.
traj.waypoints{1} = wp0;
% Position resolution: 1mm
traj.resolution = 1e-3;
end