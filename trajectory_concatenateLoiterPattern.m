function traj = trajectory_concatenateLoiterPattern( r, traj)
%TRAJECTORY_ADDWAYPOINT Adds a loiter pattern, resembling a circle with
%the given radius.
% Unit vector pointing in the direction of the heading at the last
% waypoint:
v = unit(traj.waypoints{end} - traj.waypoints{end-1});
% loiter at constant altitude:
v(3) = 0;
v = unit(v);
vp = cross(v, [0;0;1]);
vr = v*r;
vpr = vp*r;
traj = trajectory_concatenateWaypoint(vr+vpr,traj);
traj = trajectory_concatenateWaypoint(vpr-vr,traj);
traj = trajectory_concatenateWaypoint(-vr-vpr,traj);
traj = trajectory_concatenateWaypoint(-vpr+vr,traj);
traj = trajectory_concatenateWaypoint(vr,traj);
end