% Adds a helical loiter pattern to the trajectory.
function traj = trajectory_concatenateHelicalPattern( r, traj, sign, nTurns, deltaHperQuadrant)
% Unit vector pointing in the direction of the heading at the last
% waypoint:
v = unit(traj.waypoints{end} - traj.waypoints{end-1});

v(3) = 0;
v = unit(v);
vp = sign * cross(v, [0;0;1]);
vr = v*r;
vpr = vp*r;
dH = [0 0 -deltaHperQuadrant]';
for i=1:nTurns
    traj = trajectory_concatenateWaypoint(vr+vpr+ dH,traj);
    traj = trajectory_concatenateWaypoint(vpr-vr+ dH,traj);
    traj = trajectory_concatenateWaypoint(-vr-vpr+ dH,traj);
    traj = trajectory_concatenateWaypoint(-vpr+vr+ dH,traj);
    %traj = trajectory_concatenateWaypoint(vr,traj);
end

end