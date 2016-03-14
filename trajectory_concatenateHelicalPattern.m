% Adds a helical loiter pattern to the trajectory.
function traj = trajectory_concatenateHelicalPattern( r, traj, sign, nTurns, deltaHperTurn)
% Unit vector pointing in the direction of the heading at the last
% waypoint:
if length(traj.waypoints) > 1
v = unit(traj.waypoints{end} - traj.waypoints{end-1});
else
 v = [1 0 0]';   
end
% Compute center of the circle:
c = traj.waypoints{end} + sign * r * unit(cross(v, [0; 0; 1]));
vr = traj.waypoints{end} - c;
n = 6;
dcm = angle2dcm(0,0,sign*deg2rad(360/n), 'XYZ');
for i=1:n*nTurns
    vr = dcm * vr;
    traj = trajectory_addWaypoint(c+vr + (i-1)*[0;0;-deltaHperTurn/n],traj);
end

end