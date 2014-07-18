% Rotates a given trajectory by a given quaternion around a given point in
% the NED frame by all waypoints and spline control points.
function traj = trajectory_rotate( p, q, traj)
    for i=1:length(traj.waypoints)
        x = traj.waypoints{i}-p;
        traj.waypoints{i} = quatrotate(q, x')';
    end
    for i=1:length(traj.splines)
        traj.splines{i}.p1 = quatrotate(q, traj.splines{i}.p1')';
        traj.splines{i}.p2 = quatrotate(q, traj.splines{i}.p2')';
        traj.splines{i}.p3 = quatrotate(q, traj.splines{i}.p3')';
        traj.splines{i}.p4 = quatrotate(q, traj.splines{i}.p4')';
    end
end