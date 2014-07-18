%% Builds a trajectory for which smoothing should fail due to minimum radii
% that are too large for the given waypoint sequence, leading to
% overlapping splines

traj = trajectory_build([0;0;0]);
% Add U-Shape:
traj = trajectory_concatenateWaypoint([100;0;0],traj);
traj = trajectory_concatenateWaypoint([0;100;0],traj);
traj = trajectory_concatenateWaypoint([-100;0;0],traj);

% Smooth:
maxc2 = 0.01;
traj = trajectory_smooth(traj, maxc2);
plotoptions.controlpointsvisible = false;
plotoptions.waypointsvisible = true;
trajectory_display(traj, plotoptions);
