% Builds a standard trajectory starting in mid-flight
tic;
% NOTE: Trajectory functions use an NED system.
traj = trajectory_build([0;0;-50]);
lStraightsegments = 100;
% Fly straight north
traj = trajectory_concatenateWaypoint([100*lStraightsegments;0;0],traj);
% Add curve
traj = trajectory_concatenateWaypoint([0;lStraightsegments/1;0],traj);
traj = trajectory_concatenateWaypoint([lStraightsegments;0;0],traj);
traj = trajectory_concatenateWaypoint([0;-lStraightsegments/1;0],traj);
traj = trajectory_concatenateWaypoint([lStraightsegments;0;0],traj);

% Climb by x m
gammaClimb = deg2rad(10);
hClimb = 10;
xClimb = hClimb/tan(gammaClimb);
traj = trajectory_concatenateWaypoint([xClimb;0;-hClimb],traj);
% Add straight segment
traj = trajectory_concatenateWaypoint([lStraightsegments;0;0],traj);
% Descend by x m
traj = trajectory_concatenateWaypoint([xClimb;0;hClimb],traj);
% Add straight segment
traj = trajectory_concatenateWaypoint([lStraightsegments;0;0],traj);

% Fly loiter circle:
traj = trajectory_concatenateLoiterPattern(lStraightsegments*2, traj);
% Add straight segment
traj = trajectory_concatenateWaypoint([lStraightsegments;0;0],traj);

% Smooth:
maxc2 = 0.04;
traj = trajectory_smooth(traj, maxc2);
traj = trajectory_rotate([0 0 0]', angle2quat(deg2rad(-30), 0, 0, 'ZYX'), traj);
% Prepare for simulation:
traj = trajectory_discretize(traj, 1.0);
save('trajectory/benchmarkTrajectory.mat', 'traj');
toc;
plotoptions.controlpointsvisible = false;
plotoptions.waypointsvisible = true;
plotoptions.discretepointsvisible = true;
trajectory_display(traj, plotoptions);
