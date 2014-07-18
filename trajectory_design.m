% Design considerations for trajectory generator.
% #Objective:
%   - A trajectory that is c2 continuous
%   - It shall be possible to limit the curvature
% #Design
%   - The trajectory is a series ob Bézier spirals, as described in 
%     "3D Smooth Path Planning for a UAV in Cluttered Natural Environments"
%   - The trajectory is first constructed as a series of lines and smoothed
%     in a second step. If new waypoints are added, the smoothed trajetory
%     is build anew.
%   - The trajectory is defined in a local NED frame.
%
% #Implementation
%   - Functions that allow to construct the straight-lines trajectory
%       - structure, series of waypoints, preserves familiar waypoint paradigm
%   - Function that smoothes the waypoint trajectory
%       - augments waypoint structure by series of Bézier spline structures
%         and returns it
%   - Function that accepts a spline trajectory and plots it
%   - Function names: trajectory_<name>, e.g. trajectory_smooth(wptrajectory)

% Start trajectory at (0,0,0), landing strip runs from west to east.
traj = trajectory_build([0;0;0]);
% Add x meters to accelerate:
traj = trajectory_concatenateWaypoint([10;0;0],traj);
% Climb to 10 meters, climb angle = 20 degrees
gamma = deg2rad(20);
hClimb = 10;
xClimb = hClimb/tan(gamma);
traj = trajectory_concatenateWaypoint([xClimb;0;-hClimb],traj);
% Fly straight:
traj = trajectory_concatenateWaypoint([30;0;0],traj);
% Turn to the right:
%traj = trajectory_concatenateWaypoint([0;-50;0],traj);

% Smooth:
maxc2 = 0.01;
traj = trajectory_smooth(traj, maxc2);
traj = trajectory_discretize(traj, 1);

plotoptions.controlpointsvisible = false;
plotoptions.waypointsvisible = true;
plotoptions.discretepointsvisible = true;
trajectory_display(traj, plotoptions);
