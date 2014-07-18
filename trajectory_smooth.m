function traj = trajectory_smooth( traj , maxc2)
%TRAJECTORY_SMOOTH Accepts a trajectory structure and updates the spline
%trajectory based on the waypoint trajectory.

% Check if input trajectory is valid:
validWP = {};
validWP{1} = traj.waypoints{1};
for i=2:length(traj.waypoints)-1
    isvalid = true;
    % Check if there are no waypoints in exactly the same place:
    if norm(traj.waypoints{i-1} - traj.waypoints{i}) < traj.resolution
        isvalid = false;
        disp([mfilename 'There are at least two waypoints extremely close to one another,'...
                        'aborting trajectory smoothing.']);
    end
    % Check if there are any three collinear waypoints, and, if so, remove
    % the middle one:
    if max(abs(unit(traj.waypoints{i} - traj.waypoints{i-1}) - unit(traj.waypoints{i+1} - traj.waypoints{i}))) < traj.resolution
        disp([mfilename '>> Collinear waypoints, removing the middle one']);
        isvalid = false;
    end    
    
    if isvalid
        validWP{end+1} = traj.waypoints{i};        
    end
end
validWP{end+1} = traj.waypoints{end};

traj.waypoints = validWP;

traj.splines = {};
% Iterate over waypoints and build spline segments:
for i=1:length(traj.waypoints)-2
    [s1, s2] = smoothSegment(traj.waypoints{i}, traj.waypoints{i+1}, traj.waypoints{i+2});
    traj.splines{end+1} = s1;
    traj.splines{end+1} = s2;
end

% Fill gaps between splines with straight line splines:
gapless = {};
% Check for gap between the first waypoint and the first control point
% of the first spline:
distance = norm(traj.splines{i}.p1 - traj.waypoints{1});
if distance > traj.resolution
    startpoint = traj.waypoints{1};
    endpoint = traj.splines{1}.p1;
    controlpoint = startpoint + 0.5 .* (endpoint - startpoint);
    line = spline_build(startpoint, controlpoint, controlpoint, endpoint);
    gapless{end+1} = line;
end
for i=1:length(traj.splines)-1
    % In any case, the first spline can already be added to the gapless
    % trajectory:
    gapless{end+1} = traj.splines{i};
    % If the distance between the last control point of this spline and the
    % first one of the following one exceeds the coordinate resolution,
    % connect the two splines by a straight line:
    distance = norm(traj.splines{i}.p4 - traj.splines{i+1}.p1);
    if distance > traj.resolution
        startpoint = traj.splines{i}.p4;
        endpoint = traj.splines{i+1}.p1;
        controlpoint = startpoint + 0.5 .* (endpoint - startpoint);
        line = spline_build(startpoint, controlpoint, controlpoint, endpoint);
        gapless{end+1} = line;
    end
end
gapless{end+1} = traj.splines{end};
% Check for gap between the last control point of the last spline and
% the last waypoint:
distance = norm(traj.splines{end}.p4 - traj.waypoints{end});
if distance > traj.resolution
    startpoint = traj.splines{end}.p4;
    endpoint = traj.waypoints{end};
    controlpoint = startpoint + 0.5 .* (endpoint - startpoint);
    line = spline_build(startpoint, controlpoint, controlpoint, endpoint);
    gapless{end+1} = line;
end
traj.splines = gapless;

% Do Sanity checks:
% Check if there are overlaps between splines, leading to infeasible turns.
% This can be verified by checking if the vectors formed by the last two
% control points of a spline and the vector formed by the first two control
% points of the following spline are collinear and pointing in opposite
% directions:
% 
for i=1:length(traj.splines)-1
    thisspline = traj.splines{i};
    nextspline = traj.splines{i+1};
    uthis = fflib_normalize(thisspline.p4 - thisspline.p3);
    unext = fflib_normalize(nextspline.p1 - nextspline.p2);
    % TODO Hugely arbitrary parameter to check for collinearity here; there
    % sure are better ways to do this.
    if norm(uthis + unext) > 1e6 * eps
       error([mfilename '>> Smoothing splines overlap, increase space or allow for smaller radii. Aborting.']); 
    end
end

disp([mfilename '>> Trajectory smoothing went just fine.']); 


% Accepts three consecutive waypoints, builds the smoothing spline segments
% that connects the three and returns them
    function [s1, s2] = smoothSegment(p1, p2, p3)
        p1_3D = p1;
        p2_3D = p2;
        p3_3D = p3;
        % Compute unit vectors of local cartesian system to be able to build
        % the spline in 2D:
        ux = unit(p2_3D-p1_3D);
        uz = unit(cross(ux, unit(p3_3D-p2_3D)));
        uy = unit(cross(uz, ux));
        % Transformation local2D <-> global 3D:
        T = [[ux; 0] [uy; 0] [uz; 0] [p1_3D; 1]];
        % Rotate points from 3D to local 2D:
        p1_2D = fcn3Dto2D(p1_3D);
        p2_2D = fcn3Dto2D(p2_3D);
        p3_2D = fcn3Dto2D(p3_3D);
        % Compute spline points.
        % Equations from
        % "3D Smooth Path Planning for a UAV in Cluttered Natural Environments"
        % Angle between the two line segments:
        u1 = unit(p1_2D-p2_2D);
        u2 = unit(p3_2D-p2_2D);
        ud = unit((p2_2D+u2) - (p2_2D+u1));
        angle = acos(dot(u1, u2));
        gamma = pi - angle;
        beta = gamma/2;
        d = (1.1128 * sin(beta)) / (maxc2 * (cos(beta)^2));
        hb = 0.346 * d;
        gb = 0.58 * hb;
        kb = 1.31 * hb * cos(beta);
        he = hb;
        ge = gb;
        ke = kb;
        B0_2D = p2_2D + d * u1;
        B1_2D = B0_2D - gb * u1;
        B2_2D = B1_2D - hb * u1;
        B3_2D = B2_2D + kb * ud;
        % The two splines are symmetric, thus the second one is computed
        % just like the first one:
        E0_2D = p2_2D + d * u2;
        E1_2D = E0_2D - ge * u2;
        E2_2D = E1_2D - he * u2;
        E3_2D = B3_2D;
        %plot2Dspline;
        % Build resulting spline sequence:
        s1 = spline_build(fcn2Dto3D(B0_2D), fcn2Dto3D(B1_2D), ...
            fcn2Dto3D(B2_2D), fcn2Dto3D(B3_2D));
        s2 = spline_build(fcn2Dto3D(E3_2D), fcn2Dto3D(E2_2D), ...
            fcn2Dto3D(E1_2D), fcn2Dto3D(E0_2D));
        
        % Rotates and translates a given 3D point to the local 2D system
        % used to build the spline.
        function p2D = fcn3Dto2D(p3D)
            p2D = T \ [p3D; 1];
            p2D = p2D(1:2);
        end
        
        % Rotates and translates a given 2D point from the local 2D system
        % used to build the spline to the 3D frame.
        function p3D = fcn2Dto3D(p2D)
            p3D = T * [p2D; 0; 1];
            p3D = p3D(1:3);
        end
        
        % Plots the spline in 2D and the involved control points and unit
        % vectors.
        function plot2Dspline
            hold off;
            col = distinguishable_colors(20);
            % Plot line points:
            plotPoint(p1_2D, 'P1', 1);
            hold on;
            plotPoint(p2_2D, 'P2',1);
            plotPoint(p3_2D, 'P3',1);
            % Plot spline control points
            plotPoint(B0_2D, 'B0',2);
            plotPoint(B1_2D, 'B1',2);
            plotPoint(B2_2D, 'B2',2);
            plotPoint(B3_2D, 'B3',2);
            plotPoint(E0_2D, 'E0',3);
            plotPoint(E1_2D, 'E1',3);
            plotPoint(E2_2D, 'E2',3);
            plotPoint(E3_2D, 'E3',3);
            arrow(p2_2D, p2_2D+u1);
            arrow(p2_2D, p2_2D+u2);
            %annotation('arrows',[p2_2D(1) u1(1)],[p2_2D(2) u1(2)]);
            
            plotbrowser on;
            function plotPoint(point, name, color)
                plot(point(1), point(2), 'Displayname', name, 'Marker', 'o', 'color', col(color, :));
            end
        end
    end

% Builds a spline structure from four given control points.
    function s = spline_build(p1, p2, p3, p4)
        s.p1 = p1;
        s.p2 = p2;
        s.p3 = p3;
        s.p4 = p4;
    end

end
