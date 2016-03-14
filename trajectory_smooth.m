function traj = trajectory_smooth( traj, envelope, c2maxFix)

%TRAJECTORY_SMOOTH Accepts a trajectory structure and updates the spline
%trajectory based on the waypoint trajectory.
output_verbose = strcmpi(traj.news, 'verbose');
output_minimal = strcmpi(traj.news, 'minimal');

% Check if input trajectory is valid:
validWP = {};
validWP{1} = traj.waypoints{1};
% FIXME Indexing like this keeps somewayoints from being checked!
for i=2:length(traj.waypoints)-1
    isvalid = true;
    % Check if there are any consecutive waypoints in exactly the same place:
    if norm(traj.waypoints{i+1} - traj.waypoints{i}) < traj.resolution
        isvalid = false;
        error([mfilename 'There are at least two waypoints extremely close to one another,'...
            'aborting trajectory smoothing.']);
    end
    if isvalid
        validWP{end+1} = traj.waypoints{i};
    end
end
validWP{end+1} = traj.waypoints{end};

traj.waypoints = validWP;

traj.splines = {};
% Iterate over waypoints and build spline segments:
dwp = [];
for i=1:length(traj.waypoints)-2
    % Check if the waypoints are collinear:
    if max(abs(unit(traj.waypoints{i+1} - traj.waypoints{i}) - unit(traj.waypoints{i+2} - traj.waypoints{i}))) < traj.resolution
        disp([mfilename '>> Three collinear waypoints, not smoothing']);
    else
        if output_verbose
            disp([mfilename '>> Smoothing corner ' num2str(i) '/' num2str(length(traj.waypoints)-2)]);
        end
        [s1, s2] = smoothSegment(traj.waypoints{i}, traj.waypoints{i+1}, traj.waypoints{i+2});
        traj.splines{end+1} = s1;
        traj.splines{end+1} = s2;
    end
end

% Detect closed trajectory: if the last and any of the other waypoints are close,
% build additional spline to transition between the end and this waypoint:
dwp = [];
for i=1:length(traj.waypoints)-1
    dwp(end+1) = norm(traj.waypoints{i} - traj.waypoints{end});
end
[minD, ind] = min(dwp);
if minD < traj.resolution
    error([mfilename '>> Closed trajectories not implemented, avoid putting more than on waypoint too close (< trajectory.resolution) together.']);
    [s1, s2] = smoothSegment(traj.waypoints{end-1}, traj.waypoints{end}, traj.waypoints{ind+1});
    traj.splines{end+1} = s1;
    traj.splines{end+1} = s2;
end

% Fill gaps between splines with straight line splines:
gapless = {};
% Check for gap between the first waypoint and the first control point
% of the first spline:
if length(traj.waypoints) > 2
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
    % If there are only two waypoints, connect them by a straight line spline:
else
    distance = norm(traj.waypoints{1} - traj.waypoints{2});
    if distance > traj.resolution
        startpoint = traj.waypoints{1};
        endpoint = traj.waypoints{2};
        controlpoint = startpoint + 0.5 .* (endpoint - startpoint);
        line = spline_build(startpoint, controlpoint, controlpoint, endpoint);
        gapless{end+1} = line;
    end
    
end

traj.splines = gapless;

% Do Sanity checks:
% TODO This should be done during smoothing.
% Check if there are overlaps between splines, leading to infeasible turns.
% This can be verified by checking if the vectors formed by the last two
% control points of a spline and the vector formed by the first two control
% points of the following spline are collinear and pointing in opposite
% directions:
%
success = true;
for i=1:length(traj.splines)-1
    thisspline = traj.splines{i};
    nextspline = traj.splines{i+1};
    uthis = fflib_normalize(thisspline.p4 - thisspline.p3);
    unext = fflib_normalize(nextspline.p1 - nextspline.p2);
    % TODO Hugely arbitrary parameter to check for collinearity here; there
    % sure are better ways to do this.
    if norm(uthis + unext) > 1e6 * eps
        disp([mfilename '>> Smoothing splines overlap, increase space or allow for smaller radii.' ... 
                            ' Removing all following splines.']);
        traj.splines = traj.splines(1:i);
        success = false;
        break;
    end
end

if success
   disp([mfilename '>> Trajectory smoothing went just fine.']); 
else
   disp([mfilename '>> Trajectory smoothing failed, have a look at error messages.']);     
end

% Builds a smoothing spline between three consecutive waypoints,
% checking for aerodynamic constraints.
    function [s1, s2] = smoothSegment(p1, p2, p3)
        % Build initial spline
        % TODO Use approximate analytic solution of maximum curvature
        % parameter as initial value to accelerate convergence.
        % c2max = 0.5:-0.05:0.1;
        % If maximum curvature is provides, use it:
        if nargin > 2
            c2max = c2maxFix;
        else
            c2max = 0.05;
        end
        deltalift = zeros(size(c2max));
        
        for ic2max=1:length(c2max);
            [s1, s2] = buildSegment(p1, p2, p3, c2max(ic2max));
            segment.splines{1} = s1;
            segment.splines{2} = s2;
            deltalift(ic2max) = getLiftForceDelta(segment);
        end
        bp = 0;
        function deltaLiftForce = getLiftForceDelta(seg)
            env = envelope;
            % Sample splines, compute forces for each sample,
            for ispline=1:2
                seg.splines{ispline} = trajectory_parameterizeWithArcLength(seg.splines{ispline});
                ds = seg.splines{ispline}.discrete.arclength(end)/100;
                s = 0:ds:seg.splines{ispline}.discrete.arclength(end)/2;
                seg.resolution = ds;
                cost = s;
                for iS=1:length(s)
                    [p, uTangent, Kappa] = trajectory_get(seg, s(iS));
                    % Build unit vectors of local coordinate system:
                    uX = unit(uTangent);
                    uZ = unit(cross(unit(p3-p2), unit(p1-p2)));
                    uY = unit(cross(uZ, uX));
                    M_ned2sframe = [ uX uY uZ ]';
                    FG_ned = [0 0 env.m_max * env.g]';
                    % There we go: the gravitaional force in the local spline
                    % system:
                    FG_sframe = M_ned2sframe * FG_ned;
                    % Compute centrifugal force in local spline system:
                    r = abs(1/norm(Kappa));
                    
                    FC_sframe = [0;
                        env.m_max * env.v_max^2 / r;
                        0];
                    % Compute maximum lift force:
                    FL_max = env.CL_max * env.S_min * env.v_max^2 * 0.5 * env.rho_min;
                    % Rotate by local bank angle to local spline frame:
                    phi_local = acos(FG_sframe(3)/FL_max);
                    dcm_tframe2sframe = angle2dcm(0, 0, -phi_local, 'ZYX');
                    FL_max_sframe = dcm_tframe2sframe * [0; 0; -FL_max];
                    sumOfForces_sframe = FC_sframe + FG_sframe + FL_max_sframe;
                    sumOfForces_tframe = dcm_tframe2sframe' * sumOfForces_sframe;
                    % Get resulting force in lift direction:
                    cost(iS) = abs(sumOfForces_tframe(3));
                end
                costMax(ispline) = max(cost);
            end
            deltaLiftForce = max(costMax);
        end
    end

% Accepts three consecutive waypoints, builds the smoothing spline segments
% that connects the three and returns them
    function [s1, s2] = buildSegment(p1, p2, p3, maxc2)
        p1_3D = p1;
        p2_3D = p2;
        p3_3D = p3;
        % Compute unit vectors of local cartesian system to be able to build
        % the spline in 2D:
        ux = unit(p2_3D-p1_3D);
        uz = unit(cross(ux, unit(p3_3D-p2_3D)));
        uy = unit(cross(uz, ux));
        % Transformation matrix local2D <-> global 3D:
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
        
        % Computes the maximum feasible curvature on this spline segment
        % based on the aircraft's performance limits:
        function cmax = getMaxCurvature()
            % Compute maximum lift force:
            env = envelope;
            FL_max = env.CL_max * env.S_min * env.v_max^2 * 0.5 * env.rho_min;
            % Compute gravitational force in spline system:
            % Build rotation matrix from ned frame to spline frame from unit
            % vectors of spline frame in NED frame:
            pa = p2_3D + unit(p1_3D-p2_3D);
            pb = p2_3D + unit(p3_3D-p2_3D);
            uX = unit(pb-pa);
            uZ = unit(cross(unit(p3_3D-p2_3D), unit(p1_3D-p2_3D)));
            uY = unit(cross(uZ, uX));
            M_ned2sframe = [ uX uY uZ ]';
            FG_ned = [0 0 env.m_max * env.g]';
            FG_sframe = M_ned2sframe * FG_ned;
            % Compute bank angle in spline frame:
            phi_local = acos(FG_sframe(3)/FL_max);
            % Compute minimal turn radius:
            r = (1/(FL_max * sin(phi_local) + FG_sframe(2))) * env.v_max / env.m_max;
            cmax = 1/r;
        end
        
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
            % arrow(p2_2D, p2_2D+u1);
            % arrow(p2_2D, p2_2D+u2);
            %annotation('arrows',[p2_2D(1) u1(1)],[p2_2D(2) u1(2)]);
            
            % plotbrowser on;
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
