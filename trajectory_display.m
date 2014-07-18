function traj = trajectory_display( traj, options)
%TRAJECTORY_SMOOTH Plots a given smoothed trajectory in 3D.

% Plot colors:
col = distinguishable_colors(3*length(traj.waypoints));

if isfield(traj, 'splines')
    % Plot splines:
    hold off;
    % Iterate over splines, plot series of points for each one:
    splinepoints = [];
    t = 0:0.01:1;
    for i=1:length(traj.splines)
        spline = traj.splines{i};
        
        for j=1:length(t)
            splinepoints = [splinepoints; trajectory_evaluateBezier(spline, t(j))'];
        end
        plot3(splinepoints(:, 1), splinepoints(:, 2), splinepoints(:, 3),...
            'Displayname', ['spline ' num2str(i)], 'color', col(i, :));
        splinepoints = [];
        
        hold on;
        if optionisset(options, 'controlpointsvisible')
            controlpoints = [spline.p1'; spline.p2'; spline.p3'; spline.p4'];
            plot3(controlpoints(:, 1), controlpoints(:, 2), controlpoints(:, 3),...
                'Displayname', ['control points spline ' num2str(i)], 'color', col(i, :), 'Marker', 'o');
        end
    end
end

% Plot waypoints:
% Copy waypoints to array for plotting:
if optionisset(options, 'waypointsvisible')
    wpoints = [];
    for i=1:length(traj.waypoints)
        wpoints(end+1, :) = traj.waypoints{i};
    end
    plot3(wpoints(:, 1), wpoints(:, 2), wpoints(:, 3),...
        'Displayname', 'waypoints', 'Marker', 'o');
end

% Plot discrete points:
if optionisset(options, 'discretepointsvisible') && isfield(traj, 'discrete')
    plot3(traj.discrete.points(:, 1), traj.discrete.points(:, 2), traj.discrete.points(:, 3),...
        'Displayname', 'discrete points', 'Marker', 'x');
end

xlabel 'x'
ylabel 'y'
zlabel 'z'

plotbrowser on;

    function result = optionisset(options, field)
        result = [];
        if isfield(options, field)
            result = options.(field);
        end
    end

end
