function traj = trajectory_discretize( traj, ds)
%TRAJECTROY_DISCRETIZE Computes equidistant points on a given
%continuous trajectory.
points = [];
for i=1:length(traj.splines)
    %traj.splines{i} = trajectory_approximateSplineLength(traj.splines{i});
    traj.splines{i} = trajectory_parameterizeWithArcLength(traj.splines{i});
end

% Get equidistant points on the trajectory:
sAtEndOfLastSpline = 0;
s = 0;
iSpline = 1;
while true
    % Get spline parameter
    t = fnval(traj.splines{iSpline}.TofS, s-sAtEndOfLastSpline);
    % Check if we have exceeded the end of the current spline:
    % TODO This one-time-check is bound to fail once there are very short
    % splines.
    if t>1
        % Switch to the next spline
        iSpline = iSpline + 1;
        % Check if we are out of splines
        if iSpline <= length(traj.splines)
            %  If not so get spline parameter for new spline
            sAtEndOfLastSpline = sAtEndOfLastSpline + traj.splines{iSpline-1}.discrete.arclength(end);
            t = fnval(traj.splines{iSpline}.TofS, s-sAtEndOfLastSpline);
        else
            % If so, break while loop:
            break;
        end
    end
    % Get spline point 
    points = [points; trajectory_evaluateBezier(traj.splines{iSpline}, t)'];
    % Compute next s
    s = s + ds;
end
traj.discrete.points = points;
traj.discrete.ds = ds;

ds = points(2:end, :) - points(1:end-1, :);
norms = sqrt(sum(ds.^2,2));
end
