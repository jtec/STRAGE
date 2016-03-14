function sign = trajectory_getHorizontalCurvatureSign( dp, ddp )

% Figure out sign of curvature in NED frame. 
% Strategy: does the acceleration
% vector point into the same direction as the cross product of the
% velocity vector with [0 0 1]_NED? 
% clockwise (seen from above) = 1,
% anticlockwise = -1;
% FIXME There are singularities for purely vertical velocities.
anticlockwise = fflib_normalize(cross(fflib_normalize(dp), [0 0 1]'));
if norm(anticlockwise - fflib_normalize(ddp)) <= norm(anticlockwise + fflib_normalize(ddp))
    sign = -1;
else
    sign = 1;
end

end