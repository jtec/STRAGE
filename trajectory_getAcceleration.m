% Computes the acceleration acting on a point moving on the given spline
% trajectory at the given total arc length and the given velocity,
function acc = trajectory_getAcceleration( traj, s, v)
[p, uTangent, Kappa] = trajectory_get( traj, s);
% The acceleration is equivalent to the centripetal acceleration on the
% osculating circle:
% unit vector:
if norm(Kappa) > eps
    u = unit(Kappa);
    r = 1/norm(Kappa);
    mag = v^2 / r;
    acc = mag * u;
else
    % If Kappa is [0 0 0], there is now way to compute a unit vector - but, 
    % since acceleration is zero, we just return:
    acc = [0; 0; 0];
end
end