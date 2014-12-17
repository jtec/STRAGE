% Evaluates a given cubic Bezier spline at the given path parameter t [0:1].
function [p, dpdt, dpddt] = trajectory_evaluateBezier(bez, t)
n = 3;
% Evaluate Bernstein polynoms:
p = bernstein(n, 0, t) .* bez.p1 ...
    + bernstein(n, 1, t) .* bez.p2 ...
    + bernstein(n, 2, t) .* bez.p3 ...
    + bernstein(n, 3, t) .* bez.p4;

% Compute first derivative:
dpdt = zeros(3,1);
n = 3;
% Evaluate Bernstein polynoms:
dpdt = bernstein(n-1, 0, t) .* (bez.p2 - bez.p1)...
    + bernstein(n-1, 1, t) .* (bez.p3 - bez.p2) ...
    + bernstein(n-1, 2, t) .* (bez.p4 - bez.p3);
dpdt = n * dpdt;

% Compute second derivative:
dpddt = zeros(3,1);
n = 3;
% Evaluate Bernstein polynoms:
dpddt = bernstein(n-2, 0, t) .* (bez.p3 - 2*bez.p2 + bez.p1)...
    + bernstein(n-2, 1, t) .* (bez.p4 - 2*bez.p3 + bez.p2);
dpddt = n * (n-1) * dpddt;

    function brn = bernstein(order, i, t)
        brn = nchoosek(order,i) * t^i * (1-t)^(order-i);
    end
end