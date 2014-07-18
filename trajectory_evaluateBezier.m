% Evaluates a given cubic Bezier spline at the given path parameter t [0:1].
    function b = trajectory_evaluateBezier(bez, t)
        b = zeros(3,1);
        n = 3;
        % Evaluate Bernstein polynoms:
        bernstein = [];
        for ivan=0:n
            bernstein(end+1) = nchoosek(n,ivan) * t^ivan * (1-t)^(n-ivan);
        end
        b = bernstein(1) .* bez.p1 ...
            + bernstein(2) .* bez.p2 ...
            + bernstein(3) .* bez.p3 ...
            + bernstein(4) .* bez.p4;
    end