% Returns an approximate value of the spline length.
function spline = trajectory_approximateSplineLength( spline)

% Recursively divide spline parameter into two sections, compare distance
% of endpoints to sum of distances; segment length is accepted if the
% relative error is smaller than the following threshold:
e = 1e-3;
spline.discrete.ts = [];
spline.discrete.arclength = 0;
% spline.l = checkInterval(0,1);

% For rather straight splines, the recursive strategy might create only a
% small number of points (e.g. only 3 for a straight line), which is
% sufficient to estimate the total arc length, but not the arc length
% distribution as function of the spline parameter t [0,1].
spline.discrete.ts = 0:0.01:1;
for i=1:length(spline.discrete.ts)-1
    ds = norm(trajectory_evaluateBezier(spline, spline.discrete.ts(i)) - trajectory_evaluateBezier(spline, spline.discrete.ts(i+1)));
    spline.discrete.arclength = [spline.discrete.arclength spline.discrete.arclength(end) + ds];
end

    function l = checkInterval(a, b)
        middle = (a + b) / 2;
        aP = trajectory_evaluateBezier(spline, a);
        bP = trajectory_evaluateBezier(spline, b);
        middleP = trajectory_evaluateBezier(spline, middle);
        l = norm(middleP-aP) + norm(bP-middleP);
        error = norm(1 -  norm(aP-bP)/l);
        if error > e
            l = checkInterval(a, middle) + checkInterval(middle, b);
        else
            spline.discrete.ts = [spline.discrete.ts a middle b];
        end
    end
end
