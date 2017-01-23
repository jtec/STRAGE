function dtrejo = trajectory_generateTimeseries(traj, dt, v)

output_verbose = strcmpi(traj.news, 'verbose');
output_minimal = strcmpi(traj.news, 'minimal');

vair = v;
ds = vair*dt;
n = ceil(traj.sTotal/ds);
s = linspace(0, traj.sTotal-ds, n);
t = linspace(0, (traj.sTotal-ds)/vair, n);
p = zeros(n, 3);
v = zeros(n, 3);
c = zeros(n, 3);

for k=1:length( s)
    [ps, uTangent, Kappa] = trajectory_get( traj, s(k));
    p(k, :) = ps';
    v(k, :) = uTangent'*vair;
    c(k, :) = Kappa';
end

dtrejo = traj;
dtrejo.ts_p = timeseries([p], t);
dtrejo.ts_v = timeseries([v], t);
dtrejo.ts_c = timeseries([c], t);
end