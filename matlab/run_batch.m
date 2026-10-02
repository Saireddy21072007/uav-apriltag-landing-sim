function T = run_batch(seeds, opts)
%RUN_BATCH  Fly every address with several noise seeds and report.
%
%   T = RUN_BATCH()            four seeds, the default controller
%   T = RUN_BATCH(0:5)         six seeds
%   T = RUN_BATCH(0:3, opts)   with a specific controller configuration
%
%   Prints a per-address table and the headline figures, the same way
%   ../drone_sim/evaluate.py does for the Python build.

if nargin < 1 || isempty(seeds), seeds = 0:3; end
if nargin < 2, opts = struct(); end

C = droneconfig();
ids = [C.PADS.id];
rows = [];

fprintf('\n  Flying %d addresses x %d seeds = %d missions\n\n', ...
        numel(ids), numel(seeds), numel(ids)*numel(seeds));

for i = 1:numel(ids)
    errs = []; times = []; landed = 0;
    for sd = seeds
        r = simulate_landing(ids(i), sd, opts);
        if strcmp(r.outcome,'LANDED')
            landed = landed + 1;
            errs(end+1)  = r.error_m;   %#ok<AGROW>
            times(end+1) = r.time_s;    %#ok<AGROW>
        end
    end
    p = C.PADS(i);
    rows(end+1).id = p.id;              %#ok<AGROW>
    rows(end).name    = p.name;
    rows(end).moving  = ~strcmp(p.motion,'static');
    rows(end).landed  = landed;
    rows(end).runs    = numel(seeds);
    rows(end).meanErr = mean(errs);
    rows(end).meanT   = mean(times);
    fprintf('   %2d  %-26s %-7s %d/%d  %5.2f cm  %5.1f s\n', ...
        p.id, p.name, kindOf(p), landed, numel(seeds), 100*mean(errs), mean(times));
end

T = struct2table(rows);
allErr = [];
for i = 1:numel(rows), allErr(end+1) = rows(i).meanErr; end %#ok<AGROW>
tot = sum([rows.landed]); runs = sum([rows.runs]);
mv  = [rows.moving];

fprintf('\n  %d/%d landed (%.1f %%)\n', tot, runs, 100*tot/runs);
fprintf('  mean touchdown error   %.2f cm\n', 100*mean(allErr));
fprintf('  static addresses       %.2f cm\n', 100*mean(allErr(~mv)));
fprintf('  moving addresses       %.2f cm\n', 100*mean(allErr(mv)));
fprintf('  mean mission time      %.1f s\n\n', mean([rows.meanT]));
end

function k = kindOf(p)
if strcmp(p.motion,'static')
    if p.height > 0, k = 'roof'; else, k = 'ground'; end
else
    k = 'vehicle';
end
end
