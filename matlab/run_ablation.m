function run_ablation(seeds)
%RUN_ABLATION  What each addition to the base paper's PID actually buys.
%
%   Two questions are being asked and they need different tests.
%
%   Whether the drone can land on a moving pad AT ALL is answered on the
%   moving addresses. How ACCURATELY it lands is not, because a
%   configuration that never touches down reports no error at all. The
%   same ladder is therefore also flown against stationary addresses,
%   where every configuration lands and the numbers are comparable.

if nargin < 1 || isempty(seeds), seeds = 0:1; end
C = droneconfig();

base = struct('feedforward',false,'gainSchedule',false, ...
              'antiWindup',false,'tracker','abg');

cfgs = { 'paper PID only',              base;
         '+ integral anti-windup',      setf(base,'antiWindup',true);
         '+ pad-velocity feed-forward', setf(setf(base,'antiWindup',true),'feedforward',true);
         '+ altitude gain scheduling',  setf(setf(setf(base,'antiWindup',true),'feedforward',true),'gainSchedule',true);
         '+ coordinated-turn tracker',  setf(setf(setf(setf(base,'antiWindup',true),'feedforward',true),'gainSchedule',true),'tracker','ct') };

moving = [C.PADS(~strcmp({C.PADS.motion},'static')).id];
static = [C.PADS(strcmp({C.PADS.motion},'static')).id];
static = static(1:3);

fprintf('\n  MOVING addresses  (can it land at all?)\n');
for i = 1:size(cfgs,1)
    [landed, runs, me, mt] = sweep(moving, seeds, cfgs{i,2});
    report(cfgs{i,1}, landed, runs, me, mt);
end

fprintf('\n  STATIONARY addresses  (how accurately?)\n');
for i = 1:size(cfgs,1)
    [landed, runs, me, mt] = sweep(static, seeds, cfgs{i,2});
    report(cfgs{i,1}, landed, runs, me, mt);
end
fprintf('\n');
end

function [landed, runs, meanErr, meanT] = sweep(ids, seeds, opts)
errs = []; times = []; landed = 0; runs = 0;
for id = ids
    for sd = seeds
        r = simulate_landing(id, sd, opts);
        runs = runs + 1;
        if strcmp(r.outcome,'LANDED')
            landed = landed + 1;
            errs(end+1) = r.error_m;  %#ok<AGROW>
            times(end+1) = r.time_s;  %#ok<AGROW>
        end
    end
end
if isempty(errs), meanErr = NaN; meanT = NaN;
else, meanErr = mean(errs); meanT = mean(times); end
end

function report(name, landed, runs, me, mt)
if isnan(me)
    fprintf('     %-30s %d/%2d landed       -          -\n', name, landed, runs);
else
    fprintf('     %-30s %d/%2d landed  %6.2f cm  %5.1f s\n', ...
            name, landed, runs, 100*me, mt);
end
end

function s = setf(s, f, v)
s.(f) = v;
end
