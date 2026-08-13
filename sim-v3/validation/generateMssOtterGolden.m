function generateMssOtterGolden(mssRoot, outputDir)
% Generate frozen planar traces with the official MSS otter.m model.
% Usage:
%   octave --quiet --eval "generateMssOtterGolden('/path/to/MSS','mss-golden')"

if nargin ~= 2
    error('Expected MSS checkout root and output directory.');
end

expectedCommit = 'c660120aa7ea16d0022064bd759d12a934ec4f76';
modelsDir = fullfile(mssRoot, 'CRAFT', 'USV', 'models');
if exist(fullfile(modelsDir, 'otter.m'), 'file') ~= 2
    error('Official MSS otter.m was not found below %s.', mssRoot);
end

[status, commit] = system(sprintf('git -C "%s" rev-parse HEAD', mssRoot));
if status ~= 0 || ~strcmp(strtrim(commit), expectedCommit)
    error('MSS checkout must be pinned to commit %s (found %s).', ...
        expectedCommit, strtrim(commit));
end

addpath(genpath(mssRoot));
if exist(outputDir, 'dir') ~= 7
    mkdir(outputDir);
end

names = {'constant-thrust', 'coast-down', 'turning-circle', ...
    'zig-zag', 'current-drift'};
for i = 1:length(names)
    runManeuver(names{i}, outputDir, expectedCommit);
end
end

function runManeuver(name, outputDir, commit)
dt = 0.05;
duration = 60.0;
steps = round(duration / dt);
x = zeros(12, 1);
mp = 0;
rp = zeros(3, 1);

filePath = fullfile(outputDir, [name '.csv']);
fid = fopen(filePath, 'w');
if fid < 0
    error('Unable to create %s.', filePath);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# source=https://github.com/cybergalactic/MSS\n');
fprintf(fid, '# commit=%s\n', commit);
fprintf(fid, '# model=CRAFT/USV/models/otter.m\n');
fprintf(fid, '# payload_mass_kg=0\n');
fprintf(fid, 't,N,E,yaw,u,v,r\n');

for k = 1:steps
    t = (k - 1) * dt;
    [surge, yaw, currentSpeed, currentDirection] = commandAt(name, t);
    n = propellerSpeeds(surge, yaw);
    x = rk4(@otter, dt, x, n, mp, rp, currentSpeed, currentDirection);
    fprintf(fid, '%.10g,%.16g,%.16g,%.16g,%.16g,%.16g,%.16g\n', ...
        t + dt, x(7), x(8), x(12), x(1), x(2), x(6));
end
end

function [surge, yaw, currentSpeed, currentDirection] = commandAt(name, t)
surge = 0;
yaw = 0;
currentSpeed = 0;
currentDirection = 0;

switch name
    case 'constant-thrust'
        surge = 60;
    case 'coast-down'
        if t < 2
            surge = 60;
        end
    case 'turning-circle'
        surge = 65;
        yaw = 18.9;
    case 'zig-zag'
        surge = 60;
        yaw = 15.12 * (1 - 2 * mod(floor(t / 2), 2));
    case 'current-drift'
        currentSpeed = 0.3;
        currentDirection = 0;
    otherwise
        error('Unknown maneuver %s.', name);
end
end

function n = propellerSpeeds(surge, yaw)
yPont = 0.395;
kPos = 0.02216 / 2;
kNeg = 0.01289 / 2;
thrustLeft = 0.5 * (surge + yaw / yPont);
thrustRight = 0.5 * (surge - yaw / yPont);
n = [speedForThrust(thrustLeft, kPos, kNeg); ...
     speedForThrust(thrustRight, kPos, kNeg)];
end

function value = speedForThrust(thrust, kPos, kNeg)
if thrust >= 0
    value = sqrt(thrust / kPos);
else
    value = -sqrt(abs(thrust) / kNeg);
end
end
