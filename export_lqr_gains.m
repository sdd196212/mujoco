% Export the MATLAB LQR gain table for the MuJoCo Python controller.
%
% Usage:
%   1) Put leg, k11-k16 and k21-k26 in the MATLAB workspace, then run:
%          outputDir = 'C:\\Users\\ggg\\Documents\\Codex\\workspace\\projects\\wheel_leg_mujoco\\wheel_leg_mujoco';
%          export_lqr_gains
%   2) Or set sourceFile to a MAT file before running:
%          sourceFile = 'D:\\path\\to\\calculated_gains.mat';
%          outputDir = 'C:\\path\\to\\wheel_leg_mujoco';
%          export_lqr_gains
%
% The generated files are written beside this script by default:
%   lqr_gains.mat
%   lqr_gains.csv

variableNames = [{'leg'}, ...
    arrayfun(@(n) sprintf('k1%d', n), 1:6, 'UniformOutput', false), ...
    arrayfun(@(n) sprintf('k2%d', n), 1:6, 'UniformOutput', false)];

if ~exist('sourceFile', 'var') || isempty(sourceFile)
    sourceFile = '';
end

% Prefer freshly calculated variables in the base workspace. This prevents
% a stale sourceFile variable from silently reloading old gains.
workspaceReady = true;
for i = 1:numel(variableNames)
    if ~evalin('base', sprintf('exist(''%s'', ''var'')', variableNames{i}))
        workspaceReady = false;
        break;
    end
end

if workspaceReady
    loaded = struct();
    for i = 1:numel(variableNames)
        name = variableNames{i};
        loaded.(name) = evalin('base', name);
    end
elseif ~isempty(sourceFile)
    loaded = load(sourceFile);
else
    error('Workspace is missing leg/k11-k16/k21-k26 and sourceFile is empty.');
end

gainNames = [{'k11'}, {'k12'}, {'k13'}, {'k14'}, {'k15'}, {'k16'}, ...
             {'k21'}, {'k22'}, {'k23'}, {'k24'}, {'k25'}, {'k26'}];

if ~isfield(loaded, 'leg')
    error('Input does not contain variable: leg');
end

leg = double(loaded.leg(:));
if numel(leg) < 2 || any(~isfinite(leg))
    error('leg must contain at least two finite samples.');
end

tableData = zeros(numel(leg), 13);
tableData(:, 1) = leg;
for i = 1:numel(gainNames)
    name = gainNames{i};
    if ~isfield(loaded, name)
        error('Input does not contain variable: %s', name);
    end
    values = double(loaded.(name)(:));
    if numel(values) ~= numel(leg)
        error('%s has %d samples; leg has %d samples.', ...
            name, numel(values), numel(leg));
    end
    if any(~isfinite(values))
        error('%s contains NaN or Inf.', name);
    end
    tableData(:, i + 1) = values;
    loaded.(name) = values;
end

if numel(leg) ~= 31
    warning('Expected 31 leg samples, but found %d.', numel(leg));
end

if ~exist('outputDir', 'var') || isempty(outputDir)
    outputDir = fileparts(mfilename('fullpath'));
end
if ~isfolder(outputDir)
    error('Output directory does not exist: %s', outputDir);
end
matPath = fullfile(outputDir, 'lqr_gains.mat');
csvPath = fullfile(outputDir, 'lqr_gains.csv');

% Keep the individual variable names because lqr_controller.py loads them.
loaded.leg = leg;
save(matPath, '-struct', 'loaded', '-v7');

% No header: Python's CSV fallback expects exactly 13 numeric columns.
dlmwrite(csvPath, tableData, 'delimiter', ',', 'precision', '%.17g');

fprintf('Exported %d LQR samples.\nMAT: %s\nCSV: %s\n', ...
    numel(leg), matPath, csvPath);
