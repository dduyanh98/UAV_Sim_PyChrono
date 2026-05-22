function make_rkhs_video(workspaceFile, outDir)
% Make trajectory-vs-RKHS-dictionary videos from a workspace_log_*.mat.
%
% Usage:
%   make_rkhs_workspace_video
%   make_rkhs_workspace_video('logs/.../workspace_log_....mat')
%   make_rkhs_workspace_video('logs/.../workspace_log_....mat', 'logs/videos')

if nargin < 1
    workspaceFile = '';
end
if nargin < 2 || isempty(outDir)
    outDir = default_video_dir();
end
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

[S, workspaceFile] = load_workspace_log(workspaceFile);
if ~isfield(S.log, 'rkhs')
    error('Workspace has no log.rkhs field. Re-run a RKHS simulation with the new workspace logger.');
end

D = build_data(S.log);
rkhs = S.log.rkhs;

vBody = inertial_to_body_velocity(D.velocity, D.euler);
vBodyRef = inertial_to_body_velocity(D.velocity_ref, D.euler);

[~, baseName] = fileparts(workspaceFile);
make_one_video(D.time, vBody, vBodyRef, rkhs, 0, 'tran', outDir, baseName);
make_one_video(D.time, D.omega, D.omega_ref, rkhs, 1, 'rot', outDir, baseName);

disp("Done. Videos saved in: " + string(outDir));

end

function make_one_video(t, traj3, traj3Ref, rkhs, domainId, tag, outDir, baseName)
samples = rkhs.samples;
isDomain = col(samples.domain_id) == domainId;

timeE = col(samples.time);
timeE = timeE(isDomain);
libGrew = col(samples.library_grew);
libGrew = libGrew(isDomain);
activeCount = col(samples.active_center_count);
activeCount = activeCount(isDomain);
added = xyz(samples.last_added_center);
added = added(isDomain, :);
centerAdded = all(isfinite(added), 2);
activeCenters = active_center_array(samples.active_centers);
activeCenters = activeCenters(isDomain, :, :);

[timeE, order] = sort(timeE);
libGrew = libGrew(order);
activeCount = activeCount(order);
added = added(order, :);
centerAdded = centerAdded(order);
activeCenters = activeCenters(order, :, :);

N = numel(t);
snapshotCenters = nan(N, 8, 3);
snapshotCounts = zeros(N, 1);
addedHistory = nan(N, 3);

j = 0;
lastCenters = nan(8, 3);
lastCount = 0;
for k = 1:N
    while j + 1 <= numel(timeE) && timeE(j + 1) <= t(k)
        j = j + 1;
        lastCenters = squeeze(activeCenters(j, :, :));
        lastCount = activeCount(j);
        if (libGrew(j) == 1 || centerAdded(j)) && all(isfinite(added(j, :)))
            addedHistory(k, :) = added(j, :);
        end
    end
    snapshotCenters(k, :, :) = lastCenters;
    snapshotCounts(k) = lastCount;
end

vidFile = fullfile(outDir, char(string(baseName) + "_" + string(tag) + "_rkhs_dictionary.mp4"));
writer = VideoWriter(vidFile, 'MPEG-4');
writer.FrameRate = 30;
open(writer);

stride = 2;
idxFrames = 1:stride:N;

fig = figure('Color','w', 'Position',[100 100 1100 820]);
set(fig, 'MenuBar','none', 'ToolBar','none');
ax = axes(fig); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');

xlabel(ax, string(tag) + "_x");
ylabel(ax, string(tag) + "_y");
zlabel(ax, string(tag) + "_z");
title(ax, upper(string(tag)) + " trajectory + RKHS dictionary");

plot3(ax, traj3(:,1), traj3(:,2), traj3(:,3), 'Color',[0.85 0.85 0.85], 'LineWidth',0.5);
haveRef = ~isempty(traj3Ref) && size(traj3Ref,2) == 3 && any(all(isfinite(traj3Ref), 2));
if haveRef
    plot3(ax, traj3Ref(:,1), traj3Ref(:,2), traj3Ref(:,3), 'Color',[0.70 0.70 0.70], 'LineWidth',1.0);
end

hTraj = plot3(ax, traj3(1,1), traj3(1,2), traj3(1,3), 'b-', 'LineWidth',2);
hDot = scatter3(ax, traj3(1,1), traj3(1,2), traj3(1,3), 50, 'b', 'filled');
if haveRef
    hTrajRef = plot3(ax, traj3Ref(1,1), traj3Ref(1,2), traj3Ref(1,3), '-', 'LineWidth',2);
    hDotRef = scatter3(ax, traj3Ref(1,1), traj3Ref(1,2), traj3Ref(1,3), 50, 'filled');
else
    hTrajRef = [];
    hDotRef = [];
end

hCenter = scatter3(ax, nan, nan, nan, 90, 'r', 'filled');
hCorners = scatter3(ax, nan, nan, nan, 60, 'r');
hLib = scatter3(ax, nan, nan, nan, 40, 'k', 'filled');

if haveRef
    legend(ax, {'Trajectory full','Reference full','Trajectory so far','Trajectory state', ...
                'Reference so far','Reference state','Active center mean','Active centers','Library growth'}, ...
                'Location','best');
else
    legend(ax, {'Trajectory full','Trajectory so far','Trajectory state','Active center mean','Active centers','Library growth'}, ...
                'Location','best');
end
view(ax, 3);

libPts = nan(numel(idxFrames), 3);
libCount = 0;
for ii = 1:numel(idxFrames)
    k = idxFrames(ii);

    set(hTraj, 'XData',traj3(1:k,1), 'YData',traj3(1:k,2), 'ZData',traj3(1:k,3));
    set(hDot, 'XData',traj3(k,1), 'YData',traj3(k,2), 'ZData',traj3(k,3));

    if haveRef
        set(hTrajRef, 'XData',traj3Ref(1:k,1), 'YData',traj3Ref(1:k,2), 'ZData',traj3Ref(1:k,3));
        set(hDotRef, 'XData',traj3Ref(k,1), 'YData',traj3Ref(k,2), 'ZData',traj3Ref(k,3));
    end

    centers = squeeze(snapshotCenters(k, :, :));
    count = min(max(round(snapshotCounts(k)), 0), 8);
    if count > 0
        validCenters = centers(1:count, :);
        centerMean = mean(validCenters, 1, 'omitnan');
        set(hCenter, 'XData',centerMean(1), 'YData',centerMean(2), 'ZData',centerMean(3));
        set(hCorners, 'XData',validCenters(:,1), 'YData',validCenters(:,2), 'ZData',validCenters(:,3));
    else
        set(hCenter, 'XData',nan, 'YData',nan, 'ZData',nan);
        set(hCorners, 'XData',nan, 'YData',nan, 'ZData',nan);
    end

    if all(isfinite(addedHistory(k, :)))
        libCount = libCount + 1;
        libPts(libCount, :) = addedHistory(k, :);
        set(hLib, 'XData',libPts(1:libCount,1), 'YData',libPts(1:libCount,2), 'ZData',libPts(1:libCount,3));
    end

    title(ax, sprintf('%s RKHS dictionary | t = %.2f s', upper(tag), t(k)));
    drawnow;
    writeVideo(writer, getframe(fig));
end

close(writer);
close(fig);
disp("Saved video: " + string(vidFile));

end

function D = build_data(L)
D.time = col(L.time);
D.velocity = xyz(L.velocity);
D.euler = [col(L.euler_angles.roll), col(L.euler_angles.pitch), col(L.euler_angles.yaw)];
D.omega = xyz(L.angular_velocity);
D.velocity_ref = xyz(L.outer_loop.reference_model.velocity);
D.omega_ref = xyz(L.inner_loop.reference_model.angular_velocity);
end

function vBody = inertial_to_body_velocity(vI, euler)
vBody = zeros(size(vI));
for k = 1:size(vI, 1)
    roll = euler(k,1);
    pitch = euler(k,2);
    yaw = euler(k,3);

    R3 = [cos(yaw), -sin(yaw), 0;
          sin(yaw),  cos(yaw), 0;
          0,         0,        1];
    R2 = [ cos(pitch), 0, sin(pitch);
           0,          1, 0;
          -sin(pitch), 0, cos(pitch)];
    R1 = [1, 0,          0;
          0, cos(roll), -sin(roll);
          0, sin(roll),  cos(roll)];

    vBody(k,:) = ((R3 * R2 * R1)' * vI(k,:).').';
end
end

function A = active_center_array(activeCenters)
C0 = xyz(activeCenters.center_0);
A = nan(size(C0, 1), 8, 3);
A(:, 1, :) = reshape(C0, size(C0, 1), 1, 3);
for i = 0:7
    if i == 0
        continue;
    end
    fieldName = sprintf('center_%d', i);
    C = xyz(activeCenters.(fieldName));
    A(:, i + 1, :) = reshape(C, size(C, 1), 1, 3);
end
end

function X = xyz(S)
X = [col(S.x), col(S.y), col(S.z)];
end

function y = col(x)
y = double(x(:));
end

function workspaceFile = latest_workspace_file()
scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
files = dir(fullfile(projectRoot, 'logs', '**', 'workspace_log_*.mat'));
if isempty(files)
    error('No workspace_log_*.mat files found under %s. Load a workspace first or pass a workspace file path explicitly.', fullfile(projectRoot, 'logs'));
end
[~, idx] = max([files.datenum]);
workspaceFile = fullfile(files(idx).folder, files(idx).name);
end

function [S, workspaceFile] = load_workspace_log(workspaceFile)
if nargin < 1 || isempty(workspaceFile)
    if evalin('base', 'exist(''log'', ''var'')')
        S.log = evalin('base', 'log');
        if evalin('base', 'exist(''sim_cfg'', ''var'')')
            S.sim_cfg = evalin('base', 'sim_cfg');
        end
        if evalin('base', 'exist(''gains'', ''var'')')
            S.gains = evalin('base', 'gains');
        end
        workspaceFile = 'base workspace';
        return;
    end

    workspaceFile = latest_workspace_file();
end

S = load(workspaceFile);
if ~isfield(S, 'log')
    error('Workspace file does not contain variable "log": %s', workspaceFile);
end
end

function outDir = default_video_dir()
scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
outDir = fullfile(projectRoot, 'logs', 'videos');
end
