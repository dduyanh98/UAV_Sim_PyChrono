function plot_rkhs(workspaceFile)
% Plot regular flight data and RKHS center updates from a workspace_log_*.mat.
%
% Usage:
%   plot_rkhs_workspace
%   plot_rkhs_workspace('logs/2026/05/20260506/TwoLayerMRACwithRKHS/workspaces/workspace_log_....mat')

if nargin < 1
    workspaceFile = '';
end

[S, workspaceFile] = load_workspace_log(workspaceFile);
L = S.log;
D = build_data(L);
rkhs = get_rkhs(L);

set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultAxesFontSize',20);

fontSize = 20;
fontSizeTitle = 22;
titleText = controller_title(S, workspaceFile);
axisNames = {'x', 'y', 'z'};
axisNamesUpper = {'X', 'Y', 'Z'};

fprintf('Loaded workspace: %s\n', workspaceFile);
fprintf('Samples: %d, time range [%.3f, %.3f] s\n', numel(D.time), D.time(1), D.time(end));

%% Total thrust
set(figure, 'Color','white', 'WindowState','maximized')
plot(D.time, sum(D.thrust, 2), 'b-', 'LineWidth', 2)
xlabel('$t$ [s]', 'interpreter','latex', 'fontsize',fontSize)
ylabel('Total thrust [N]', 'interpreter','latex', 'fontsize',fontSize)
title([titleText ' - Total thrust'], 'interpreter','latex', 'fontsize',fontSizeTitle)
legend('$u_1(t)$', 'Location','best');
axis tight

%% Motor thrusts
set(figure, 'Color','white', 'WindowState','maximized')
subplot(2,1,1)
plot(D.time, D.thrust(:,1), 'b-', 'LineWidth', 2); hold on
plot(D.time, D.thrust(:,2), 'r-.', 'LineWidth', 2)
plot(D.time, D.thrust(:,3), 'g--', 'LineWidth', 2)
plot(D.time, D.thrust(:,4), 'k-', 'LineWidth', 0.8)
legend('$T_1(t)$','$T_2(t)$','$T_3(t)$','$T_4(t)$', 'Location','best');
xlabel('$t$ [s]', 'interpreter','latex', 'fontsize',fontSize)
ylabel('Thrust motor $i$ [N]', 'interpreter','latex', 'fontsize',fontSize)
title([titleText ' - motor thrusts'], 'interpreter','latex', 'fontsize',fontSizeTitle)
axis tight; hold off

subplot(2,1,2)
plot(D.time, D.thrust(:,5), 'b-', 'LineWidth', 2); hold on
plot(D.time, D.thrust(:,6), 'r-.', 'LineWidth', 2)
plot(D.time, D.thrust(:,7), 'g--', 'LineWidth', 2)
plot(D.time, D.thrust(:,8), 'k-', 'LineWidth', 0.8)
legend('$T_5(t)$','$T_6(t)$','$T_7(t)$','$T_8(t)$', 'Location','best');
xlabel('$t$ [s]', 'interpreter','latex', 'fontsize',fontSize)
ylabel('Thrust motor $i$ [N]', 'interpreter','latex', 'fontsize',fontSize)
axis tight; hold off

%% Euler angles
set(figure, 'Color','white', 'WindowState','maximized')
labels = {'Roll [deg]', 'Pitch [deg]', 'Yaw [deg]'};
names = {'$\phi(t)$', '$\theta(t)$', '$\psi(t)$'};
refs = {'$\phi_{\rm ref}(t)$', '$\theta_{\rm ref}(t)$', '$\psi_{\rm ref}(t)$'};
for i = 1:3
    subplot(3,1,i)
    plot(D.time, rad2deg(D.euler(:,i)), 'b-', 'LineWidth', 2); hold on
    plot(D.time, rad2deg(D.euler_ref(:,i)), 'r-.', 'LineWidth', 2)
    legend(names{i}, refs{i}, 'Location','best');
    xlabel('$t$ [s]', 'interpreter','latex', 'fontsize',fontSize)
    ylabel(labels{i}, 'interpreter','latex', 'fontsize',fontSize)
    if i == 1
        title([titleText ' - attitude'], 'interpreter','latex', 'fontsize',fontSizeTitle)
    end
    axis tight; hold off
end

%% Position
set(figure, 'Color','white', 'WindowState','maximized')
for i = 1:3
    subplot(3,1,i)
    sgn = 1;
    if i == 3
        sgn = -1;
    end
    plot(D.time, sgn * D.position(:,i), 'r-', 'LineWidth', 2); hold on
    plot(D.time, sgn * D.position_user(:,i), 'b-.', 'LineWidth', 2)
    plot(D.time, sgn * D.position_ref(:,i), 'k-', 'LineWidth', 1)
    axisName = axisNames{i};
    legend(sprintf('$r_%s(t)$', axisName), sprintf('$r_{{\\rm user},%s}(t)$', axisName), sprintf('$r_{{\\rm ref},%s}(t)$', axisName), 'Location','best');
    xlabel('$t$ [s]', 'interpreter','latex', 'fontsize',fontSize)
    ylabel(sprintf('%s position [m]', axisNamesUpper{i}), 'interpreter','latex', 'fontsize',fontSize)
    if i == 1
        title([titleText ' - position'], 'interpreter','latex', 'fontsize',fontSizeTitle)
    end
    axis tight; hold off
end

%% Velocity
set(figure, 'Color','white', 'WindowState','maximized')
for i = 1:3
    subplot(3,1,i)
    sgn = 1;
    if i == 3
        sgn = -1;
    end
    plot(D.time, sgn * D.velocity(:,i), 'r-', 'LineWidth', 2); hold on
    plot(D.time, sgn * D.velocity_user(:,i), 'b-.', 'LineWidth', 2)
    plot(D.time, sgn * D.velocity_ref(:,i), 'k-', 'LineWidth', 1)
    axisName = axisNames{i};
    legend(sprintf('$v_%s(t)$', axisName), sprintf('$v_{{\\rm user},%s}(t)$', axisName), sprintf('$v_{{\\rm ref},%s}(t)$', axisName), 'Location','best');
    xlabel('$t$ [s]', 'interpreter','latex', 'fontsize',fontSize)
    ylabel(sprintf('$v_%s$ [m/s]', axisName), 'interpreter','latex', 'fontsize',fontSize)
    if i == 1
        title([titleText ' - velocity'], 'interpreter','latex', 'fontsize',fontSizeTitle)
    end
    axis tight; hold off
end

%% Angular velocity
set(figure, 'Color','white', 'WindowState','maximized')
for i = 1:3
    subplot(3,1,i)
    plot(D.time, D.omega(:,i), 'r-', 'LineWidth', 2); hold on
    plot(D.time, D.omega_ref(:,i), 'k-', 'LineWidth', 1)
    axisName = axisNames{i};
    legend(sprintf('$\\omega_%s(t)$', axisName), sprintf('$\\omega_{{\\rm ref},%s}(t)$', axisName), 'Location','best');
    xlabel('$t$ [s]', 'interpreter','latex', 'fontsize',fontSize)
    ylabel(sprintf('$\\omega_%s$ [rad/s]', axisName), 'interpreter','latex', 'fontsize',fontSize)
    if i == 1
        title([titleText ' - angular velocity'], 'interpreter','latex', 'fontsize',fontSizeTitle)
    end
    axis tight; hold off
end

%% Mu
set(figure, 'Color','white', 'WindowState','maximized')
for i = 1:3
    subplot(3,1,i)
    plot(D.time, D.mu(:,i), 'r-', 'LineWidth', 2)
    axisName = axisNames{i};
    legend(sprintf('$\\mu_%s(t)$', axisName), 'Location','best');
    xlabel('$t$ [s]', 'interpreter','latex', 'fontsize',fontSize)
    ylabel(sprintf('$\\mu_%s(t)$ [N]', axisName), 'interpreter','latex', 'fontsize',fontSize)
    if i == 1
        title([titleText ' - translational control'], 'interpreter','latex', 'fontsize',fontSizeTitle)
    end
    axis tight
end

%% Tracking errors
trajError = [D.position - D.position_user, D.velocity - D.velocity_user];
posErrorNorm = vecnorm(trajError(:,1:3), 2, 2);
trajErrorNorm = vecnorm(trajError, 2, 2);
trajErrorL2 = sqrt(cumtrapz(D.time, trajErrorNorm.^2));

set(figure, 'Color','white', 'WindowState','maximized')
for i = 1:3
    subplot(3,2,2*i-1)
    sgn = 1;
    if i == 3
        sgn = -1;
    end
    plot(D.time, sgn * trajError(:,i), 'r-', 'LineWidth', 2)
    xlabel('$t$ [s]', 'interpreter','latex', 'fontsize',fontSize)
    axisName = axisNames{i};
    ylabel(sprintf('$e_{r_%s}$ [m]', axisName), 'interpreter','latex', 'fontsize',fontSize)
    if i == 1
        title('Trajectory tracking error components', 'interpreter','latex', 'fontsize',fontSizeTitle)
    end
    axis tight

    subplot(3,2,2*i)
    plot(D.time, sgn * trajError(:,i+3), 'b-', 'LineWidth', 2)
    xlabel('$t$ [s]', 'interpreter','latex', 'fontsize',fontSize)
    ylabel(sprintf('$e_{v_%s}$ [m/s]', axisName), 'interpreter','latex', 'fontsize',fontSize)
    axis tight
end

set(figure, 'Color','white', 'WindowState','maximized')
plot(D.time, trajErrorNorm, 'r-', 'LineWidth', 2); hold on
plot(D.time, posErrorNorm, 'b-.', 'LineWidth', 2)
legend('$\|[e_r(t)\; e_v(t)]\|$', '$\|e_r(t)\|$', 'Location','best');
xlabel('$t$ [s]', 'interpreter','latex', 'fontsize',fontSize)
ylabel('Error norm', 'interpreter','latex', 'fontsize',fontSize)
title('Tracking error norms', 'interpreter','latex', 'fontsize',fontSizeTitle)
axis tight; hold off

set(figure, 'Color','white', 'WindowState','maximized')
plot(D.time, trajErrorL2, 'k-', 'LineWidth', 2)
legend('$$\sqrt{\int_{t_0}^t\|[e_r(\tau)\;e_v(\tau)]\|^2\,\mathrm{d}\tau}$$', 'Location','best');
xlabel('$t$ [s]', 'interpreter','latex', 'fontsize',fontSize)
ylabel('$\mathcal{L}_2$-norm', 'interpreter','latex', 'fontsize',fontSize)
title('$\mathcal{L}_2$-norm of trajectory tracking error', 'interpreter','latex', 'fontsize',fontSizeTitle)
axis tight

%% 3D trajectory
set(figure, 'Color','white', 'WindowState','maximized')
plot3(D.position_user(:,1), -D.position_user(:,2), -D.position_user(:,3), 'Color',[0.30 0.75 0.93], 'LineWidth', 2); hold on
plot3(D.position(:,1), -D.position(:,2), -D.position(:,3), 'r-.', 'LineWidth', 2)
plot3(D.position_ref(:,1), -D.position_ref(:,2), -D.position_ref(:,3), 'k-', 'LineWidth', 1)
legend('$r_{\rm user}$', '$r$', '$r_{\rm ref}$', 'Location','best');
axis equal; grid on; view(3)
xlabel('X [m]', 'interpreter','latex', 'fontsize',fontSize)
ylabel('Y [m]', 'interpreter','latex', 'fontsize',fontSize)
zlabel('Z [m]', 'interpreter','latex', 'fontsize',fontSize)
title([titleText ' - position 3D'], 'interpreter','latex', 'fontsize',fontSizeTitle)
hold off

%% RKHS centers
if has_rkhs(rkhs)
    plot_rkhs_center_evolution(rkhs, 0, 'Translational RKHS center evolution', {'$c_{v_x}$','$c_{v_y}$','$c_{v_z}$'});
    plot_rkhs_center_evolution(rkhs, 1, 'Rotational RKHS center evolution', {'$c_{\omega_x}$','$c_{\omega_y}$','$c_{\omega_z}$'});
    overlay_rkhs_update_times(rkhs);
else
    warning('This workspace has no log.rkhs field. Regular flight plots were generated only.');
end

end

function D = build_data(L)
D.time = col(L.time);
D.position = xyz(L.position);
D.velocity = xyz(L.velocity);
D.euler = [col(L.euler_angles.roll), col(L.euler_angles.pitch), col(L.euler_angles.yaw)];
D.omega = xyz(L.angular_velocity);
D.position_user = xyz(L.user_defined_position);
D.velocity_user = xyz(L.user_defined_velocity);
D.position_ref = xyz(L.outer_loop.reference_model.position);
D.velocity_ref = xyz(L.outer_loop.reference_model.velocity);
D.euler_ref = [col(L.desired_euler_angles.roll), col(L.desired_euler_angles.pitch), col(L.user_defined_yaw)];
D.omega_ref = xyz(L.inner_loop.reference_model.angular_velocity);
D.mu = xyz(L.mu_translational);
D.thrust = thrust_matrix(L.thrust_motors_N);
end

function X = xyz(S)
X = [col(S.x), col(S.y), col(S.z)];
end

function y = col(x)
y = double(x(:));
end

function T = thrust_matrix(S)
names = {'T1','T2','T3','T4','T5','T6','T7','T8'};
T = zeros(numel(col(S.T1)), 8);
for i = 1:8
    T(:,i) = col(S.(names{i}));
end
end

function rkhs = get_rkhs(L)
if isfield(L, 'rkhs')
    rkhs = L.rkhs;
else
    rkhs = [];
end
end

function tf = has_rkhs(rkhs)
tf = ~isempty(rkhs) && isfield(rkhs, 'samples');
end

function plot_rkhs_center_evolution(rkhs, domainId, plotTitle, axisLabels)
samples = rkhs.samples;
C = xyz(samples.last_added_center);
centerAdded = all(isfinite(C), 2);
idx = col(samples.domain_id) == domainId & (col(samples.library_grew) == 1 | centerAdded);
C = C(idx, :);
C = C(all(isfinite(C), 2), :);

set(figure, 'Color','white', 'WindowState','maximized')
hold on; grid on
if isempty(C)
    title([plotTitle ' - no library growth events']);
else
    plot3(C(:,1), C(:,2), C(:,3), '--o', 'LineWidth', 1.5, 'MarkerSize', 4)
    scatter3(C(1,1), C(1,2), C(1,3), 80, 'g', 'filled')
    scatter3(C(end,1), C(end,2), C(end,3), 80, 'r', 'filled')
    legend('New centers', 'Start', 'End', 'Location','best')
end
xlabel(axisLabels{1}, 'interpreter','latex')
ylabel(axisLabels{2}, 'interpreter','latex')
zlabel(axisLabels{3}, 'interpreter','latex')
title(plotTitle, 'interpreter','latex')
axis equal; view(3)
hold off
end

function overlay_rkhs_update_times(rkhs)
samples = rkhs.samples;
t = col(samples.time);
domain = col(samples.domain_id);
addedCenter = all(isfinite(xyz(samples.last_added_center)), 2);
changed = col(samples.box_changed) == 1 | col(samples.library_grew) == 1 | addedCenter;
tTran = unique(t(domain == 0 & changed));
tRot = unique(t(domain == 1 & changed));

figs = findall(0, 'Type','figure');
for k = 1:numel(figs)
    axAll = findall(figs(k), 'Type','axes');
    for j = 1:numel(axAll)
        ax = axAll(j);
        if ~isempty(ax.ZLabel.String)
            continue;
        end

        ylab = lower(string(ax.YLabel.String));
        ttl = lower(string(ax.Title.String));
        useRot = contains(ylab, 'omega') || contains(ylab, '\omega') || ...
                 contains(ylab, 'roll') || contains(ylab, 'pitch') || contains(ylab, 'yaw') || ...
                 contains(ttl, 'attitude') || contains(ttl, 'angular');
        if useRot
            draw_vlines(ax, tRot);
        else
            draw_vlines(ax, tTran);
        end
    end
end
end

function draw_vlines(ax, tvec)
if isempty(tvec)
    return;
end
xlims = xlim(ax);
tvec = tvec(tvec >= xlims(1) & tvec <= xlims(2));
if isempty(tvec)
    return;
end
ylims = ylim(ax);
hold(ax, 'on')
for i = 1:numel(tvec)
    plot(ax, [tvec(i), tvec(i)], ylims, 'k--', 'LineWidth', 0.8, 'HandleVisibility','off');
end
ylim(ax, ylims)
hold(ax, 'off')
end

function out = controller_title(S, workspaceFile)
if isfield(S, 'sim_cfg') && isfield(S.sim_cfg, 'mission_config') && isfield(S.sim_cfg.mission_config, 'controller_type')
    out = char(string(S.sim_cfg.mission_config.controller_type));
else
    [~, name] = fileparts(workspaceFile);
    out = strrep(name, '_', '\_');
end
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
