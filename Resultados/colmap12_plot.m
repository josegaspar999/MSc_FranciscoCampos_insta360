function colmap12_plot
% Colmap - Colmap

% -------------------------------------------------------------------------
[frame_id_1, COLMAP_1] = colmap12_load_traj('COLMAP fisheye/text/frames.txt');
[frame_id_2, COLMAP_2] = colmap12_load_traj('COLMAP pinhole/text/frames.txt');

% myfig(1); %figure(101); clf; hold on;
% plot3(COLMAP_1(:,1), COLMAP_1(:,2), COLMAP_1(:,3), 'b-', 'LineWidth', 2);
% plot3(COLMAP_2(:,1), COLMAP_2(:,2), COLMAP_2(:,3), 'r-', 'LineWidth', 2);
% legend('COLMAP Fisheye', 'COLMAP Pinhole');
% xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
% title('COLMAP Trajectories (unaligned)');
% grid on; axis equal; view(3);
% rotate3d on;

% -------------------------------------------------------------------------
[d, COLMAP_2_aligned, transform_2_to_1] = procrustes(COLMAP_1, COLMAP_2);

myfig(2); %figure(102); clf; hold on;
plot3(COLMAP_1(:,1), COLMAP_1(:,2), COLMAP_1(:,3), 'k-', 'LineWidth', 2);
plot3(COLMAP_2_aligned(:,1), COLMAP_2_aligned(:,2), COLMAP_2_aligned(:,3), 'r-', 'LineWidth', 2);
grid on; axis equal; view(3); rotate3d on;
plot3(COLMAP_1(1,1), COLMAP_1(1,2), COLMAP_1(1,3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot3(COLMAP_1(end,1), COLMAP_1(end,2), COLMAP_1(end,3), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot3(COLMAP_2_aligned(1,1), COLMAP_2_aligned(1,2), COLMAP_2_aligned(1,3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot3(COLMAP_2_aligned(end,1), COLMAP_2_aligned(end,2), COLMAP_2_aligned(end,3), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
xlabel(sprintf('#1 pts = %d, #2 pts = %d', length(COLMAP_1), length(COLMAP_2)))
legend('COLMAP 1 Reference', 'COLMAP 2 Aligned', 'Start','End');


% -------------------------------------------------------------------------
err = vecnorm(COLMAP_1 - COLMAP_2_aligned, 2, 2);

rmse = sqrt(mean(err.^2));
mean_err = mean(err);
max_err = max(err);

fprintf('RMSE: %.4f m\n', rmse);
fprintf('Mean error: %.4f m\n', mean_err);
fprintf('Max error: %.4f m\n', max_err);

% traj_fisheye = load('MonoSLAM fisheye/final/trajectory.mat').trajectory;
% traj_pinhole = load('MonoSLAM pinhole/final/trajectory.mat').trajectory;
[traj_fisheye, n1] = colmap12_load_traj('MonoSLAM fisheye/final/trajectory.mat');
[traj_pinhole, n2] = colmap12_load_traj('MonoSLAM pinhole/final/trajectory.mat');

% Plot
myfig(3); %figure(103); clf; hold on;

% Trajectories
h1 = plot3(traj_fisheye(1,1:n1), traj_fisheye(2,1:n1), traj_fisheye(3,1:n1), 'b-', 'LineWidth', 2);
h2 = plot3(traj_pinhole(1,1:n2), traj_pinhole(2,1:n2), traj_pinhole(3,1:n2), 'r-', 'LineWidth', 2);

% Shared start markers
h3 = plot3(traj_fisheye(1,1), traj_fisheye(2,1), traj_fisheye(3,1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot3(traj_pinhole(1,1), traj_pinhole(2,1), traj_pinhole(3,1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');

% Shared end markers
h4 = plot3(traj_fisheye(1,n1), traj_fisheye(2,n1), traj_fisheye(3,n1), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot3(traj_pinhole(1,n2), traj_pinhole(2,n2), traj_pinhole(3,n2), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('MonoSLAM Trajectories');
legend([h1, h2, h3, h4], ...
    sprintf('Fisheye %d pts', n1), ...
    sprintf('Pinhole %d pts', n2), 'Start', 'End');

grid on; axis equal; view(3); rotate3d on;


% -------------------------------------------------------------------------
xx= {COLMAP_1', COLMAP_2_aligned', ...
    traj_fisheye(1:3,1:10:n1-1), traj_pinhole(1:3,1:10:n2-1)};
% xx= {COLMAP_1', COLMAP_2', ...
%     traj_fisheye(1:3,1:10:n1-1), traj_pinhole(1:3,1:10:n2-1)};
global ca1 ca2 ca2p; ca1=COLMAP_1'; ca2=COLMAP_2'; ca2p=COLMAP_2_aligned';
leg= {'CFisheye','CPPinhole','MFisheye','MPinhole'};
data_plot( 'procrustes', xx, leg )


% % -------------------------------------------------------------------------
% filename = 'OLD/COLMAP fisheye/bestmodel/frames.txt';
% [frame_id, COLMAP] = colmap12_load_traj(filename);
% 
% myfig(4); %figure(104); clf; hold on;
% plot3(COLMAP(:,1), COLMAP(:,2), COLMAP(:,3), 'k-', 'LineWidth', 2);
% % Start point
% plot3(COLMAP(1,1), COLMAP(1,2), COLMAP(1,3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
% % End point
% plot3(COLMAP(end,1), COLMAP(end,2), COLMAP(end,3), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
% xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
% title('COLMAP Trajectory');
% legend('Trajectory', 'Start', 'End');
% grid on; axis equal; view(3); rotate3d on;
% 
% 
% % -------------------------------------------------------------------------
% % Convert COLMAP frame IDs to trajectory indices
% traj_idx = frame_id * 30;
% 
% % Use same COLMAP entries
% colmap_matched = COLMAP(1:end-1, :);
% 
% % Extract corresponding MonoSLAM trajectory points
% traj_fisheye_matched = traj_fisheye(:, traj_idx(1:end-1))';
% traj_pinhole_matched = traj_pinhole(:, traj_idx(1:end-1))';
% 
% % Procrustes: align MonoSLAM trajectories to COLMAP
% [d_fisheye, traj_fisheye_aligned, transform_fisheye] = procrustes(COLMAP(1:end-1,:), traj_fisheye_matched(:,1:3));
% [d_pinhole, traj_pinhole_aligned, transform_pinhole] = procrustes(COLMAP(1:end-1,:), traj_pinhole_matched(:,1:3));
% 
% myfig(5); %figure(105); clf; hold on;
% % COLMAP reference
% h1 = plot3(colmap_matched(:,1), colmap_matched(:,2), colmap_matched(:,3), 'k-', 'LineWidth', 2);
% % Fisheye aligned
% h2 = plot3(traj_fisheye_aligned(:,1), traj_fisheye_aligned(:,2), traj_fisheye_aligned(:,3), 'b-', 'LineWidth', 2);
% % Pinhole aligned
% h3 = plot3(traj_pinhole_aligned(:,1), traj_pinhole_aligned(:,2), traj_pinhole_aligned(:,3), 'r-', 'LineWidth', 2);
% % Shared start marker
% h4 = plot3(colmap_matched(1,1), colmap_matched(1,2), colmap_matched(1,3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
% % Shared end marker
% h5 = plot3(colmap_matched(end,1), colmap_matched(end,2), colmap_matched(end,3), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
% 
% xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
% title('Aligned Trajectories');
% legend([h1,h2,h3,h4,h5], 'COLMAP', 'Fisheye', 'Pinhole', 'Start', 'End');
% grid on; axis equal; view(3); rotate3d on;
% 
% 
% % -------------------------------------------------------------------------
% % Errors per matched frame
% err_fisheye = vecnorm(colmap_matched - traj_fisheye_aligned, 2, 2);
% err_pinhole = vecnorm(colmap_matched - traj_pinhole_aligned, 2, 2);
% 
% % Summary metrics
% rmse_fisheye = sqrt(mean(err_fisheye.^2));
% rmse_pinhole = sqrt(mean(err_pinhole.^2));
% 
% mean_fisheye = mean(err_fisheye);
% mean_pinhole = mean(err_pinhole);
% 
% median_fisheye = median(err_fisheye);
% median_pinhole = median(err_pinhole);
% 
% max_fisheye = max(err_fisheye);
% max_pinhole = max(err_pinhole);
% 
% fprintf('Fisheye RMSE: %.4f m\n', rmse_fisheye);
% fprintf('Pinhole RMSE: %.4f m\n', rmse_pinhole);
% 
% fprintf('Fisheye mean error: %.4f m\n', mean_fisheye);
% fprintf('Pinhole mean error: %.4f m\n', mean_pinhole);
% 
% fprintf('Fisheye median error: %.4f m\n', median_fisheye);
% fprintf('Pinhole median error: %.4f m\n', median_pinhole);
% 
% fprintf('Fisheye max error: %.4f m\n', max_fisheye);
% fprintf('Pinhole max error: %.4f m\n', max_pinhole);
% 
% myfig(6); %figure(106); clf; hold on;
% 
% plot(frame_id(1:end-1), err_fisheye, 'b-', 'LineWidth', 2);
% plot(frame_id(1:end-1), err_pinhole, 'r-', 'LineWidth', 2);
% xlabel('Frame ID');
% ylabel('Position Error (m)');
% title('Trajectory Alignment Error');
% legend('Fisheye', 'Pinhole'); grid on;

my_figs( 'pos_load' )


% -------------------------------------------------------------------------
function myfig(figNum)
figure(100+figNum); clf; hold on


function N = vecnorm(A, p, dim)
% VECNORM Vector-wise norm for older MATLAB versions (pre-R2017b).
%   N = VECNORM(A) returns the 2-norm (Euclidean norm) along the first
%   non-singleton dimension of A.
%   N = VECNORM(A, P) calculates the generalized p-norm.
%   N = VECNORM(A, P, DIM) operates along the dimension DIM.

% 1. Set default p-norm to 2 if not provided
if nargin < 2 || isempty(p)
    p = 2;
end

% 2. Determine target dimension if not provided
if nargin < 3 || isempty(dim)
    % Find the first dimension whose size is not equal to 1
    dim = find(size(A) ~= 1, 1);
    if isempty(dim)
        dim = 1; % Fallback for scalar inputs
    end
end

% 3. Calculate the norm along the specified dimension
if isinf(p)
    if p > 0
        N = max(abs(A), [], dim);
    else
        N = min(abs(A), [], dim);
    end
elseif p == 1
    N = sum(abs(A), dim);
elseif p == 2
    % Optimized path for standard Euclidean norm
    N = sqrt(sum(abs(A).^2, dim));
else
    % Generalized p-norm
    N = sum(abs(A).^p, dim).^(1/p);
end
