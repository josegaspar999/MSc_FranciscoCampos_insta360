%-----------------------------------------------------------------------
% 1-point RANSAC EKF SLAM from a monocular sequence
%-----------------------------------------------------------------------

% Copyright (C) 2010 Javier Civera and J. M. M. Montiel
% Universidad de Zaragoza, Zaragoza, Spain.

% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation. Read http://www.gnu.org/copyleft/gpl.html for details

% If you use this code for academic work, please reference:
%   Javier Civera, Oscar G. Grasa, Andrew J. Davison, J. M. M. Montiel,
%   1-Point RANSAC for EKF Filtering: Application to Real-Time Structure from Motion and Visual Odometry,
%   to appear in Journal of Field Robotics, October 2010.

%-----------------------------------------------------------------------
% Authors:  Javier Civera -- jcivera@unizar.es 
%           J. M. M. Montiel -- josemari@unizar.es

% Robotics, Perception and Real Time Group
% Arag�n Institute of Engineering Research (I3A)
% Universidad de Zaragoza, 50018, Zaragoza, Spain
% Date   :  May 2010
%-----------------------------------------------------------------------

clear variables; close all; clc;
rand('state',0); % rand('state',sum(100*clock));

%-----------------------------------------------------------------------
% Sequence, camera and filter tuning parameters, variable initialization
%-----------------------------------------------------------------------

% Camera calibration
cam = initialize_cam;

% Set plot windows
set_plots;

% Sequence path and initial image
sequencePath = '../sequences/images/frame';
initIm = 1;
lastIm = 14872;

% Load detection mask (white = valid region, black = excluded)
mask = imread('../sequences/mask_final.png');
if size(mask, 3) == 3
    mask = rgb2gray(mask);
end
mask = ~(mask > 0);  % mask_final is inverted: black=valid, so negate
mask_rsz = imresize(mask, [cam.nRows, cam.nCols], 'nearest');

% Output path
outputFolder = 'output';
v = VideoWriter(fullfile(outputFolder, 'results_final.mp4'), 'MPEG-4');
v.FrameRate = 30;
open(v);

% Initialize state vector and covariance
[x_k_k, p_k_k] = initialize_x_and_p;

% Initialize EKF filter
sigma_a = 0.009; % standar deviation for linear acceleration noise
sigma_alpha = 0.009; % standar deviation for angular acceleration noise
sigma_image_noise = 3.0; % standar deviation for measurement noise
filter = ekf_filter( x_k_k, p_k_k, sigma_a, sigma_alpha, sigma_image_noise, 'constant_velocity' );

% variables initialization
features_info = [];
trajectory = zeros( 7, lastIm - initIm );
% other
min_number_of_features_in_image = 45;
generate_random_6D_sphere;
measurements = []; predicted_measurements = [];

% Timing accumulators (DELETE!!!)
t_map = 0; t_pred = 0; t_imread = 0; t_match = 0;
t_ransac = 0; t_update = 0; t_plots = 0;
timing_report_interval = 10; % print every N steps

%---------------------------------------------------------------
% Main loop
%---------------------------------------------------------------

im = takeImage( sequencePath, initIm );

for step=initIm+1:lastIm
    
    p_diag = get_p_k_k(filter);
    if any(isnan(p_diag(:))) || any(isinf(p_diag(:)))
        fprintf('Step %d: NaN/Inf in P BEFORE map_management\n', step);
    end

    % Map management (adding and deleting features; and converting inverse depth to Euclidean)
    tic; [ filter, features_info ] = map_management( filter, features_info, cam, im, min_number_of_features_in_image, step, mask ); t_map = t_map + toc;
    p_diagA = get_p_k_k(filter); if any(isnan(p_diagA(:))), fprintf('Step %d: NaN after map_management\n', step); end

    % EKF prediction (state and measurement prediction)
    tic; [ filter, features_info ] = ekf_prediction( filter, features_info ); t_pred = t_pred + toc;
    p_diagB = get_p_k_k(filter); if any(isnan(p_diagB(:))), fprintf('Step %d: NaN after ekf_prediction\n', step); end

    % Debug: map size
    n_in_map = length(features_info);
    fprintf('Step %d: map=%d\n', step, n_in_map);
    
    % Grab image
    tic; im = takeImage( sequencePath, step ); t_imread = t_imread + toc;
    
    % Search for individually compatible matches
    tic; features_info = search_IC_matches( filter, features_info, cam, im, mask ); t_match = t_match + toc;
    
    % 1-Point RANSAC hypothesis and selection of low-innovation inliers
    tic; features_info = ransac_hypotheses( filter, features_info, cam ); t_ransac = t_ransac + toc;
    
    % Partial update using low-innovation inliers
    tic; filter = ekf_update_li_inliers( filter, features_info );
    p_diag2 = get_p_k_k(filter); if any(isnan(p_diag2(:))), fprintf('Step %d: NaN after ekf_update_li_inliers\n', step); end
    
    % "Rescue" high-innovation inliers
    features_info = rescue_hi_inliers( filter, features_info, cam );
    
    % Partial update using high-innovation inliers
    filter = ekf_update_hi_inliers( filter, features_info ); t_update = t_update + toc;
    p_diag3 = get_p_k_k(filter); if any(isnan(p_diag3(:))), fprintf('Step %d: NaN after ekf_update_hi_inliers\n', step); end

    % Clear h for features projected outside the mask (cosmetic: avoids drawing in black zone)
    if ~isempty(mask)
        for i = 1:length(features_info)
            if ~isempty(features_info(i).h)
                col = round(features_info(i).h(1));
                row = round(features_info(i).h(2));
                if row < 1 || row > cam.nRows || col < 1 || col > cam.nCols || ~mask_rsz(row, col)
                    features_info(i).h = [];
                end
            end
        end
    end

    % Plots,
    tic; plots; display( step ); drawnow
    writeVideo(v, getframe(figure_all)); t_plots = t_plots + toc;

    % Print timing report every N steps
    if mod(step - initIm, timing_report_interval) == 0
        n = step - initIm;
        t_total = t_map + t_pred + t_imread + t_match + t_ransac + t_update + t_plots;
        fprintf('\n--- Timing after %d steps (avg ms/step) ---\n', n);
        fprintf('  map_management : %6.1f ms  (%4.1f%%)\n', t_map/n*1000,   t_map/t_total*100);
        fprintf('  ekf_prediction : %6.1f ms  (%4.1f%%)\n', t_pred/n*1000,  t_pred/t_total*100);
        fprintf('  takeImage      : %6.1f ms  (%4.1f%%)\n', t_imread/n*1000,t_imread/t_total*100);
        fprintf('  search_IC      : %6.1f ms  (%4.1f%%)\n', t_match/n*1000, t_match/t_total*100);
        fprintf('  ransac         : %6.1f ms  (%4.1f%%)\n', t_ransac/n*1000,t_ransac/t_total*100);
        fprintf('  ekf_update     : %6.1f ms  (%4.1f%%)\n', t_update/n*1000,t_update/t_total*100);
        fprintf('  plots+video    : %6.1f ms  (%4.1f%%)\n', t_plots/n*1000, t_plots/t_total*100);
        fprintf('  TOTAL          : %6.1f ms/step\n', t_total/n*1000);
    end

end

%close(v);

save(fullfile(outputFolder, 'trajectory.mat'), 'trajectory');

% 3D camera trajectory plot
figure;
n = lastIm - initIm;
plot3( trajectory(1,1:n), trajectory(2,1:n), trajectory(3,1:n), 'b-', 'LineWidth', 2 );
hold on;
plot3( trajectory(1,1),   trajectory(2,1),   trajectory(3,1),   'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g' );
plot3( trajectory(1,n),   trajectory(2,n),   trajectory(3,n),   'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r' );
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Camera Trajectory');
legend('Trajectory', 'Start', 'End');
grid on; axis equal;