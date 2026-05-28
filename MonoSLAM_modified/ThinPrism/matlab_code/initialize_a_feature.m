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

function [ filter, features_info, uv ] = initialize_a_feature( step, cam, im_k, filter, features_info, mask )

% numerical values
half_patch_size_when_initialized = 20;
half_patch_size_when_matching = 15;
excluded_band = half_patch_size_when_initialized + 1;
max_initialization_attempts = 2;
initializing_box_size = [540,360];
initializing_box_semisize = initializing_box_size/2;
min_feature_separation = 45;
initial_rho = 1;
std_rho = 1;

std_pxl = get_std_z(filter);

rand_attempt = 1;
not_empty_box = 1;
detected_new=0;


features_info = predict_camera_measurements( get_x_k_k(filter), cam, features_info );

uv_pred = [];
for i=1:length(features_info)
    uv_pred = [uv_pred features_info(i).h'];
end

for i=1:max_initialization_attempts
    
    % if a feature has been initialized, exit
    if ( detected_new )
        break;
    end
    
%     search_region_center = rand(2,1);
%     search_region_center(1) = round(search_region_center(1)*(cam.nCols-2*excluded_band-2*initializing_box_semisize(1)))...
%         +excluded_band+initializing_box_semisize(1);
%     search_region_center(2) = round(search_region_center(2)*(cam.nRows-2*excluded_band-2*initializing_box_semisize(2)))...
%         +excluded_band+initializing_box_semisize(2);

    % Compute valid center bounds (box must fit within image + excluded band)
    min_col = excluded_band + initializing_box_semisize(1) + 1;
    max_col = cam.nCols - excluded_band - initializing_box_semisize(1);
    min_row = excluded_band + initializing_box_semisize(2) + 1;
    max_row = cam.nRows - excluded_band - initializing_box_semisize(2);

    % Resize mask to image size if needed, then crop to valid center region
    if ~isempty(mask)
        if size(mask,1) ~= cam.nRows || size(mask,2) ~= cam.nCols
            mask_img = imresize(mask, [cam.nRows, cam.nCols], 'nearest');
        else
            mask_img = mask;
        end
        valid_region = mask_img(min_row:max_row, min_col:max_col);
    else
        valid_region = true(max_row-min_row+1, max_col-min_col+1);
    end

    [valid_rows, valid_cols] = find(valid_region);
    if isempty(valid_rows)
        uv = [];
        break;
    end
    idx = randi(length(valid_rows));
    search_region_center = [valid_cols(idx) + min_col - 1; valid_rows(idx) + min_row - 1];
    
    % Extract FAST corners
    cd fast-matlab-src
    cs = fast_corner_detect_9(double(im_k(search_region_center(2)-initializing_box_semisize(2):search_region_center(2)+initializing_box_semisize(2),...
        search_region_center(1)-initializing_box_semisize(1):search_region_center(1)+initializing_box_semisize(1))),... % the image,
        20);
    c = fast_nonmax(double(im_k(search_region_center(2)-initializing_box_semisize(2):search_region_center(2)+initializing_box_semisize(2),...
        search_region_center(1)-initializing_box_semisize(1):search_region_center(1)+initializing_box_semisize(1))),... % the image,
        20, cs);
    all_uv = c';
    cd ..
    if ~isempty(all_uv)
        all_uv = all_uv + [ (- initializing_box_semisize(1) + search_region_center(1) - 1)*ones(1,size(all_uv,2));...
            (- initializing_box_semisize(2) + search_region_center(2) - 1)*ones(1,size(all_uv,2))];
    end
    
    % Are there corners in the box?
    are_there_corners = not(isempty(all_uv));

    if are_there_corners
        uv = all_uv;
        uv_integer = uv;

        % Keep only corners sufficiently far from already-predicted features.
        if ~isempty(uv_pred)
            valid_idx = false(1, size(uv,2));
            for k = 1:size(uv,2)
                du = uv_pred(1,:) - uv(1,k);
                dv = uv_pred(2,:) - uv(2,k);
                min_dist2 = min(du.^2 + dv.^2);
                valid_idx(k) = (min_dist2 >= min_feature_separation^2);
            end
            uv = uv(:, valid_idx);
        end

        if ~isempty(uv)
            idx_corner = randi(size(uv,2));
            uv = uv(:,idx_corner);% - [0.5,0.5]';
            detected_new = 1;
        else
            uv = [];
        end
    else
        uv = [];
    end
    
    if(~isempty(uv))
        
        % Skip if undistortion of this pixel produces NaN (degenerate pixel)
        uvu_test = undistort_fm(uv, cam);
        if any(isnan(uvu_test(:))) || any(isinf(uvu_test(:)))
            return;
        end
        
        % add the feature to the filter
        [ X_RES, P_RES, newFeature ] = add_features_inverse_depth( uv, get_x_k_k(filter),...
            get_p_k_k(filter), cam, std_pxl, initial_rho, std_rho );
        filter = set_x_k_k(filter, X_RES);
        filter = set_p_k_k(filter, P_RES);
        
        % add the feature to the features_info vector
        features_info = add_feature_to_info_vector( uv, im_k, X_RES, features_info, step, newFeature );
        
    end
    
    for i=1:length(features_info)
        features_info(i).h = [];
    end
    
end