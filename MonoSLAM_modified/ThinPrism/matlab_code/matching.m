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

function features_info = matching( im, features_info, cam )

correlation_threshold = 0.80;

chi_095_2 = 5.9915;
% chi_099_2 = 9.2103;

for i_feature=1:length(features_info) % for every feature in the map
    
    if ~isempty(features_info(i_feature).h); % if it is predicted, search in the region
        
        h = features_info(i_feature).h;
        S = features_info(i_feature).S;
        
        half_patch_size_when_matching = features_info(i_feature).half_patch_size_when_matching;
        pixels_in_the_matching_patch = (2*half_patch_size_when_matching+1)^2;
        
        if eig(S)< 8100 %if the ellipse is too big, do not search (something may be wrong)
            
            invS = inv(S);
            
            predicted_patch = double(features_info(i_feature).patch_when_matching);
            
            % skip if predicted patch is invalid (NaN from out-of-bounds warp)
            if any(isnan(predicted_patch(:)))
                continue;
            end
            
            half_search_region_size_x = min(ceil(2*sqrt(S(1,1))), 60);
            half_search_region_size_y = min(ceil(2*sqrt(S(2,2))), 60);

            % --- Old: nested loop extracting patches one by one ---
            % patches_for_correlation = zeros( pixels_in_the_matching_patch,...
            %     (2*half_search_region_size_x+1)*(2*half_search_region_size_y+1) + 1);
            % match_candidates = zeros( 2,(2*half_search_region_size_x+1)*(2*half_search_region_size_y+1) + 1 );
            % patches_for_correlation( :, 1 ) = reshape( predicted_patch, pixels_in_the_matching_patch, 1 );
            % index_patches_for_correlation = 1;
            % for j = round(h(1))-half_search_region_size_x:round(h(1))+half_search_region_size_x
            %     for i = round(h(2))-half_search_region_size_y:round(h(2))+half_search_region_size_y
            %         nu = [ j-h(1); i-h(2) ];
            %         if ( (nu'*invS*nu) < chi_095_2 )
            %             if (j>half_patch_size_when_matching)&&(j<cam.nCols-half_patch_size_when_matching)&&...
            %                     (i>half_patch_size_when_matching)&&(i<cam.nRows-half_patch_size_when_matching)
            %                 image_patch = double( im( i-half_patch_size_when_matching:i+half_patch_size_when_matching,...
            %                     j-half_patch_size_when_matching:j+half_patch_size_when_matching ) );
            %                 index_patches_for_correlation = index_patches_for_correlation + 1;
            %                 patches_for_correlation(:,index_patches_for_correlation) = reshape(image_patch, pixels_in_the_matching_patch, 1);
            %                 match_candidates( :, index_patches_for_correlation - 1 ) = [ j; i ];
            %             end
            %         end
            %     end
            % end
            % --- Old: NCC via corrcoef or manual matrix ---
            % (removed, replaced by normxcorr2 below)

            % --- New: normxcorr2 (compiled C, FFT-based) replaces all loops ---
            hw  = half_patch_size_when_matching;
            hsx = half_search_region_size_x;
            hsy = half_search_region_size_y;
            hc  = round(h(1));  % predicted col (x)
            hr  = round(h(2));  % predicted row (y)

            % Extract padded region: hw border so template fits at all search positions
            r1 = hr - hsy - hw;  r2 = hr + hsy + hw;
            c1 = hc - hsx - hw;  c2 = hc + hsx + hw;

            % Skip if region goes outside image
            if r1 < 1 || r2 > cam.nRows || c1 < 1 || c2 > cam.nCols
                continue;
            end

            C = normxcorr2(predicted_patch, double(im(r1:r2, c1:c2)));

            % Valid part of C: size (2*hsy+1) x (2*hsx+1)
            % C_valid(i,j) = NCC at image position (hr-hsy+i-1, hc-hsx+j-1)
            C_valid = C(2*hw+1 : 2*hw+1+2*hsy, 2*hw+1 : 2*hw+1+2*hsx);

            % Mask positions outside the chi-squared ellipse
            [nu_c, nu_r] = meshgrid(-hsx:hsx, -hsy:hsy);
            chi_sq = invS(1,1)*nu_c.^2 + 2*invS(1,2)*nu_c.*nu_r + invS(2,2)*nu_r.^2;
            C_valid(chi_sq >= chi_095_2) = -inf;

            [maximum_correlation, lin_idx] = max(C_valid(:));
            if maximum_correlation >= correlation_threshold
                [best_i, best_j] = ind2sub(size(C_valid), lin_idx);
                features_info(i_feature).individually_compatible = 1;
                features_info(i_feature).z = [hc - hsx + best_j - 1; hr - hsy + best_i - 1];
            end
            
        end
        
    end
    
end
        