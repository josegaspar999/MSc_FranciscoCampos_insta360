function pts = vec8_pose_to_axes_points(vec8, axisLength)
if numel(vec8)>8
    pts = vec8_pose_to_axes_points_nposes(vec8, axisLength);
    return
end

    % VEC8POSETOAXESPOINTS Converts a vec8 camera pose to 4 world points.
    % Inputs:
    %   vec8       - 8x1 vector [tx; ty; tz; qw; qx; qy; qz; s] (C2W pose)
    %   axisLength - Scalar defining the base length of the directional axes
    % Output:
    %   pts        - 4x3 matrix where:
    %                pts(1,:) = Camera Center (x, y, z)
    %                pts(2,:) = X-axis endpoint (x, y, z)
    %                pts(3,:) = Y-axis endpoint (x, y, z)
    %                pts(4,:) = Z-axis endpoint (x, y, z)

    % 1. Extract components from vec8
    camCenter = vec8(1:3);
    q = vec8(4:7);
    s = vec8(8);

    % 2. Ensure correct formatting and normalize quaternion for quat2rotm
    q_row = q(:)' / norm(q);

    % 3. Convert quaternion to Camera-to-World (C2W) Rotation Matrix
    R_c2w = quat2rotm(q_row);

    % 4. Define local axes in camera space scaled by axisLength AND vec8 internal scale
    totalLength = axisLength * s;
    localX = [totalLength; 0; 0];
    localY = [0; totalLength; 0];
    localZ = [0; 0; totalLength];

    % 5. Transform local axes to world coordinates and add camera center
    worldX = R_c2w * localX + camCenter(:);
    worldY = R_c2w * localY + camCenter(:);
    worldZ = R_c2w * localZ + camCenter(:);

    % 6. Combine into a 4x3 matrix (each row is a 3D point)
    pts = [camCenter(:)'; ... % Row 1: Origin
           worldX';        ... % Row 2: X-axis
           worldY';        ... % Row 3: Y-axis
           worldZ'];           % Row 4: Z-axis
end


function pts = vec8_pose_to_axes_points_nposes(vec8, axisLength)
if size(vec8,1)~=8
    error('input not 8 rows');
end
pts= zeros(4*size(vec8,2), 3);
i1= 1;
i2= 4;
for c=1:size(vec8,2)
    pts(i1:i2,:)= vec8_pose_to_axes_points(vec8(:,c), axisLength);
    i1= i1+4;
    i2= i2+4;
end
end
