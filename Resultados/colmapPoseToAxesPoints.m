function pts = colmapPoseToAxesPoints(q, t, axisLength)
    % COLMAPPOSETOAXESPOINTS Converts a world-space camera pose (C2W) to 4 world points.
    % Inputs:
    %   q          - 4x1 or 1x4 quaternion (C2W orientation) [qw, qx, qy, qz]
    %   t          - 3x1 or 1x3 vector (C2W camera center) [tx, ty, tz]
    %   axisLength - Scalar defining the length of the directional axes
    % Output:
    %   pts        - 4x3 matrix where:
    %                pts(1,:) = Camera Center (x, y, z)
    %                pts(2,:) = X-axis endpoint (x, y, z)
    %                pts(3,:) = Y-axis endpoint (x, y, z)
    %                pts(4,:) = Z-axis endpoint (x, y, z)

    % 1. Ensure correct formatting and normalize quaternion
    q = q(:)' / norm(q); % quat2rotm expects a row vector [qw, qx, qy, qz]
    camCenter = t(:);    % In C2W, t is already the camera center in world coordinates

    % 2. Convert quaternion directly to Camera-to-World (C2W) Rotation Matrix
    R_c2w = quat2rotm(q);

    % 3. Define local axes in camera space scaled by axisLength
    % In COLMAP convention: X points right, Y points down, Z points forward
    localX = [axisLength; 0; 0];
    localY = [0; axisLength; 0];
    localZ = [0; 0; axisLength];

    % 4. Transform local axes to world coordinates and add camera center
    worldX = R_c2w * localX + camCenter;
    worldY = R_c2w * localY + camCenter;
    worldZ = R_c2w * localZ + camCenter;

    % 5. Combine into a 4x3 matrix (each row is a 3D point)
    pts = [camCenter';  ... % Row 1: Origin
           worldX';     ... % Row 2: X-axis
           worldY';     ... % Row 3: Y-axis
           worldZ'];        % Row 4: Z-axis
end
