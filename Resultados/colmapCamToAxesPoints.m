function pts = colmapCamToAxesPoints(q, t, axisLength)
    % COLMAPPOSETOAXESPOINTS Converts COLMAP camera pose (W2C) to 4 world points.
    % Inputs:
    %   q          - 4x1 or 1x4 COLMAP quaternion [qw, qx, qy, qz]
    %   t          - 3x1 or 1x3 COLMAP translation vector [tx, ty, tz]
    %   axisLength - Scalar defining the length of the directional axes
    % Output:
    %   pts        - 4x3 matrix where:
    %                pts(1,:) = Camera Center (x, y, z)
    %                pts(2,:) = X-axis endpoint (x, y, z)
    %                pts(3,:) = Y-axis endpoint (x, y, z)
    %                pts(4,:) = Z-axis endpoint (x, y, z)

    % 1. Normalize quaternion and ensure column vectors
    q = q(:) / norm(q);
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    t = t(:);

    % 2. Construct World-to-Camera (W2C) Rotation Matrix from COLMAP quaternion
    R_w2c = [1 - 2*qy^2 - 2*qz^2,   2*qx*qy - 2*qz*qw,   2*qx*qz + 2*qy*qw;
             2*qx*qy + 2*qz*qw,   1 - 2*qx^2 - 2*qz^2,   2*qy*qz - 2*qx*qw;
             2*qx*qz - 2*qy*qw,   2*qy*qz + 2*qx*qw,   1 - 2*qx^2 - 2*qy^2];

    % 3. Invert to Camera-to-World (C2W) to get world coordinates
    % For rotation matrices, the inverse is the transpose.
    R_c2w = R_w2c'; 
    
    % Camera center in world coordinates: C = -R_w2c^T * t
    camCenter = -R_c2w * t;

    % 4. Define local axes in camera space scaled by axisLength
    % In COLMAP: X points right, Y points down, Z points forward into the scene
    localX = [axisLength; 0; 0];
    localY = [0; axisLength; 0];
    localZ = [0; 0; axisLength];

    % 5. Transform local axes to world coordinates and add camera center
    worldX = R_c2w * localX + camCenter;
    worldY = R_c2w * localY + camCenter;
    worldZ = R_c2w * localZ + camCenter;

    % 6. Combine into a 4x3 matrix (each row is a 3D point)
    pts = [camCenter';  ... % Row 1: Origin
           worldX';     ... % Row 2: X-axis
           worldY';     ... % Row 3: Y-axis
           worldZ'];        % Row 4: Z-axis
end
