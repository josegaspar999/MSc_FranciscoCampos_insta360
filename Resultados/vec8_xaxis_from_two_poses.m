function vec8 = vec8_xaxis_from_two_poses(qA, tA, qB, tB)
    % COMPUTETARGETEDTRANSFORM Computes a vec8 transform [tx;ty;tz; qw;qx;qy;qz; s]
    % using inputs that are ALREADY in World Coordinates (C2W Poses).
    %
    % Inputs:
    %   qA - 4x1 or 1x4 quaternion for Camera A (C2W orientation) [qw, qx, qy, qz]
    %   tA - 3x1 or 1x3 vector for Camera A (C2W center/position) [tx, ty, tz]
    %   qB - 4x1 or 1x4 quaternion for Camera B (C2W orientation) [qw, qx, qy, qz]
    %   tB - 3x1 or 1x3 vector for Camera B (C2W center/position) [tx, ty, tz]
    % Output:
    %   vec8 - 8x1 vector [tx; ty; tz; qw; qx; qy; qz; s]

    % 1. Ensure inputs are column vectors
    C_A = tA(:);
    C_B = tB(:);
    qA  = qA(:) / norm(qA);

    % 2. Construct the C2W Rotation Matrix for Camera A directly from qA
    qw = qA(1); qx = qA(2); qy = qA(3); qz = qA(4);
    R_c2w_A = [1 - 2*qy^2 - 2*qz^2,   2*qx*qy - 2*qz*qw,   2*qx*qz + 2*qy*qw;
               2*qx*qy + 2*qz*qw,   1 - 2*qx^2 - 2*qz^2,   2*qy*qz - 2*qx*qw;
               2*qx*qz - 2*qy*qw,   2*qy*qz + 2*qx*qw,   1 - 2*qx^2 - 2*qy^2];

    % 3. Define the New Coordinate System Axes (in World Space)
    
    % New X-axis: Points from Camera A center to Camera B center
    dirAB = C_B - C_A;
    distAB = norm(dirAB);
    if distAB < 1e-6
        error('Camera A and Camera B centers are coincident. Cannot define X-axis.');
    end
    new_X = dirAB / distAB;

    % Camera A's local x-axis expressed in world space
    local_x_A_world = R_c2w_A(:, 1); 

    % Constraint: Camera A's local x-axis must lie on the new XY plane.
    % This means the new Z-axis must be perpendicular to both the new X-axis 
    % AND Camera A's local x-axis.
    new_Z = cross(new_X, local_x_A_world);
    normZ = norm(new_Z);
    
    if normZ < 1e-6
        % Fallback if Camera A's local x-axis is already perfectly aligned with the AB vector
        new_Z = cross(new_X, [0; 0; 1]);
        if norm(new_Z) < 1e-6
            new_Z = cross(new_X, [0; 1; 0]);
        end
        new_Z = new_Z / norm(new_Z);
    else
        new_Z = new_Z / normZ;
    end

    % New Y-axis completion (Z cross X)
    new_Y = cross(new_Z, new_X);
    new_Y = new_Y / norm(new_Y);

    % 4. Construct the Transformation Matrix (World to Target Frame)
    R_target = [new_X'; new_Y'; new_Z']; 
    t_target = -R_target * C_A; % Shifts C_A to [0; 0; 0]

    % 5. Convert R_target to Quaternion [qw; qx; qy; qz] (Stanley's Method)
    T = R_target;
    tr = trace(T);
    if tr > 0
        S = sqrt(tr + 1.0) * 2;
        qw = 0.25 * S;
        qx = (T(2,3) - T(3,2)) / S;
        qy = (T(3,1) - T(1,3)) / S;
        qz = (T(1,2) - T(2,1)) / S;
    elseif (T(1,1) > T(2,2)) && (T(1,1) > T(3,3))
        S = sqrt(1.0 + T(1,1) - T(2,2) - T(3,3)) * 2;
        qw = (T(2,3) - T(3,2)) / S;
        qx = 0.25 * S;
        qy = (T(1,2) + T(2,1)) / S;
        qz = (T(3,1) + T(1,3)) / S;
    elseif T(2,2) > T(3,3)
        S = sqrt(1.0 + T(2,2) - T(1,1) - T(3,3)) * 2;
        qw = (T(3,1) - T(1,3)) / S;
        qx = (T(1,2) + T(2,1)) / S;
        qy = 0.25 * S;
        qz = (T(2,3) + T(3,2)) / S;
    else
        S = sqrt(1.0 + T(3,3) - T(1,1) - T(2,2)) * 2;
        qw = (T(1,2) - T(2,1)) / S;
        qx = (T(3,1) + T(1,3)) / S;
        qy = (T(2,3) + T(3,2)) / S;
        qz = 0.25 * S;
    end

    q_target = [qw; qx; qy; qz];
    q_target = q_target / norm(q_target);

    scale = 1.0; 

    % Combine into final 8x1 vector
    vec8 = [t_target; q_target; scale];
end
