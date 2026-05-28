function vec8= vec8_force_xalign( qA, tA, qB, tB, camsFlag )
if nargin<5
    camsFlag= 0;
end

if camsFlag
    % given COLMAP cameras
    vec8 = computeTargetedTransform(qA, tA, qB, tB);
else
    % given world poses
    vec8 = computeTargetedTransform_fromPoses(qA, tA, qB, tB);
end

end


function vec8 = computeTargetedTransform_fromPoses(qA, tA, qB, tB)
    % COMPUTETARGETEDTRANSFORM Computes a vec8 transform [tx;ty;tz; qw;qx;qy;qz; s]
    % considering that the inputs are already given in World Coordinates (C2W).
    %
    % Inputs:
    %   qA, tA - Camera A pose in World coordinates (tA is 3D center, qA is orientation)
    %   qB, tB - Camera B pose in World coordinates (tB is 3D center)
    % Output:
    %   vec8   - 8x1 vector [tx; ty; tz; qw; qx; qy; qz; s]

    % 1. Ensure column vectors for positions
    C_A = tA(:);
    C_B = tB(:);

    % 2. Convert Camera A's C2W quaternion directly to a 3x3 rotation matrix
    qA = qA(:) / norm(qA);
    qwA = qA(1); qxA = qA(2); qyA = qA(3); qzA = qA(4);
    R_c2w_A = [1 - 2*qyA^2 - 2*qzA^2,   2*qxA*qyA - 2*qzA*qwA,   2*qxA*qzA + 2*qyA*qwA;
               2*qxA*qyA + 2*qzA*qwA,   1 - 2*qxA^2 - 2*qzA^2,   2*qyA*qzA - 2*qxA*qwA;
               2*qxA*qzA - 2*qyA*qwA,   2*qyA*qzA + 2*qxA*qwA,   1 - 2*qxA^2 - 2*qyA^2];

    % 3. Define the New Coordinate System Axes
    
    % New X-axis: Points from Camera A to Camera B
    dirAB = C_B - C_A;
    distAB = norm(dirAB);
    if distAB < 1e-6
        error('Camera A and Camera B centers are coincident. Cannot define X-axis.');
    end
    new_X = dirAB / distAB;

    % Camera A's local x-axis in world space is just the first column of its C2W matrix
    local_x_A_world = R_c2w_A(:, 1); 

    % Constraint: Camera A's local x-axis must lie on the new XY plane.
    % This implies the new Z-axis is perpendicular to both new_X and local_x_A_world.
    new_Z = cross(new_X, local_x_A_world);
    normZ = norm(new_Z);
    
    if normZ < 1e-6
        % Degenerate case fallback
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

    % 4. Construct the Transformation Matrix
    % R_target maps from World to the Target Frame
    R_target = [new_X'; new_Y'; new_Z']; 
    
    % Target translation shifts C_A to the origin
    t_target = -R_target * C_A;

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

    q = [qw; qx; qy; qz];
    q = q / norm(q);

    scale = 1.0; 

    % Combine into final 8x1 vector
    vec8 = [t_target; q; scale];
end


function vec8 = computeTargetedTransform(qA, tA, qB, tB)
    % COMPUTETARGETEDTRANSFORM Computes a vec8 transform [tx;ty;tz; qw;qx;qy;qz; s]
    % that moves Camera A to the origin, aligns the A-to-B vector with the new X-axis,
    % and projects Camera A's local x-axis onto the new XY plane.
    %
    % Inputs:
    %   qA, tA - COLMAP pose for Camera A (W2C)
    %   qB, tB - COLMAP pose for Camera B (W2C)
    % Output:
    %   vec8   - 8x1 vector [tx; ty; tz; qw; qx; qy; qz; s]

    % 1. Extract Camera Centers and Rotations in World Coordinates (C2W)
    [R_c2w_A, C_A] = colmapToC2W(qA, tA);
    [~, C_B]       = colmapToC2W(qB, tB);

    % 2. Define the New Coordinate System Axes (in World Space)
    
    % New X-axis: Points from Camera A to Camera B
    dirAB = C_B - C_A;
    distAB = norm(dirAB);
    if distAB < 1e-6
        error('Camera A and Camera B centers are coincident. Cannot define X-axis.');
    end
    new_X = dirAB / distAB;

    % We need Camera A's local x-axis in world space to find the roll alignment
    local_x_A_world = R_c2w_A(:, 1); 

    % Constraint: Camera A's local x-axis must lie on the new XY plane.
    % This implies the new Z-axis must be perpendicular to both the new X-axis 
    % AND Camera A's local x-axis.
    new_Z = cross(new_X, local_x_A_world);
    normZ = norm(new_Z);
    
    if normZ < 1e-6
        % Degenerate case: Camera A's local x-axis is already collinear with the AB vector.
        % Fallback to a generic perpendicular vector using world Z-axis
        new_Z = cross(new_X, [0; 0; 1]);
        if norm(new_Z) < 1e-6
            new_Z = cross(new_X, [0; 1; 0]);
        end
        new_Z = new_Z / norm(new_Z);
    else
        new_Z = new_Z / normZ;
    end

    % New Y-axis: Right-handed system completion (Z cross X)
    new_Y = cross(new_Z, new_X);
    new_Y = new_Y / norm(new_Y);

    % 3. Construct the Transformation Matrix
    % R_target maps from World to the Target Frame
    R_target = [new_X'; new_Y'; new_Z']; 
    
    % Target translation shifts C_A to the origin
    t_target = -R_target * C_A;

    % 4. Convert R_target to Quaternion [qw; qx; qy; qz] (Stanley's Method)
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

    q = [qw; qx; qy; qz];
    q = q / norm(q);

    % Scale factor is 1.0 since we are rigidly transforming the existing coordinate space
    scale = 1.0; 

    % Combine into final 8x1 vector
    vec8 = [t_target; q; scale];
end

function [R_c2w, camCenter] = colmapToC2W(q, t)
    % Helper function to convert COLMAP W2C parameters to C2W space
    q = q(:) / norm(q);
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    t = t(:);

    % Construct W2C Rotation Matrix
    R_w2c = [1 - 2*qy^2 - 2*qz^2,   2*qx*qy - 2*qz*qw,   2*qx*qz + 2*qy*qw;
             2*qx*qy + 2*qz*qw,   1 - 2*qx^2 - 2*qz^2,   2*qy*qz - 2*qx*qw;
             2*qx*qz - 2*qy*qw,   2*qy*qz + 2*qx*qw,   1 - 2*qx^2 - 2*qy^2];

    % Invert to Camera-to-World
    R_c2w = R_w2c'; 
    camCenter = -R_c2w * t;
end
