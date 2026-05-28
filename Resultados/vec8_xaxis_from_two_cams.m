function vec8 = vec8_xaxis_from_two_cams(qA, tA, qB, tB, scale)
    % COLMAPCAMERASTOVECTOR8 Calculates the 8x1 relative transformation (Procrustes-like)
    % from Camera A to Camera B using their COLMAP world-to-camera poses.
    % Inputs:
    %   qA    - 4x1 or 1x4 COLMAP quaternion for Camera A [qw, qx, qy, qz]
    %   tA    - 3x1 or 1x3 COLMAP translation vector for Camera A [tx, ty, tz]
    %   qB    - 4x1 or 1x4 COLMAP quaternion for Camera B [qw, qx, qy, qz]
    %   tB    - 3x1 or 1x3 COLMAP translation vector for Camera B [tx, ty, tz]
    %   scale - Optional scalar scale factor (defaults to 1.0 if not provided)
    % Output:
    %   vec8  - 8x1 vector [tx; ty; tz; qw; qx; qy; qz; s] representing the relative transform

    if nargin < 5
        scale = 1.0;
    end

    % 1. Helper function to extract Camera-to-World (C2W) Rotation and Center
    [R_c2w_A, C_A] = colmapToC2W(qA, tA);
    [R_c2w_B, C_B] = colmapToC2W(qB, tB);

    % 2. Calculate Relative Translation (Difference between camera centers in World space)
    trans = C_B - C_A;

    % 3. Calculate Relative Rotation Matrix (From Camera A to Camera B orientation)
    % T * R_c2w_A = R_c2w_B  =>  T = R_c2w_B * (R_c2w_A)^T
    T = R_c2w_B * (R_c2w_A');

    % 4. Convert the relative rotation matrix T to a unit quaternion [qw; qx; qy; qz]
    % (Using Stanley's method for numerical stability)
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

    % 5. Combine into 8x1 vector [tx; ty; tz; qw; qx; qy; qz; s]
    vec8 = [trans; q; scale];
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
