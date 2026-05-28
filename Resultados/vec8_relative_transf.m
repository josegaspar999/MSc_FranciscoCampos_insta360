function vec8Z = vec8_relative_transf(vec8X, vec8Y)
    % COMPUTERELATIVETRANSFORM8 Computes vec8Z that transforms from vec8Y to vec8X.
    % Formally: P_X = s_Z * R(q_Z) * P_Y + t_Z
    %
    % Inputs:
    %   vec8X - 8x1 target vector [tx; ty; tz; qw; qx; qy; qz; s]
    %   vec8Y - 8x1 source vector [tx; ty; tz; qw; qx; qy; qz; s]
    % Output:
    %   vec8Z - 8x1 relative vector [tx; ty; tz; qw; qx; qy; qz; s]

    % 1. Invert the source transformation Y
    vec8YInv = invertVector8(vec8Y);

    % 2. Extract components of X and Y_inv
    t_X = vec8X(1:3);  q_X = vec8X(4:7);  s_X = vec8X(8);
    t_YInv = vec8YInv(1:3); q_YInv = vec8YInv(4:7); s_YInv = vec8YInv(8);

    % 3. Compound Scales: s_Z = s_X * s_YInv
    s_Z = s_X * s_YInv;

    % 4. Compound Quaternions: q_Z = q_X * q_YInv (Quaternion multiplication)
    % Hamilton product formulation for [qw, qx, qy, qz]
    w1 = q_X(1); x1 = q_X(2); y1 = q_X(3); z1 = q_X(4);
    w2 = q_YInv(1); x2 = q_YInv(2); y2 = q_YInv(3); z2 = q_YInv(4);

    qw_Z = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    qx_Z = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    qy_Z = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    qz_Z = w1*z2 + x1*y2 - y1*x2 + z1*w2;
    
    q_Z = [qw_Z; qx_Z; qy_Z; qz_Z];
    q_Z = q_Z / norm(q_Z); % Keep it normalized

    % 5. Compound Translations: t_Z = s_X * R(q_X) * t_YInv + t_X
    % First, build the rotation matrix for X
    qw = q_X(1); qx = q_X(2); qy = q_X(3); qz = q_X(4);
    R_X = [1 - 2*qy^2 - 2*qz^2,   2*qx*qy - 2*qz*qw,   2*qx*qz + 2*qy*qw;
           2*qx*qy + 2*qz*qw,   1 - 2*qx^2 - 2*qz^2,   2*qy*qz - 2*qx*qw;
           2*qx*qz - 2*qy*qw,   2*qy*qz + 2*qx*qw,   1 - 2*qx^2 - 2*qy^2];

    t_Z = s_X * (R_X * t_YInv(:)) + t_X(:);

    % 6. Output final 8x1 vector
    vec8Z = [t_Z; q_Z; s_Z];
    
    if 0 %1 % debug
        vec8X2= vec8_operation('a*b', vec8Z, vec8Y);
        err= vec8X2 - vec8X;
    end
end

function vec8Inv = invertVector8(vec8)
    % Re-used internal inversion helper for standalone execution
    t = vec8(1:3); q = vec8(4:7); s = vec8(8);
    if abs(s) < 1e-9, error('Scale factor is zero.'); end
    sInv = 1.0 / s;
    qInv = [q(1); -q(2); -q(3); -q(4)];
    qInv = qInv / norm(qInv);
    qw = qInv(1); qx = qInv(2); qy = qInv(3); qz = qInv(4);
    R_inv = [1 - 2*qy^2 - 2*qz^2,   2*qx*qy - 2*qz*qw,   2*qx*qz + 2*qy*qw;
             2*qx*qy + 2*qz*qw,   1 - 2*qx^2 - 2*qz^2,   2*qy*qz - 2*qx*qw;
             2*qx*qz - 2*qy*qw,   2*qy*qz + 2*qx*qw,   1 - 2*qx^2 - 2*qy^2];
    tInv = -sInv * (R_inv * t(:));
    vec8Inv = [tInv; qInv; sInv];
end
