function z = vec8_operation(op, x, y, z)
switch op
    case 'invert', z= vec8_invert(x);
    case 'a*b', z= applyTransformToPose8(x, y);
    case 'z-x*y'
        z= z-applyTransformToPose8(x, y);
    case 'unitScale', z = makeUnitaryScale8(x);
    case 'vec8_to_vec7'
        z = makeUnitaryScale8(x);
        z = z(1:end-1,:);
    otherwise, error('unknown op');
end


function vec8Inv = vec8_invert(vec8)
% INVERTVECTOR8 Computes the mathematical inverse of an 8x1 transformation vector.
% Input:
%   vec8    - 8x1 vector [tx; ty; tz; qw; qx; qy; qz; s]
% Output:
%   vec8Inv - 8x1 inverted vector [tx; ty; tz; qw; qx; qy; qz; s]

% 1. Extract components
t = vec8(1:3);
q = vec8(4:7);
s = vec8(8);

% Validate scale to avoid division by zero
if abs(s) < 1e-9
    error('Scale factor is too close to zero to invert.');
end

% 2. Compute Inverse Scale
sInv = 1.0 / s;

% 3. Compute Inverse Quaternion (Conjugate of a unit quaternion)
qInv = [q(1); -q(2); -q(3); -q(4)];
qInv = qInv / norm(qInv); % Ensure numerical stability

% 4. Build rotation matrix from qInv to transform the translation vector
qw = qInv(1); qx = qInv(2); qy = qInv(3); qz = qInv(4);
R_inv = [1 - 2*qy^2 - 2*qz^2,   2*qx*qy - 2*qz*qw,   2*qx*qz + 2*qy*qw;
    2*qx*qy + 2*qz*qw,   1 - 2*qx^2 - 2*qz^2,   2*qy*qz - 2*qx*qw;
    2*qx*qz - 2*qy*qw,   2*qy*qz + 2*qx*qw,   1 - 2*qx^2 - 2*qy^2];

% 5. Compute Inverse Translation: t_inv = -(1/s) * R(q_inv) * t
tInv = -sInv * (R_inv * t(:));

% 6. Combine into final 8x1 vector
vec8Inv = [tInv; qInv; sInv];


function vec8New = applyTransformToPose8(vec8Transform, vec8Pose)
if size(vec8Transform,1)~=8 || size(vec8Pose,1)~=8
    error('num of rows in a or b is not 8')
end
if size(vec8Pose,2)>1
    vec8New = applyTransformToPose8_multiple(vec8Transform, vec8Pose);
    return
end

% APPLYTRANSFORMTOPOSE8 Applies a vec8 transformation to a vec8 camera pose.
% Formally: P_new = s_T * R(q_T) * P_old + t_T
%
% Inputs:
%   vec8Transform - 8x1 transformation vector [tx; ty; tz; qw; qx; qy; qz; s]
%   vec8Pose      - 8x1 initial camera pose vector [tx; ty; tz; qw; qx; qy; qz; s]
% Output:
%   vec8New       - 8x1 transformed camera pose vector

% 1. Extract components from the transformation (Z)
t_Z = vec8Transform(1:3);
q_Z = vec8Transform(4:7);
s_Z = vec8Transform(8);

% 2. Extract components from the initial pose (A)
t_A = vec8Pose(1:3);
q_A = vec8Pose(4:7);
s_A = vec8Pose(8);

% 3. Compute New Scale
s_new = s_Z * s_A;

% 4. Compute New Orientation (Quaternion Multiplication: q_Z * q_A)
% quat2rotm and rotm2quat require row vectors
w1 = q_Z(1); x1 = q_Z(2); y1 = q_Z(3); z1 = q_Z(4);
w2 = q_A(1); x2 = q_A(2); y2 = q_A(3); z2 = q_A(4);

qw_new = w1*w2 - x1*x2 - y1*y2 - z1*z2;
qx_new = w1*x2 + x1*w2 + y1*z2 - z1*y2;
qy_new = w1*y2 - x1*z2 + y1*w2 + z1*x2;
qz_new = w1*z2 + x1*y2 - y1*x2 + z1*w2;

q_new = [qw_new; qx_new; qy_new; qz_new];
q_new = q_new / norm(q_new); % Normalize to prevent numerical drift

% 5. Compute New Position (Transforming the camera center coordinate)
q_Z_row = q_Z(:)' / norm(q_Z);
R_Z = quat2rotm(q_Z_row);

t_new = s_Z * (R_Z * t_A(:)) + t_Z(:);

% 6. Combine into final 8x1 vector
vec8New = [t_new; q_new; s_new];


function  vec8New = applyTransformToPose8_multiple(vec8Transform, vec8Pose)

if size(vec8Transform,2)~=size(vec8Pose,2)
    error('input matrices with diff sizes')
end

vec8New = vec8Pose;
for i=1:size(vec8New,2)
    vec8New(:,i)= applyTransformToPose8(vec8Transform(:,i), vec8Pose(:,i));
end


function xNew = makeUnitaryScale8(x)
% MAKEUNITARYSCALE8 Modifies a collection of vec8 vectors to have unitary scale (s=1).
% It adjusts the translation vectors accordingly to preserve geometric consistency.
%
% Input:
%   x    - 8xM matrix where each column is a vec8 pose
%          [tx; ty; tz; qw; qx; qy; qz; s]
% Output:
%   xNew - 8xM matrix with all scale factors set to 1.0

% 1. Copy the original matrix
xNew = x;

% 2. Extract all original scale factors (Row 8, size: 1xM)
scales = x(8, :);

% Avoid division by zero if any scale factor is unexpectedly 0
if any(abs(scales) < 1e-9)
    error('One or more vec8 vectors have a scale factor of zero and cannot be normalized.');
end

% 3. Rescale translations (Rows 1 to 3) by dividing by the original scales
% MATLAB automatically broadcasts the 1xM row vector across the 3xM rows
xNew(1:3, :) = x(1:3, :) ./ repmat(scales,3,1);;

% 4. Set all scale factors (Row 8) to exactly 1.0
xNew(8, :) = 1.0;
