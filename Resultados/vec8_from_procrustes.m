function vec8 = vec8_from_procrustes(T, c, b)
if nargin==2 && isProcrustesData(T,c)
    % usage: vec8 = vec8_from_procrustes(x,y)
    vec8= procrustes_transf(T,c);
    return
end

% PROCRUSTESTOVECTOR8 Converts Procrustes output to an 8x1 transformation vector.
% Inputs:
%   T - 3x3 rotation matrix from procrustes (TRANSFORM.T)
%   c - 1x3 (or 3x1) translation vector from procrustes (TRANSFORM.c)
%   b - Scalar scale factor from procrustes (TRANSFORM.b)
% Output:
%   vec8 - 8x1 vector [tx; ty; tz; qw; qx; qy; qz; s]

% Ensure translation is a 3x1 column vector
trans = c(:);

% Convert 3x3 rotation matrix T to a unit quaternion [qw; qx; qy; qz]
% Using Stanley's method for numerical stability
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

% Normalize quaternion to ensure it is a unit quaternion
q = [qw; qx; qy; qz];
q = q / norm(q);

% Ensure scale is a single scalar value
scale = b(1);

% Combine into 8x1 vector
vec8 = [trans; q; scale];


function flag= isProcrustesData(x,y)
flag=0;
if size(x,2)==3 && size(y,2)==3 && size(x,1)==size(y,1)
    flag=1;
end


function vec8= procrustes_transf(x,y)
%   T - 3x3 rotation matrix from procrustes (transf.T)
%   c - 1x3 (or 3x1) translation vector from procrustes (transf.c)
%   b - Scalar scale factor from procrustes (transf.b)
[~,~,transf]= procrustes(x,y);
vec8 = vec8_from_procrustes(transf.T, transf.c(1,:), transf.b);
return
