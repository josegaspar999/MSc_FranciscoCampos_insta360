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

function J_undistor = jacob_undistor_fm(camera, uvd)
% Jacobian of undistortion w.r.t. distorted pixel coordinates.
% Returns d(uv_undistorted)/d(uv_distorted) [2x2].
% Strategy: compute the closed-form Jacobian of distortion
%           d(uv_distorted)/d(uv_undistorted), then invert it.
%
% Input:  uvd     - 2x1 distorted pixel coordinates
% Output: J_undistor - 2x2 Jacobian of undistortion

% 1. Undistort to get undistorted pixels, then normalize to fisheye space
uvu = undistor_a_point(uvd, camera);
u = (uvu(1) - camera.Cx) / camera.fx;
v = (uvu(2) - camera.Cy) / camera.fy;

% 2. Compute distortion Jacobian in fisheye-normalized space
u2   = u^2;
v2   = v^2;
r2   = u2 + v2;
r4   = r2^2;
r6   = r4 * r2;
r8   = r6 * r2;

D       = camera.k1*r2 + camera.k2*r4 + camera.k3*r6 + camera.k4*r8;
dD_dr2  = camera.k1 + 2*camera.k2*r2 + 3*camera.k3*r4 + 4*camera.k4*r6;

% d(ud)/d(u) and d(ud)/d(v)
dud_du = 1 + D + 2*u2*dD_dr2  + 2*camera.p1*v  + 6*camera.p2*u  + 2*camera.sx1*u;
dud_dv =     2*u*v*dD_dr2     + 2*camera.p1*u  + 2*camera.p2*v  + 2*camera.sx1*v;

% d(vd)/d(u) and d(vd)/d(v)
dvd_du =     2*u*v*dD_dr2     + 2*camera.p2*v  + 2*camera.p1*u  + 2*camera.sy1*u;
dvd_dv = 1 + D + 2*v2*dD_dr2  + 2*camera.p2*u  + 6*camera.p1*v  + 2*camera.sy1*v;

% 3. Scale to pixel space: p_ud = fx*ud + cx, p_uu = fx*u + cx
%    d(p_ud)/d(p_uu) = d(ud)/d(u)          (fx cancels)
%    d(p_ud)/d(p_vu) = (fx/fy)*d(ud)/d(v)
%    d(p_vd)/d(p_uu) = (fy/fx)*d(vd)/d(u)
%    d(p_vd)/d(p_vu) = d(vd)/d(v)          (fy cancels)
ratio = camera.fx / camera.fy;
J_distort = [ dud_du,          ratio * dud_dv;
              (1/ratio)*dvd_du, dvd_dv        ];

% 4. Jacobian of undistortion is the inverse
% Guard against NaN/Inf/singular J_distort (e.g. extreme pixels with degenerate distortion)
if any(isnan(J_distort(:))) || any(isinf(J_distort(:))) || rcond(J_distort) < 1e-12
    J_undistor = eye(2);
else
    J_undistor = inv(J_distort);
end