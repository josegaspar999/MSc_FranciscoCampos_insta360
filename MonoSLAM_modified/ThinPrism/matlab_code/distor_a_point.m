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

function uvd = distor_a_point( uvu, camera )
% Distort a single undistorted pixel using the thin prism model.

% 1. Normalize to fisheye-projected space
u = (uvu(1) - camera.Cx) / camera.fx;
v = (uvu(2) - camera.Cy) / camera.fy;

% 2. Compute distortion terms
u2  = u^2;
v2  = v^2;
uv_ = u * v;
r2  = u2 + v2;
r4  = r2^2;
r6  = r4 * r2;
r8  = r6 * r2;

radial = camera.k1*r2 + camera.k2*r4 + camera.k3*r6 + camera.k4*r8;

du = u * radial + 2.0*camera.p1*uv_ + camera.p2*(r2 + 2.0*u2) + camera.sx1*r2;
dv = v * radial + 2.0*camera.p2*uv_ + camera.p1*(r2 + 2.0*v2) + camera.sy1*r2;

ud = u + du;
vd = v + dv;

% 3. Back to pixel coordinates
uvd = [ camera.fx * ud + camera.Cx;
        camera.fy * vd + camera.Cy ];
