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
function newFeature = hinv( uvd, Xv, camera, initial_rho )

  uv = undistort_fm( uvd, camera );
  u = uv(1);
  v = uv(2);

  % Inverse fisheye: undistorted pixel -> normalized 3D ray direction
  uf = (u - camera.Cx) / camera.fx;
  vf = (v - camera.Cy) / camera.fy;
  r_theta = sqrt(uf^2 + vf^2);
  if r_theta > 1e-12
      r_tan = tan(r_theta);
      xn = uf * (r_tan / r_theta);
      yn = vf * (r_tan / r_theta);
  else
      xn = uf;
      yn = vf;
  end
  h_LR = [xn; yn; 1];

  r_W = Xv(1:3);
  q_WR = Xv(4:7);

  n = q2r(q_WR) * h_LR;
  nx = n(1);
  ny = n(2);
  nz = n(3);

  newFeature = [ r_W; atan2(nx,nz); atan2(-ny,sqrt(nx*nx+nz*nz)); initial_rho ];

return