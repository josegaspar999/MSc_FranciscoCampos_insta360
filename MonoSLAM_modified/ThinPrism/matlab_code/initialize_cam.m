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

function cam = initialize_cam()

nRows = 2880;
nCols = 2880;

% Thin prism model parameters: fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, sx1, sy1
fx  = 852.13705237092313;
fy  = 850.17193586488634;
cx  = 1440;
cy  = 1440;
k1  = 0.089108710858519902;
k2  = -0.040448487168936217;
p1  = -0.00051923858012622775;
p2  = -0.00032993437201914437;
k3  = 0.016053928938995387;
k4  = -0.0038407855887184119;
sx1 = 0.0019045576073914582;
sy1 = 0.0032420175002338338;


cam.nRows = nRows;
cam.nCols = nCols;
cam.fx =    fx;
cam.fy =    fy;
cam.Cx =    cx;
cam.Cy =    cy;
cam.k1 =    k1;
cam.k2 =    k2;
cam.p1 =    p1;
cam.p2 =    p2;
cam.k3 =    k3;
cam.k4 =    k4;
cam.sx1 =   sx1;
cam.sy1 =   sy1;
cam.model = 'thinprism';

% Keep backward-compatible fields used in existing code
cam.f  = fx;
cam.dx = 1;
cam.dy = 1;

cam.K = sparse( [ fx   0   cx;
                   0  fy   cy;
                   0   0    1] );