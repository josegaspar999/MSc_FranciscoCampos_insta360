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

function P_RES = add_a_feature_covariance_inverse_depth( P, uvd, Xv, std_pxl, std_rho, cam )

q_wc = Xv(4:7);
R_wc = q2r(q_wc);

uvu = undistort_fm( uvd, cam );
uu = uvu(1);
vu = uvu(2);

% Inverse fisheye: undistorted pixel -> normalized 3D ray
uf = (uu - cam.Cx) / cam.fx;
vf = (vu - cam.Cy) / cam.fy;
r_theta = sqrt(uf^2 + vf^2);
if r_theta > 1e-12
    r_tan = tan(r_theta);
    X_c = uf * (r_tan / r_theta);
    Y_c = vf * (r_tan / r_theta);
else
    X_c = uf;
    Y_c = vf;
end
Z_c = 1;

XYZ_c = [X_c; Y_c; Z_c];

XYZ_w = R_wc*XYZ_c;
X_w = XYZ_w(1);
Y_w = XYZ_w(2);
Z_w = XYZ_w(3);

% Derivatives

dtheta_dgw = [ Z_w/(X_w^2+Z_w^2) 0 -X_w/(X_w^2+Z_w^2) ];
dphi_dgw = [ (X_w*Y_w)/((X_w^2+Y_w^2+Z_w^2)*sqrt(X_w^2+Z_w^2))...
        -sqrt(X_w^2+Z_w^2)/(X_w^2+Y_w^2+Z_w^2) (Z_w*Y_w)/((X_w^2+Y_w^2+Z_w^2)*sqrt(X_w^2+Z_w^2)) ];
dgw_dqwr = dRq_times_a_by_dq( q_wc, XYZ_c );

dtheta_dqwr = dtheta_dgw*dgw_dqwr;
dphi_dqwr = dphi_dgw*dgw_dqwr;
dy_dqwr = sparse( [ zeros(3,4); dtheta_dqwr; dphi_dqwr; zeros(1,4) ] );
dy_drw = sparse( [ eye(3); zeros(3,3) ] );

dy_dxv = sparse( [ dy_drw dy_dqwr zeros(6,6) ] );

dyprima_dgw = sparse( [ zeros(3,3); dtheta_dgw; dphi_dgw ] );
dgw_dgc = R_wc;

% Jacobian of camera ray w.r.t. undistorted pixels (fisheye inverse)
% dgc_dhu: d[X_c;Y_c;Z_c] / d[uu;vu]  (3x2)
r2 = uf^2 + vf^2;
if r2 > 1e-12
    r  = sqrt(r2);
    th = atan(r);
    sc = th / r;
    ds_dr  = (r/(1+r2) - th) / r^2;
    A = sc + uf^2/r * ds_dr;
    B = uf*vf/r  * ds_dr;
    C = sc + vf^2/r * ds_dr;
else
    A = 1; B = 0; C = 1;
end
dgc_dhu = [ A/cam.fx,  B/cam.fy;
             B/cam.fx,  C/cam.fy;
             0,         0       ];

dhu_dhd = jacob_undistor_fm( cam , uvd );

dyprima_dhd = dyprima_dgw*dgw_dgc*dgc_dhu*dhu_dhd;

dy_dhd = sparse( [ dyprima_dhd zeros(5,1);
            zeros(1,2)      1 ] );

Ri = speye(2)*std_pxl^2;

Padd = sparse( [Ri  zeros(2,1);
    zeros(1,2)  std_rho^2] );

%
P_xv = P( 1:13, 1:13 );
P_yxv = P( 14:end, 1:13 );
P_y = P( 14:end, 14:end );
P_xvy = P( 1:13, 14:end );
P_RES = [ P_xv          P_xvy                       P_xv*dy_dxv';
          P_yxv         P_y                         P_yxv*dy_dxv';
          dy_dxv*P_xv   dy_dxv*P_xvy                dy_dxv*P_xv*dy_dxv'+...
                                                    dy_dhd*Padd*dy_dhd'];