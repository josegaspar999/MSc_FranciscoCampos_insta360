function [x,X]= world2camx(X, model, cTw)
%   X               3x8               192  double              
%   cTw             4x4               128  double              
%   ocam_model      1x1              1440  struct              
%
% ocam_model =   struct with fields:
%         ss: [5×1 double]
%         xc: 1440
%         yc: 1440
%          c: 1
%          d: 0
%          e: 0
%      width: 2880
%     height: 2880

X= rigid_transf( X, cTw );
x= world2cam(X, model);
x= img_horiz_mirror( x, model.width );


function X= rigid_transf( X, cTw )
X= hrem( cTw*hset(X) );


function x= img_horiz_mirror( x, W )
x(1,:)= W-x(1,:);
