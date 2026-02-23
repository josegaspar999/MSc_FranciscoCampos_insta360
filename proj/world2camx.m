function [x,X]= world2camx(X, model, cTw)
%   X               3x8               192  double              
%   model           1x1              1440  struct              
%   cTw             4x4               128  double              
%
% model =   struct with fields:
%         ss: [5×1 double]
%         xc: 1440
%         yc: 1440
%          c: 1
%          d: 0
%          e: 0
%      width: 2880
%     height: 2880

% recent "model" versions contain cTw
if nargin<3 && isfield(model, 'rvec')
    cTw= [rodrigues(model.rvec) model.tvec(:); 0 0 0 1];
end

% allow X to be a set of point arrays
if iscell(X)
    x= {};
    for i=1:length(X)
        [x{i}, X{i}]= world2camx(X{i}, model, cTw);
    end
    return
end

% main work to do
X= rigid_transf( X, cTw );
x= world2cam(X, model);
x= img_horiz_mirror( x, model.width );

return % end of main function


function X= rigid_transf( X, cTw )
if size(cTw,1)~=4
    if size(cTw,1)==3
        cTw= [cTw; 0 0 0 1];
    else
        error('cTw not 3 nor 4 lines');
    end
end
X= hrem( cTw*hset(X) );


function x= img_horiz_mirror( x, W )
x(1,:)= W-x(1,:);
