function mydata_tst( tstId )
if nargin<1
    tstId= 3; %2; %0; %1;
end
switch tstId
    case 0, MD= data_base;
    case 1, MD= data_load;
    case 2, MD= data_base_chg(0);
    case 3, MD= data_base_chg(1);
end

% show 2D and 3D data
figure(402+tstId); clf; %hold on
plot_MD( MD )


function plot_MD( MD )
fn= fieldnames(MD);

subplot(121); hold on; axis equal
subplot(122); hold on; axis equal

% plot 3D lines and points
for i=1:length(fn)
    fni= fn{i};
    X= MD.(fni);
    if     strncmp( fni, 'Line', 4 )
        subplot(121); myplot(X, struct('cstr','b.-','str',fni))
    elseif strncmp( fni, 'Pts', 3 )
        subplot(121); myplot(X, struct('cstr','bx','cntFlag',1))
    elseif strncmp( fni, 'line', 4 )
        subplot(122); myplot(X, struct('cstr','b.-','str',fni))
    elseif strncmp( fni, 'pts', 3 )
        subplot(122); myplot(X, struct('cstr','bx','cntFlag',1))
    end
end

subplot(121); box on; grid on
draw_camera(eye(4), struct('scale',1e4)); view(3)

subplot(122); draw_square; axis ij; axis tight

return


function draw_square
W= 2880; m= [0 0; 0 W; W W; W 0; 0 0]';
plot(m(1,:),m(2,:));


function MD= data_load
MD= mydata('load'); MD= MD.MD;
% MD
%   struct with fields:
%          Line1: [3×100 double]
%          line1: [2×100 double]
%          Line2: [3×100 double]
%          line2: [2×100 double]
%          Line3: [3×100 double]
%          line3: [2×100 double]
%          Line4: [3×100 double]
%          line4: [2×100 double]
%          Line5: [3×100 double]
%          line5: [2×100 double]
%            Pts: [3×8 double]
%            pts: [2×8 double]
%     ocam_model: [1×1 struct]
%         params: [15×1 double]
%       worldPts: [3×8 double]
%     worldLines: {1×5 cell}
%           cost: 8.7418e+04
%         ofname: 'mydata_251129_2050.mat'


function MD= data_base
MD= [];
[~, X,x, L,l]= mydata_get;
% show_results2( MD.ocam_model, X, x, L, l, MD.params )
MD= conv_XxLl_to_MD(X, x, L, l);


function MD= conv_XxLl_to_MD(X, x, L, l)
MD= struct('Pts',X, 'pts',x);
for i=1:length(L)
    MD.(sprintf('Line%d',i))= L{i};
    MD.(sprintf('line%d',i))= l{i};
end
return


function MD= data_base_chg( optimFlag )
% initial data
[model, X0,x0, L0,l0, ifname]= mydata_get;

% replace 2D data by reprojected 3D data
R= rotx(90-0); t=[-.9e3 .5e3 -.45e3]';
cTw= [R t; 0 0 0 1];
[x,l, X,L]= project_pts_lines(X0,L0, model, cTw);

% TODO: optimize (R,t) and model such that (x,l) to become close to (x0,l0)
if optimFlag
    [model, cTw]= ocam_optim_calibr( model, cTw, X0,x0, L0,l0, ifname );
    [x,l, X,L]= project_pts_lines(X0,L0, model, cTw);
end

% reformat data, to plot
MD= conv_XxLl_to_MD(X, x, L, l);
return


function [x,l, X,L]= project_pts_lines(X,L, model, cTw)
% apply 3D transf
[x,X]= world2camx(X, model, cTw);
[l,L]= world2camx(L, model, cTw);
% for i=1:length(L)
%     [l{i},L{i}]= world2camx(L{i}, model, cTw);
% end


function [model, cTw]= ocam_optim_calibr( model, cTw, X0,x0, L0,l0, ifname )

% cost= world2camx_cost( params, worldLines, imgLines, worldPts, imgPts, ocam_model )

% R= rotx(90+20); t=[-.9e3 1.1e3 -.5e3]';
% [X, L]= rigid_transf_pts_lines( X, L, R, t );
% [x, l]= mirror_horiz_pts_lines( x, l, model );
% 
% global MyDataIni
% MyDataIni= struct('r0',[0 0 0]', 't0',[0 0 0]');
% [bestParams, fitError] = optimizePoseOcam(L, l, X, x, model);
% MyDataIni= [];
% 
% show_results2( model, X, x, L, l, bestParams );
% plot_scene(X, L, bestParams);
% fprintf('Final reprojection error: %.3f pixels\n', fitError);


return

