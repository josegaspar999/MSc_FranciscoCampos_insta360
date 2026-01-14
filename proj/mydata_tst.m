function mydata_tst( tstId )
if nargin<1
    tstId= 2; %0; %1;
end
switch tstId
    case 0, MD= data_base;
    case 1, MD= data_load;
    case 2, MD= data_load_chg;
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

subplot(122); axis ij

return


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


function MD= data_load_chg
% initial data
[ocam_model, X,~, L,~]= mydata_get;

% project 3D data
R=rotx(90); t=[-1e3 1e3 -1e3]';
[x,l, X,L]= project_pts_lines(X,L, ocam_model, [R t; 0 0 0 1]);

% reformat data
MD= conv_XxLl_to_MD(X, x, L, l);


function [x,l, X,L]= project_pts_lines(X,L, ocam_model, cTw)
% apply 3D transf
[x,X]= world2camx(X, ocam_model, cTw);
for i=1:length(L)
    [l{i},L{i}]= world2camx(L{i}, ocam_model, cTw);
end
