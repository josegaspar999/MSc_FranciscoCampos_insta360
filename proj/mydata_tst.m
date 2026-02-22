function mydata_tst( tstId )
% mydata_tst(0) % image data
% mydata_tst(1) % optimiz reached (wrong)
% mydata_tst(3) % optimiz from orig data

if nargin<1
    tstId= 3; %0:3; %3; %2; %0; %1;
end
if length(tstId)>1
    for i=tstId, mydata_tst(i); end
    return
end

switch tstId
    case 0, MD= data_base; tstr= 'orig data';
    case 1, MD= data_load; tstr= 'old optimiz';
    case 2, MD= data_base_chg(0); tstr= 'new ^cT_w';
    case 3, MD= data_base_chg(1); tstr= 'new ^cT_w & optimiz';
end

% show 2D and 3D data
%figure(402+tstId); clf; %hold on
figure(800+tstId); clf; %hold on
plot_MD( MD, tstr )


% ----------------------------------------------------------
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
return


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


% ----------------------------------------------------------
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


function r = invRodrigues(R)
theta = acos( (trace(R) - 1) / 2 );
if abs(theta) < 1e-12
    r = [0;0;0];
    return;
end
u = 1/(2*sin(theta)) * [
    R(3,2) - R(2,3);
    R(1,3) - R(3,1);
    R(2,1) - R(1,2)
];
r = theta * u;


function [x,l, X,L]= project_pts_lines(X,L, model, cTw)
% apply 3D transf
[x,X]= world2camx(X, model, cTw);
[l,L]= world2camx(L, model, cTw);


function [model, cTw]= ocam_optim_calibr( model, cTw, X0,x0, L0,l0, ifname0 )
%
% model = struct with fields:
%         ss: [5×1 double]
%         xc: 1440
%         yc: 1440
%          c: 1
%          d: 0
%          e: 0
%      width: 2880
%     height: 2880

% --- cost function:
% cost= world2camx_cost( params, worldLines, imgLines, worldPts, imgPts )
costFun = @(params) world2camx_cost( params, L0,l0, X0,x0 );

% --- starting data:
bfname= './mydata_tst_';
d= dir([bfname '*.mat']);
if ~isempty(d)
    fname= filenames_last_only( [bfname '*.mat'] );
    load(fname, 'ifname', 'm2','p2','c2')
    if ~strcmp(ifname0, ifname)
        button= questdlg('Loaded ifname mismatch ifname0. Abort?', ...
            'Yes','No','No');
        if strcmp(button, 'Yes')
            error('Aborted work.');
        end
    end
    m1= m2; p1= p2; c1= c2;
else
    m1= model; % add cTw into model "m1", and convert to an array "params":
    m1.tvec= cTw(1:3,4);
    m1.rvec= invRodrigues(cTw(1:3,1:3));
    p1= world2camx_params( 'struct2params', m1 );
    c1= costFun( p1 );
end

% --- optimization:
global MyCost
MyCost= inf;
p2= fminsearch( costFun, p1 );
c2= costFun( p2 );
m2= world2camx_params( 'params2struct', p2 );

% --- save data:
fname= mkfname( bfname, 'mat', struct('outputFormat',2) );
ifname= ifname0;
save(fname, 'ifname', 'm1','p1','c1', 'm2','p2','c2', ...
    'L0','l0', 'X0','x0');

% --- return results:
model= m2;
cTw= [rodrigues(model.rvec(:)) model.tvec(:)];
return

