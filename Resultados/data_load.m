function [x, name]= data_load( dataId, formId )

% dataId: colmapFisheye, colmapPinhole, monoslamFisheye, monoslamPinhole
% formId: worldFrames, cameraFrames

if nargin<1
    %     for i=1:4, data_load(i); end % just for demo
    %     return
    dataId= 'cmp1234';
end
if nargin<2
    formId= 'array (3+4) x N';
end

if strcmp(dataId, 'cmp1234')
    [x, name]= load_cmp1234(0);
    return
end
if strcmp(dataId, 'cmp2143')
    [x, name]= load_cmp1234(1);
    return
end

switch dataId
    case {1, 'colmapFisheye'},   name='CFisheye'; ifname='COLMAP fisheye/text/frames.txt';
    case {2, 'colmapPinhole'},   name='CPPinhole'; ifname='COLMAP pinhole/text/frames.txt';
    case {3, 'monoslamFisheye'}, name='MFisheye'; ifname='MonoSLAM fisheye/final/trajectory.mat';
    case {4, 'monoslamPinhole'}, name='MPinhole'; ifname='MonoSLAM pinhole/final/trajectory.mat';
    otherwise, erro('inv dataId %s', dataId);
end

[~,~,e]= fileparts(ifname);
if strcmp(e, '.txt')
    x= load_colmap(ifname, formId);
elseif strcmp(e, '.mat')
    x= load_monoslam(ifname, formId);
else
    error('inv ext')
end

%disp(x)
return % end of main function


% -------------------------------------------------------------
function [xx, leg]= load_cmp1234( flag2143 )
xx= {}; leg= {};
iRange= 1:4; %[2 1 3 4]; %1:4;
if flag2143
    iRange= [2 1 4 3];
end
for i= iRange
    [x, name]= data_load(i);
    if i==3 || i==4
        x= x(:,1:10:end-1);
    end
    xx{end+1}= x;
    leg{end+1}= name;
end
% leg= {'CFisheye','CPPinhole','MFisheye','MPinhole'};
% leg= leg(iRange);


% -------------------------------------------------------------
function x= load_colmap(ifname, formId)
% [frame_id, COLMAP] = colmap12_load_traj(ifname);
% %   COLMAP        1487x3             35688  double              
% %   frame_id      1487x1             11896  double              
% x= COLMAP;

if ~strcmp(formId, 'array (3+4) x N')
    error('unsupported formId');
end

d= colmap12_load_traj(ifname, struct('rawData',1));
% d : 1x9 cell : FRAME_ID, RIG_ID, RIG_FROM_WORLD[QW, QX, QY, QZ, TX, TY, TZ]
%                1         2                      3   4   5   6   7   8   9

x= [];
for i= [7:9 3:6]
    x= [x; d{i}'];
end

x= conv_cam_to_world( x );

return


function COLMAP= conv_cam_to_world( data )
% y= Rx+t -> R'(y-t)=x -> x= R'y -R't
COLMAP = zeros(size(data));
for i = 1:size(data,2)
    q = data(4:7,i);
    t = data(1:3,i);
    R = quat2rotm(q');
    COLMAP(:,i)= [-R'*t; rotm2quat(R')'];
end


% -------------------------------------------------------------
function x= load_monoslam(ifname, formId)
% [traj, n1] = colmap12_load_traj(ifname);
% %   n1          1x1                     8  double              
% %   traj        7x14871            832776  double 
% x= traj;

if ~strcmp(formId, 'array (3+4) x N')
    error('unsupported formId');
end

x= colmap12_load_traj(ifname, struct('rawData',1));
%   x           7x14871            832776  double  
% each column of x: [x y z ...]

return
