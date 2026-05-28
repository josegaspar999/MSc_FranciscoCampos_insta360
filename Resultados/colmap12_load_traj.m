function [ret1, ret2] = colmap12_load_traj(filename, options)
if nargin<2
    options= [];
end

[~,~,ext]= fileparts(filename);
if strcmp(ext, '.txt')
    [frame_id, COLMAP] = colmap12_load_traj_txt(filename, options);
    ret1=frame_id; ret2=COLMAP;
elseif strcmp(ext, '.mat')
    [traj, n1] = colmap12_load_traj_mat(filename, options);
    ret1=traj; ret2=n1;
else
    error('inv ext')
end


function [traj, n1] = colmap12_load_traj_mat(filename, options)
load( filename, 'trajectory' );
traj= trajectory;

if isfield(options, 'rawData') && options.rawData
    n1= [];
    return
end

% If no zero column exists, use full length
idx1 = find(all(traj == 0, 1), 1);
if isempty(idx1)
    n1 = size(traj, 2);
else
    n1 = idx1 - 1;
end


function [frame_id, COLMAP] = colmap12_load_traj_txt(filename, options)

fid = fopen(filename, 'r');
data = textscan(fid, ...
    '%f %f %f %f %f %f %f %f %f %*[^\n]', ...
    'CommentStyle', '#');
fclose(fid);

if isfield(options, 'rawData') && options.rawData
    frame_id = data;
    COLMAP= [];
    return
end

frame_id = data{1};

QW = data{3};
QX = data{4};
QY = data{5};
QZ = data{6};

TX = data{7};
TY = data{8};
TZ = data{9};

COLMAP = zeros(length(frame_id), 3);

for i = 1:length(frame_id)
    
    q = [QW(i), QX(i), QY(i), QZ(i)];
    t = [TX(i); TY(i); TZ(i)];
    
    R = quat2rotm(q);
    
    % Camera center in world coordinates
    C = -R' * t;
    
    COLMAP(i,:) = C';
    
end
