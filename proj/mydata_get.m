function varargout= mydata_get(dataId)

% Usages:
% [model, X,x, L,l]= mydata_get;

if nargin<1
    dataId= 'mXxLl';
end

switch dataId
    case 'mXx'
        % [model, X, x]= get_data1;
        [varargout{1}, varargout{2}, varargout{3}]= get_data1;

    case 'mLl'
        % usage: [model, L, l]= mydata_get('mLl');
        % [model, L, l]= get_data2; % wrong
        % [model, X, x]= get_data2;
        [varargout{1}, varargout{2}, varargout{3}]= get_data2;

    case 'mXxLl'
        % "model" assumes 3D data is in the coordinates frame of the camera
        % [~, X, x]= get_data1;
        % [model, L, l]= get_data2;
        % [model, X,x, L,l]= mydata_get;
        [varargout{1}, varargout{2}, varargout{3}]= get_data1;
        [varargout{1}, varargout{4}, varargout{5}]= get_data2;
        varargout{6}= 'frame7.jpg';

    case 'mXxLl_show'
        % usage: mydata_get('mXxLl_show')
        [model, X,x, L,l]= mydata_get;
        show_datax(model, X, x, L, l)

    case 'ini'
        % [init_r, init_t]= get_init_r_t
        if nargout>=2
            [varargout{1}, varargout{2}]= get_init_r_t;
        else
            varargout{1}= get_init_r_t;
        end


end
% ** ADD here extra 3D & 2D lines so that info is not all in the same 3D
% plane **


function [init_r, init_t]= get_init_r_t
% Initial extrinsics
% usage:
% [init_r, init_t]= get_init_r_t;
% structRt= get_init_r_t;

init_r = [0.0294; 1.8805; -2.4978];
init_t = [963.4143; 340.5490; 200];
%init_t = [963.4143; 340.5490; 1000]; %200];
%init_t = [963.4143; 1000; 1000]; %200];

global MyDataIni
if isfield(MyDataIni, 'r0')
    init_r= MyDataIni.r0;
end
if isfield(MyDataIni, 't0')
    init_t= MyDataIni.t0;
end

if nargout==1
    init_r= struct('init_r',init_r, 'init_t',init_t);
end


% ---------------------------------------------------------------
function [model, X, x]= get_data1
% Coordenadas 3D (em milímetros)
X = [0 1910 790 1085 760 1120 940 940;
    0 0    0   0    0   0    0   0;
    0 0    930 930  625 625  610 590];

x = [2414 502  1730 1215 1826 1136 1487 1485;
    1746 1737 439  429  873  859  907  978];

% calib = load('Omni_Calib_Results.mat');
% model = calib.calib_data.ocam_model;
load('confirmation_my_data.mat', 'model');
% x_test = world2cam(X, model)


function Lines3D= get_data2x( model, R, tvec )
N = 100;
X1 = [zeros(2, N); linspace(0, 2000, N)];
X2 = [ones(1, N)*1910; zeros(1, N); linspace(0, 2000, N)];
X3 = [linspace(0, 1910, N); zeros(2, N)];
% Line1 = R * X1 + tvec(:); x_pred1 = world2cam(Line1, model);
% Line2 = R * X2 + tvec(:); x_pred2 = world2cam(Line2, model);
% Line3 = R * X3 + tvec(:); x_pred3 = world2cam(Line3, model);
Lines3D= {};
for i=1:3
    eval(sprintf('Lines3D{end+1}= R * X%d + tvec(:);', i));
end

% Load the image
%I = imread('frame7_anotada.png');   % replace with your filename
I = imread('frame7.jpg');
figure(201); clf; imshow(I); hold on;
% % Plot line
% plot(x_pred1(1,:), x_pred1(2,:), 'r-', 'LineWidth', 2);
% plot(x_pred2(1,:), x_pred2(2,:), 'r-', 'LineWidth', 2);
% plot(x_pred3(1,:), x_pred3(2,:), 'r-', 'LineWidth', 2);
for i=1:length(Lines3D)
    M= Lines3D{i};
    m= world2cam(M, model);
    plot(m(1,:), m(2,:), 'r-', 'LineWidth', 2);
end
legend({'Predicted'}, 'TextColor','b');

% D = pdist2(x_pred1', l1');
% [~, minIdx] = min(D, [], 2);
%
% % compute residuals (vector form)
% closestPts = l1(:, minIdx);
%
% figure;
% imshow(I); hold on;
%
%
% % Plot line
% plot(x_pred1(1,:), x_pred1(2,:), 'r-', 'LineWidth', 2);
% % plot(l1(1,:), l1(2,:), 'b.', 'MarkerSize', 2);
%
% k = 100;
% plot(x_pred1(1,k), x_pred1(2,k), 'rx', 'MarkerSize', 8, 'LineWidth', 2);
% plot(closestPts(1,k), closestPts(2,k), 'bx', 'MarkerSize', 8, 'LineWidth', 2);


function [model, L, l]= get_data2
% Model
% calib = load('Omni_Calib_Results.mat');
% model = calib.calib_data.ocam_model;
load('confirmation_my_data.mat', 'model');

% Line 3D coordinates
N = 100;
L1 = [zeros(2, N); linspace(0, 2000, N)];
L2 = [linspace(0, 1910, N); zeros(2, N)];
L3 = [ones(1, N)*1910; zeros(1, N); linspace(0, 2000, N)];
L4 = [ones(1, N)*1910; linspace(0, 300, N); zeros(1, N)];
L5 = [zeros(1, N); linspace(0, 300, N); zeros(1, N)];

% Line 2D coordinates
[l1, l2, l3, l4, l5]= get_data2_lines;

L = {L1, L2, L3, L4, L5};
l = {l1, l2, l3, l4, l5};


function [l1, l2, l3, l4, l5]= get_data2_lines
ifname= 'frame7_reta1.png'; ii= [1 410 NaN NaN];
l1= image_to_points( ifname, ii );
ifname= 'frame7_reta2.png'; ii= [1 410 NaN NaN];
l2= image_to_points( ifname, ii );
ifname= 'frame7_reta3.png'; ii= [1 410 1000 NaN];
l3= image_to_points( ifname, ii );
ifname= 'frame7_reta4.png'; ii= [1 410 NaN NaN];
l4= image_to_points( ifname, ii );
ifname= 'frame7_reta5.png'; ii= [1 410 NaN NaN];
l5= image_to_points( ifname, ii );


function lx= image_to_points( ifname, ii )
% Read the image
img = imread(ifname);  % or .jpg, .bmp, etc.

% Ensure it's in uint8 RGB format
if size(img,3) ~= 3
    error('Image is not RGB.');
end

% Extract color channels
R = img(:,:,1); G = img(:,:,2); B = img(:,:,3);

% Define the "pure red" color [255, 0, 0]
redMask = (R > 230);

% Count how many pixels match that color
% numRedPixels = sum(redMask(:));
% % (Optional) Display results
% fprintf('Number of pure red pixels: %d\n', numRedPixels);
% imshow(redMask);
% title('Pure Red Pixels');

redMask= set_false( redMask, ii );
[row, col] = find(redMask);

% Line pixel coordinates
lx = [col.'; row.'];


function redMask= set_false( redMask, ii )
if isnan(ii(1)), ii(1)=1; end
if isnan(ii(2)), ii(2)=size(redMask,1); end
if isnan(ii(3)), ii(3)=1; end
if isnan(ii(4)), ii(4)=size(redMask,2); end
redMask(ii(1):ii(2),ii(3):ii(4))= false;


% ---------------------------------------------------------------
function show_datax(model, X, x, L, l)
%   model      1x1               1440  struct              
%   X          3x8                192  double              
%   x          2x8                128  double  
%   L          1x3               7512  cell                
%   l          1x3             178696  cell                

% model = 
%   struct with fields:
%         ss: [5×1 double]
%         xc: 1440
%         yc: 1440
%          c: 1
%          d: 0
%          e: 0
%      width: 2880
%     height: 2880

% model.ss' =
%  -861.1786         0    0.0005   -0.0000    0.0000

% -- 2D points and lines on the image
figure(301); clf;
I = imread('frame7.jpg'); imshow(I); 
hold on
myplot(x, 'o')
for i=1:length(l)
    myplot( l{i}, '.' )
end
axis equal
axis ij

% -- 3D points and lines
figure(302); clf; hold on
myplot(X, 'o')
for i=1:length(L)
    myplot( L{i}, '.-' )
end
% view(3)
view( -142.1265, 25.1900 )
axis equal
xlabel('x'); ylabel('y'); zlabel('z');

return
