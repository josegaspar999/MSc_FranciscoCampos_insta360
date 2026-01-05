% Coordenadas 3D (em milímetros)
X = [0 1910 790 1085 760 1120 940 940;
     0 0    0   0    0   0    0   0;
     0 0    930 930  625 625  610 590];
x = [2414 502  1730 1215 1826 1136 1487 1485;
     1746 1737 439  429  873  859  907  978];

%% 
img = imread('frame7_reta1.png');
if size(img,3) ~= 3
    error('Image is not RGB.');
end
R = img(:,:,1);
G = img(:,:,2);
B = img(:,:,3);
redMask = (R > 230);
numRedPixels = sum(redMask(:));
redMask(1:410, :) = false;
fprintf('Number of pure red pixels: %d\n', numRedPixels);
imshow(redMask);
title('Pure Red Pixels');
[row, col] = find(redMask);
l1 = [col.'; row.'];

img = imread('frame7_reta2.png');
if size(img,3) ~= 3
    error('Image is not RGB.');
end
R = img(:,:,1);
G = img(:,:,2);
B = img(:,:,3);
redMask = (R > 230);
numRedPixels = sum(redMask(:));
redMask(1:410, :) = false;
fprintf('Number of pure red pixels: %d\n', numRedPixels);
imshow(redMask);
title('Pure Red Pixels');
[row, col] = find(redMask);
l2 = [col.'; row.'];

img = imread('frame7_reta3.png');
if size(img,3) ~= 3
    error('Image is not RGB.');
end
R = img(:,:,1);
G = img(:,:,2);
B = img(:,:,3);
redMask = (R > 230);
numRedPixels = sum(redMask(:));
redMask(1:410, :) = false;
fprintf('Number of pure red pixels: %d\n', numRedPixels);
imshow(redMask);
title('Pure Red Pixels');
[row, col] = find(redMask);
l3 = [col.'; row.'];

img = imread('frame7_reta4.png');
if size(img,3) ~= 3
    error('Image is not RGB.');
end
R = img(:,:,1);
G = img(:,:,2);
B = img(:,:,3);
redMask = (R > 230);
numRedPixels = sum(redMask(:));
redMask(1:410, :) = false;
fprintf('Number of pure red pixels: %d\n', numRedPixels);
imshow(redMask);
title('Pure Red Pixels');
[row, col] = find(redMask);
l4 = [col.'; row.'];

img = imread('frame7_reta5.png');
if size(img,3) ~= 3
    error('Image is not RGB.');
end
R = img(:,:,1);
G = img(:,:,2);
B = img(:,:,3);
redMask = (R > 230);
numRedPixels = sum(redMask(:));
redMask(1:410, :) = false;
fprintf('Number of pure red pixels: %d\n', numRedPixels);
imshow(redMask);
title('Pure Red Pixels');
[row, col] = find(redMask);
l5 = [col.'; row.'];


%% 
% Model
model = load('model.mat');

% Line 3D coordinates
N = 100;
L1 = [zeros(2, N); linspace(0, 1200, N)];
L2 = [linspace(0, 1910, N); zeros(2, N)];
L3 = [ones(1, N)*1910; zeros(1, N); linspace(0, 1200, N)];
L4 = [ones(1, N)*1910; linspace(0, 300, N); zeros(1, N)];
L5 = [zeros(1, N); linspace(0, 300, N); zeros(1, N)];

L = {L1, L2, L3, L4, L5};
l = {l1, l2, l3, l4, l5};

[bestParams, rmsError] = optimizeOcam(L, l, X, x, model, true, 'lsq');


rvec = bestParams(1:3);
tvec = bestParams(4:6);
R = rodrigues(rvec);
model.c  = bestParams(7);
model.d  = bestParams(8);
model.e  = bestParams(9);
model.xc = bestParams(10);
model.yc = bestParams(11);
model.ss(1:size(bestParams,1)-11) = bestParams(12:end);



L1_c = R * L1 + tvec(:);
l1_pred = world2cam(L1_c, model);

L2_c = R * L2 + tvec(:);
l2_pred = world2cam(L2_c, model);

L3_c = R * L3 + tvec(:);
l3_pred = world2cam(L3_c, model);

L4_c = R * L4 + tvec(:);
l4_pred = world2cam(L4_c, model);

L5_c = R * L5 + tvec(:);
l5_pred = world2cam(L5_c, model);

X_c = R * X + tvec(:);
x_pred = world2cam(X_c, model);


% Load the image
I = imread('frame7.jpg');

figure;
imshow(I); hold on;

% Plot measured lines (red dots)
plot(l1(1,:), l1(2,:), 'r.', 'MarkerSize', 2);
plot(l2(1,:), l2(2,:), 'r.', 'MarkerSize', 2);
plot(l3(1,:), l3(2,:), 'r.', 'MarkerSize', 2);
plot(l4(1,:), l4(2,:), 'r.', 'MarkerSize', 2);
plot(l5(1,:), l5(2,:), 'r.', 'MarkerSize', 2);

% Plot predicted lines (blue lines)
plot(l1_pred(1,:), l1_pred(2,:), 'b-', 'LineWidth', 2);
plot(l2_pred(1,:), l2_pred(2,:), 'b-', 'LineWidth', 2);
plot(l3_pred(1,:), l3_pred(2,:), 'b-', 'LineWidth', 2);
plot(l4_pred(1,:), l4_pred(2,:), 'b-', 'LineWidth', 2);
plot(l5_pred(1,:), l5_pred(2,:), 'b-', 'LineWidth', 2);

% Plot measured points (ground truth, red circles)
plot(x(1,:), x(2,:), 'ro', 'MarkerSize', 8, 'LineWidth', 2);

% Plot predicted points (blue crosses)
plot(x_pred(1,:), x_pred(2,:), 'bx', 'MarkerSize', 8, 'LineWidth', 2);

% Connect corresponding points to visualize errors (yellow dashed)
for i = 1:size(x,2)
    line([x(1,i) x_pred(1,i)], [x(2,i) x_pred(2,i)], ...
        'Color','y','LineStyle','--','LineWidth',1.5);
end

% Create dummy handles for legend appearance
hMeasured  = plot(nan, nan, 'r-', 'LineWidth', 2);  % red line
hPredicted = plot(nan, nan, 'b-', 'LineWidth', 2);  % blue line
hError     = plot(nan, nan, 'y--', 'LineWidth', 1.5); % yellow dashed

% Legend with nice visual line styles
legend([hMeasured, hPredicted, hError], {'Measured', 'Predicted', 'Error'});
hold off;


%% 
figure;
hold on;
view(3); axis equal; grid on; box on; 
xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]')

img = imread('frame7.jpg');
[Ximg, Zimg] = meshgrid(linspace(0,1910,size(img,2)), ...
                        linspace(0,1200,size(img,1)));
Yimg = zeros(size(Ximg)); % y=0 plane
surf(Ximg, Yimg, Zimg, flipud(img), ...
     'FaceColor', 'texturemap', 'EdgeColor', 'none');

P= [R (rvec(:)'*-inv(R))'];
draw_camera(P, struct('scale', 5, 'sc_axis', 2));
for i = 1:numel(L)
    Li = L{i};
    plot3(Li(1, :), Li(2, :), Li(3, :), 'r', 'LineWidth', 2);
end
plot3(X(1,:), X(2,:), X(3,:), 'r.', 'MarkerSize', 15);
axis([-90 2000 0 2000 0 1300]);


%% 
function [bestParams, rmsError] = optimizeOcam(worldLines, imgLines, worldPts, imgPts, model, optParams, solver)

    % A complete function that refines the pose and/or parameters of the
    % Scaramuzza camera model given a set of lines and/or points on an
    % image.

    % Step 1: Initialization
    init_r = [0.0294; 1.8805; -2.4978];         % Initial extrinsics
    init_t = [963.4143; 340.5490; -469.8617];
    
    init_c  = model.c;                          % Initial intrinsics (original model)
    init_d  = model.d;
    init_e  = model.e;
    init_xc = model.xc;
    init_yc = model.yc;
    init_ss  = model.ss(1:4);
    
    % Step 2: Construct initial estimate
    if optParams == true
        x0 = [init_r; init_t; init_c; init_d; init_e; init_xc; init_yc; init_ss(:)];
    else
        x0 = [init_r; init_t];
    end
    
    % Step 3: Declare cost function
    costFun = @(params) objFun(params, worldLines, imgLines, worldPts, imgPts, model, optParams);

    % Step 4: Startup solver
    if strcmpi(solver, 'lsq')
        opts = optimoptions('lsqnonlin','Display','iter','MaxIterations',1000);
        [bestParams, resnorm] = lsqnonlin(costFun, x0, [], [], opts);
    elseif strcmpi(solver, 'fmin')
        fun = @(p) norm(costFun(p));
        opts = optimset('Display','iter','MaxIter',500,'TolX',1e-6,'TolFun',1e-6);
        [bestParams, resnorm] = fminsearch(fun, x0, opts);
    else
        error("Unknown mode: %s", mode);
    end
    
    % Step 5: Print final results
    residuals = costFun(bestParams);
    rmsError = sqrt(mean(residuals.^2));
    fprintf('Final RMS reprojection error: %.3f pixels\n', rmsError);
end

function r = objFun(params, worldLines, imgLines, worldPts, imgPts, model, optParams)

    % Unpack extrinsics
    rvec = params(1:3);
    tvec = params(4:6);

    % Unpack intrinsics to fine-tune (if wanted)
    if optParams == true
        idx = 6;
        model.c  = params(idx+1); idx = idx + 1;
        model.d  = params(idx+1); idx = idx + 1;
        model.e  = params(idx+1); idx = idx + 1;
        model.xc = params(idx+1); idx = idx + 1;
        model.yc = params(idx+1); idx = idx + 1;
        model.ss(1:size(params,1)-idx) = params(idx+1:end);
    end

    % Rodrigues rotation
    R = rodrigues(rvec);
    
    % Project lines
    s = 0;
    r = [];
    for i = 1:numel(worldLines)
        
        Xc = (R * worldLines{i} + tvec(:));
        proj = world2cam(Xc, model);
    
        % Compute residuals (lines)
        D = pdist2(proj', imgLines{i}');
        [~, minIdx] = min(D, [], 2);
    
        closestPts = imgLines{i}(:, minIdx);
        diff = proj - closestPts;
        r_i = diff(:);
        
        r = [r; r_i];
        s = s + size(worldLines{i}, 2);
    end

    % Scale factor
    if s == 0
        f = 1;
    else
        f = s/size(worldPts, 2);
    end

    % Project points
    if size(worldPts) ~= 0
        Xc = (R * worldPts + tvec(:));
        proj = world2cam(Xc, model);
        
        % Compute residuals (pairs of points)
        diff = proj - imgPts;
        r_i = f * diff(:);
        r = [r; r_i];
    end
end

