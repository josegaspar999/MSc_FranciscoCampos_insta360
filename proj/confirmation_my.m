function confirmation_my(tstId)
% folder "251111_insta360_calibr\." is older than "251121_insta360_calibr\."
% but has a more recent (this file) "confirmation_my.m"

if nargin<1
    tstId= [-1 -2]; %-2; %-1; %-2; %2; %1;
end
if length(tstId)>1
    for i=tstId, confirmation_my(i); end
    return
end

if ~exist('Scaramuzza_OCamCalib_v3.0_win', 'dir')
    if exist('H:\Escola\IST\Mestrado\Tese\Calibração\results_vid2', 'dir')
        addpath('H:\Escola\IST\Mestrado\Tese\Calibração\results_vid2');
        addpath('H:\Escola\IST\Mestrado\Tese\Calibração\Scaramuzza_OCamCalib_v3.0_win');
    elseif exist('U:\TFC\cave3\SVN\sw\Scaramuzza_OCamCalib_v3.0_win', 'dir')
        path(path, 'U:\TFC\cave3\SVN\sw\Scaramuzza_OCamCalib_v3.0_win')
    else
        warning('Scaramuzza''s calibration toolbox not found')
    end
end

switch tstId
    case -1, tst0(1)
    case -2, tst0(2)
    case -3, tst0(3)
    case -4, tst0(4)
    case 1, tst1
    case 2, tst2
end


% -------------------------------------------------------------------------------------
function tst0( dataId )
% Ocam at calibration imaged 3D points as (x,y,z)=(0,0,-500)
[ocam_model, X, x, L, l]= mydata_get;
%[N, M0, zRefList]= mk_3d_pts( 1 );
[N, M0, zRefList]= mk_3d_pts( dataId );

nIter = numel(zRefList);     % FC
colorCycle = [1 0 0;   % red
              0 1 0;   % green
              1 1 0];  % yellow     % FC

figure(100+dataId); clf
for k = 1:nIter             % FC
    zRef = zRefList(k);     % FC
    M= M0+repmat([0 0 zRef]',1,N);
    m= world2cam(M, ocam_model);

    c = colorCycle(mod(k-1,3)+1, :);    % FC

    subplot(121); hold on; plot3(M(1,:),M(2,:),M(3,:),'.-','Color',c);  % FC
    subplot(122); hold on; plot(m(1,:),m(2,:),'.-','Color',c);          % FC
end

viewMode= 2; %1;

subplot(121);
draw_camera(eye(4), struct('scale', 1000)); axis equal; box on; grid on; view(3)
switch viewMode
    case 1, view([-7.2409  -58.2000]); title('view from bottom')
    case 2, view([-7.5535 36.2821]); title('view from box top')
end
axis tight
name_axis('XYZ');

subplot(122);
axis equal; draw_square();
switch viewMode
    case 1, axis ij;
    case 2, axis xy;
end
grid on; axis tight
name_axis('xy');

return


function [N, M0, zRefList]= mk_3d_pts( dataId )
if dataId == 1
    N=10;
    rng(0); % set fixed seed for the random number generator
    M0= [[0 0 0]' diag([50 50 0])*randn(3,N-1)];
    zRefList= [-50 -400];

elseif dataId == 2
    R= -50:10:50;
    [x,y]= meshgrid(R,R);
    M0= [x(:)'; y(:)'; 0*x(:)'];
    N= size(M0,2);
    zRefList= [-10 -50 -200 -400];

elseif dataId == 3
    theta = pi; % 180 degrees
    Rz = [cos(theta), -sin(theta), 0;
          sin(theta),  cos(theta), 0;
          0,           0,          1];

    M0 = [0, 0.51, -0.81, 0.81, -0.51, 0;
    -0.85, 0.68, -0.26, -0.26, 0.68, -0.85;
          0,0,0,0,0, 0]*30;

    M0 = Rz*M0;

    N = size(M0,2);
    zRefList= [-5 -20 -50 -100 -200 -300];

elseif dataId == 4

    M0 = [-10  0              10 -10
          -3  15*sqrt(3)/2-3 -3  -3
          0  0                0   0];

    N = 4
    zRefList= [-20 20];


end


function draw_square( W )
if nargin<1
    W= 2880;
end
m= [0 0; 0 W; W W; W 0; 0 0]';
plot(m(1,:),m(2,:));


function name_axis(names)
a='xyz';
for i=1:min(3,length(names))
    str= sprintf('%slabel(''%s'')', a(i), names(i));
    eval(str);
end


% -------------------------------------------------------------------------------------
function tst1
[model, X, x]= mydata_get('mXx'); % get just points
[bestParams, fitError] = optimizePose(X, x, model);
show_results1( model, X, x, bestParams );
fprintf('Final reprojection error (1): %.3f pixels\n', fitError);

[model, L, l]= mydata_get('mLl'); % get just lines
[bestParams, fitError] = optimizePoseOcam(L, l, X, x, model);
show_results2( model, X, x, L, l, bestParams );
fprintf('Final reprojection error (2): %.3f pixels\n', fitError);


function tst2
[model, X, x, L, l]= mydata_get;
plot_scene( X, L, mydata_get('ini') );

[bestParams, fitError] = optimizePoseOcam(L, l, X, x, model);
show_results2( model, X, x, L, l, bestParams );
plot_scene(X, L, bestParams);
fprintf('Final reprojection error: %.3f pixels\n', fitError);
return



function m = insta360proj(M, model, cTw)

    % m:   2xN
    % M:   3xN
    % cTw: 3x4 [cRw ctw]
    
    N = size(M, 2);
    if size(cTw,1) == 3, cTw = [cTw; 0 0 0 1]; end
    if size(M,1) == 3, M = hset(M); end

    Mc = hrem(cTw * M);
    xy = world2cam(Mc, model);
    
    % "conv_xy_to_m"
    m = [xy(1,:); ones(1,N)*model.height - xy(2,:)];

return


% -------------------------------------------------------------------------------------
function [bestParams, fitError] = optimizePoseOcam(worldLines, imgLines, worldPts, imgPts, ocam_model)

% --- Initial extrinsics ---
% % init_r = [0.0294; 1.8805; -2.4978];
% % init_t = [963.4143; 340.5490; -469.8617];
% init_r = [0.0294; 1.8805; -2.4978];
% init_t = [963.4143; 340.5490; 200];
[init_r, init_t]= mydata_get('ini');


% --- Initial intrinsics from ocam_model ---
init_c  = ocam_model.c;
init_d  = ocam_model.d;
init_e  = ocam_model.e;
init_xc = ocam_model.xc;
init_yc = ocam_model.yc;
init_ss  = ocam_model.ss(1:4);

% Concatenate all parameters
x0 = [init_r; init_t; init_c; init_d; init_e; init_xc; init_yc; init_ss(:)];

% Startup optimization
global MyCost
MyCost = inf;

% Run the optimization
if 0
    costFun = @(params) reprojResidualsOcam(params, worldLines, imgLines, worldPts, imgPts, ocam_model);
    opts = optimoptions('lsqnonlin','Display','iter','MaxIterations',1000);
    [bestParams, ~] = lsqnonlin(costFun, x0, [], [], opts);
    residuals = costFun(bestParams);
    fitError = sqrt(mean(residuals.^2));
else
    x1= params_select(1, x0, 1:6 );
    costFun = @(params) norm(reprojResidualsOcam(params, worldLines, imgLines, worldPts, imgPts, ocam_model));
    [bestParams, fitError] = fminsearch(costFun, x1);
end


% -------------------------------------------------------------------------------------
function r_all = reprojResidualsOcam(params, worldLines, imgLines, worldPts, imgPts, ocam_model)

params= params_select(2, params); % expand params

% Unpack extrinsics
rvec = params(1:3);
tvec = params(4:6);

% % Unpack intrinsics to fine-tune
idx = 6;
ocam_model.c  = params(idx+1); idx = idx + 1;
ocam_model.d  = params(idx+1); idx = idx + 1;
ocam_model.e  = params(idx+1); idx = idx + 1;
ocam_model.xc = params(idx+1); idx = idx + 1;
ocam_model.yc = params(idx+1); idx = idx + 1;

% Update polynomial coefficients (only first n_ss_opt)
ocam_model.ss(1:size(params,1)-idx) = params(idx+1:end);

% Rodrigues rotation
R = rodrigues(rvec);

% Lines
% Project via Scaramuzza model
r_all = [];
mydata('reset');
for i = 1:numel(worldLines)

    Xc = (R * worldLines{i} + tvec(:));
    %proj = world2cam(Xc, ocam_model);
    proj = insta360proj(worldLines{i}, ocam_model, [R tvec(:)]);
    mydata('backup', sprintf('Line%d',i), Xc);
    mydata('backup', sprintf('line%d',i), proj);

    % Compute residuals -> lines
    D = pdist2(proj', imgLines{i}');
    [~, minIdx] = min(D, [], 2);

    % Compute residuals (vector form)
    closestPts = imgLines{i}(:, minIdx);
    diff = proj - closestPts;
    r = diff(:);   % flatten to 2N1×1

    r_all = [r_all; r];
end

% Points
% Project via Scaramuzza model
Xc = (R * worldPts + tvec(:));
%proj = world2cam(Xc, ocam_model);
proj = insta360proj(worldPts, ocam_model, [R tvec(:)]);
mydata('backup', sprintf('Pts',i), Xc);
mydata('backup', sprintf('pts',i), proj);

% Compute residuals -> pairs of points
diff = proj - imgPts;
r = 37.5 * diff(:);
r_all = [r_all; r];

%r_all = norm(r_all);

% Cost tracking
global MyCost
cost = norm(r_all);
if cost <= MyCost
    show_calibr_info(R, tvec, worldPts, worldLines, 'frame7.jpg', cost);
    show_results2( ocam_model, params );
    MyCost = cost;
    mydata('backup','ocam_model',ocam_model);
    mydata('backup','params',params);
    mydata('backup','worldPts', worldPts);
    mydata('backup','worldLines', worldLines);
    mydata('backup','cost',cost);
end

return


function show_calibr_info(R, tvec, X, L, imgfile, cost)
figure(123);       % always update the SAME figure
clf;               % clear previous content

% call your existing scene plotting function
%plot_scene(X, L, R, tvec, imgfile);
plot_scene(X, L, R, tvec);

title(sprintf('Current best cost: %.4f', cost));
drawnow;           % force refresh on screen


% -------------------------------------------------------------------------------------
function [bestParams, rmsError] = optimizePose(X, x, model)

% Flatten input shapes (N points)
worldPts = X';   % Nx3
imgPts   = x';   % Nx2

% Initial guess: no rotation, some translation along Z
init_r = [0;0;0];
init_t = [-1000;-500;-500]; % adjust depending on your scale
x0 = [init_r; init_t];

% Cost function
costFun = @(params) reprojResiduals(params, worldPts, imgPts, model);

% If Optimization Toolbox is available: exist('lsqnonlin','file')
if exist('lsqnonlin','file')
    opts = optimoptions('lsqnonlin','Display','iter','MaxIterations',500);
    [bestParams, resnorm] = lsqnonlin(costFun, x0, [], [], opts);
else
    % Fallback: fminsearch
    fun = @(p) sum(costFun(p).^2);
    opts = optimset('Display','iter','MaxIter',500,'TolX',1e-6,'TolFun',1e-6);
    bestParams = fminsearch(fun, x0, opts);
    resnorm = sum(costFun(bestParams).^2);
end

% Report RMS error
residuals = costFun(bestParams);
rmsError = sqrt(mean(residuals.^2));
fprintf('Final RMS reprojection error: %.3f pixels\n', rmsError);


function r = reprojResiduals(params, worldPts, imgPts, model)

% params = [rvec(3); tvec(3)]
rvec = params(1:3);
tvec = params(4:6);

% Rodrigues rotation
R = rodrigues(rvec);

% Transform world to camera
Xc = (R * worldPts' + tvec(:))';   % Nx3

% Project using your calibrated model
proj = world2cam(Xc', model);      % expects 3xN input

proj = proj';                      % Nx2
diff = proj - imgPts;              % Nx2

% Return residuals as vector
r = diff(:);
