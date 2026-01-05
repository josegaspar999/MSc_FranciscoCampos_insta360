function show_results2( model, X, x, L, l, bestParams )
% Display of data on the image
% bestParams set the intrinsic and extrinsic parameters of the camera

if nargin==2
    % call as: show_results2( ocam_model, params );
    bestParams= X;
    [~, X, x, L, l]= mydata_get;
    show_results2( model, X, x, L, l, bestParams )
    return
end

rvec = bestParams(1:3);
tvec = bestParams(4:6);
R = rodrigues(rvec);
model.c  = bestParams(7);
model.d  = bestParams(8);
model.e  = bestParams(9);
model.xc = bestParams(10);
model.yc = bestParams(11);
model.ss(1:size(bestParams,1)-11) = bestParams(12:end);

% Load the image
I = imread('frame7.jpg');
figure(202); clf; imshow(I); hold on;

% Plot lines
for i=1:length(L)
    L1= L{i}; l1= l{i};
    L1_c = R * L1 + tvec(:); l1_pred = world2cam(L1_c, model);
    plot(l1(1,:), l1(2,:), 'r.', 'MarkerSize', 2); plot(l1_pred(1,:), l1_pred(2,:), 'b-', 'LineWidth', 2);
end

% Plot measured points (ground truth, red)
% Plot predicted points (blue crosses)
% Optionally connect corresponding points to visualize errors
plot(x(1,:), x(2,:), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
X_c = R * X + tvec(:);
x_pred = world2cam(X_c, model);
plot(x_pred(1,:), x_pred(2,:), 'bx', 'MarkerSize', 8, 'LineWidth', 2);
for i = 1:min(size(x,2),size(x_pred,2)) %8
    line([x(1,i) x_pred(1,i)], [x(2,i) x_pred(2,i)], ...
        'Color','y','LineStyle','--','LineWidth',1.5);
end
%legend({'Real','Predicted'}, 'TextColor','b');
legend({'Measured','Predicted','Error'}, 'TextColor','b');

axis tight
