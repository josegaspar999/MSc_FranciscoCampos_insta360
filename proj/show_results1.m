function [R, tvec]= show_results1( model, X, x, bestParams )
rvec = bestParams(1:3);
tvec = bestParams(4:6);
R = rodrigues(rvec);

% Load the image
I = imread('frame7.jpg');

% x (2xN) = measured 2D points
% x_pred (2xN) = projected points
N = size(x,2);

figure(200); clf
imshow(I); hold on;

% Plot measured points (ground truth, red)
% Plot predicted points (blue crosses)
% Optionally connect corresponding points to visualize errors
plot(x(1,:), x(2,:), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
X_c = R * X + tvec(:);
x_pred = world2cam(X_c, model);
plot(x_pred(1,:), x_pred(2,:), 'bx', 'MarkerSize', 8, 'LineWidth', 2);
for i = 1:N
    line([x(1,i) x_pred(1,i)], [x(2,i) x_pred(2,i)], ...
        'Color','y','LineStyle','--','LineWidth',1.5);
end
legend({'Measured','Predicted','Error'}, 'TextColor','b');
