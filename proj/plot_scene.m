function plot_scene(X, L, R, tvec, imgfile)
if nargin<4
    tvec= zeros(3,1);
end
if nargin<3
    R= eye(3);
end

figure(123); clf; hold on;
view(3); axis equal; grid on; box on;
xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]');

% --- Optional image plane ---
if nargin >= 5 && ~isempty(imgfile)
    img = imread(imgfile);

    [Ximg, Zimg] = meshgrid(linspace(0,1910,size(img,2)), ...
        linspace(0,1200,size(img,1)));
    Yimg = zeros(size(Ximg));  % y = 0 plane

    surf(Ximg, Yimg, Zimg, flipud(img), ...
        'FaceColor', 'texturemap', 'EdgeColor', 'none');
end

% --- Camera pose ---
P= get_cam_from_args(R, tvec);
draw_camera(P, struct('scale', 5000));

% --- Plot lines ---
for i = 1:numel(L)
    plot3(L{i}(1,:), L{i}(2,:), L{i}(3,:), 'r', 'LineWidth', 2);
end

% --- Plot points ---
plot3(X(1,:), X(2,:), X(3,:), 'r.', 'MarkerSize', 15);

% axis([-90 2000 0 2000 -600 1300]);
axis equal
axis tight
view(-126, 11.4);

return % end of main function


function P= get_cam_from_args(R, tvec)
% R is a matrix
if min(size(R)) >= 3
    P = [R, -(R * tvec(:))];
    if size(R,2)==4
        P= R; % all info comes in R
    end
    return
end

% R comes as a struct
if isstruct(R)
    x= R;
    rvec = x.init_r;
    tvec = x.init_t;
    R = rodrigues(rvec);
    P = [R tvec];
    return
end

% R is the bestParams vector
bestParams= R;
rvec = bestParams(1:3);
tvec = bestParams(4:6);
R = rodrigues(rvec);
P = [R tvec];
