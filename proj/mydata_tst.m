function mydata_tst

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

% [~, X,x, L,l]= mydata_get;
% show_results2( MD.ocam_model, X, x, L, l, MD.params )
% % overlap 2D data

% show 2D and 3D data
figure(403); clf; %hold on
fn= fieldnames(MD);

subplot(121); hold on; axis equal
subplot(122); hold on; axis equal

% plot 3D lines and points
for i=1:length(fn)
    X= MD.(fn{i});
    if     strncmp( fn{i}, 'Line', 4 )
        subplot(121); myplot(X, 'b.-')
    elseif strncmp( fn{i}, 'Pts', 3 )
        subplot(121); myplot(X, 'bx')
    elseif strncmp( fn{i}, 'line', 4 )
        subplot(122); myplot(X, 'b.-')
    elseif strncmp( fn{i}, 'pts', 3 )
        subplot(122); myplot(X, 'bx')
    end
end

subplot(121); box on; grid on
draw_camera(eye(4), struct('scale',1e4)); view(3)

subplot(122); axis ij

return
