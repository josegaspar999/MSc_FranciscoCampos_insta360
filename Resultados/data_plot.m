function data_plot( op, xx, leg, options )

%   op        1x10                20  char               
%   xx        1x4             333536  cell               
%   leg       1x4                514  cell               

% op = 'procrustes'
% leg = 'CFisheye'    'CPPinhole'    'MFisheye'    'MPinhole'

% demo usages:
% data_plot demo1
% data_plot demo2

if nargin<1
    %colmap12_plot
    demo('cmp1234')
    return
end
if nargin<4
    options= [];
end

switch op
    case 'no_transf' % do nothing
    case 'procrustes', options.transf= 'procrustes';

    case 'demo1', demo('cmp1234'); return
    case 'demo2', demo('cmp2143'); return

    otherwise, error('inv op %s', op);
end

data_plot_main( xx, leg, options )


function data_plot_main( xx, leg, options )
figure(201); clf; hold on
figure(202); clf; hold on
figure(203); clf; hold on
x= xx{1};
figure(201); myplot2_err(x,x)
figure(202); myplot2(x)
figure(203); myplot3(x)
for i=2:length(xx)
    y= xx{i};
    if ~isfield(options, 'transf') || isempty(options.transf)
        z= y;
    elseif isfield(options, 'transf') && strcmp(options.transf, 'procrustes')
        [~,z,~]= myprocrustes(x,y);
    else
        error('options.transf issue');
    end
    figure(201); myplot2_err(x,z)
    figure(202); myplot2(z)
    figure(203); myplot3(z)
end
figure(201); legend(leg)
figure(202); legend(leg)
figure(203); legend(leg); %axis tight

return


function [a,z,b]= myprocrustes(x,y)
if ~checkSize(x) || ~checkSize(y)
    error('expecting x and y 7xN or 3xN');
end
[a,z,b]= procrustes(x(1:3,:)', y(1:3,:)');
z= z';
return


function flag= checkSize(x)
flag=0;
if size(x,1)==3 || size(x,1)==7
    flag=1;
end


function myplot2_err(x, z)
e= z(1:3,:)-x(1:3,:);
E= sqrt(sum(e.*e, 1));
subplot(411); hold on; plot(e(1,:),'.-'); xlabel('x_{err} traj');
subplot(412); hold on; plot(e(2,:),'.-'); xlabel('y_{err} traj');
subplot(413); hold on; plot(e(3,:),'.-'); xlabel('z_{err} traj');
subplot(414); hold on; plot(E,'.-'); xlabel('err norm traj');
fprintf('\tmeanErr=%f\n', mean(E));
fprintf('\t|last-first|=%f\n', norm( x(1:3,end) - x(1:3,1) ) );


function myplot2(x)
subplot(311); hold on; plot(x(1,:),'.-'); xlabel('x traj');
subplot(312); hold on; plot(x(2,:),'.-'); xlabel('y traj');
subplot(313); hold on; plot(x(3,:),'.-'); xlabel('z traj');


function myplot3(x)
h1= plot3(x(1,:),x(2,:),x(3,:),'.-');
currentColor = h1.Color;
ax = gca; currentIndex = ax.ColorOrderIndex;
h2= plot3(x(1,1),x(2,1),x(3,1),'o', 'Color', currentColor);
h2.Annotation.LegendInformation.IconDisplayStyle = 'off';
h2= plot3(x(1,end),x(2,end),x(3,end),'*', 'Color', currentColor);
h2.Annotation.LegendInformation.IconDisplayStyle = 'off';
ax.ColorOrderIndex = currentIndex;
box on; grid on; axis equal
view(3)


function demo( dataId )
% dataId : 'cmp1234' or 'cmp2143'
fprintf('-- dataId=%s\n', dataId);
[xx, leg]= data_load(dataId);
data_plot( 'procrustes', xx, leg )
