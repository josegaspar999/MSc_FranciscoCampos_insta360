function plot_MD( MD, opt0 )
if nargin<2
    opt0=[];
end
if ischar(opt0)
    opt0= struct('tstr', opt0);
end

subplot(121); hold on; axis equal
subplot(122); hold on; axis equal

% plot 3D lines and points
cnf= get_plot_config( opt0 );
fn= fieldnames(MD);
for i=1:length(fn)
    fni = fn{i};
    opt= options_change( opt0, cnf, fni );
    if isfield(opt, 'splot')
        % finally the plot
        subplot(opt.splot); myplot(MD.(fni), opt);
    end
end

% add camera
subplot(121); box on; grid on
draw_camera(eye(4), struct('scale',1e4)); view(3)

% add title
subplot(122); draw_square; axis ij; axis tight
if isfield(opt, 'tstr')
    title(opt.tstr)
end

return


function opt= options_change(opt, cnf, fni)
% if     strncmp( fni, 'Line', 4 )
%     subplot(121); myplot(X, struct('cstr','b.-','str',fni));
% elseif strncmp( fni, 'Pts', 3 )
%     subplot(121); myplot(X, struct('cstr','bx','cntFlag',1))
% elseif strncmp( fni, 'line', 4 )
%     subplot(122); myplot(X, struct('cstr','b.-','str',fni))
% elseif strncmp( fni, 'pts', 3 )
%     subplot(122); myplot(X, struct('cstr','bx','cntFlag',1))
% end

if length(fni)<3
    return % do nothing
end

cmd= fni(1:3); indFound=0;
for i=1:size(cnf,1)
    if strcmp(cmd, cnf{i,1})
        indFound= i; break
    end
end
if indFound==0
    return
end

% finaly set options
opt.splot= cnf{indFound,2};
opt.cstr= cnf{indFound,3};
switch cnf{indFound,4}
    case 1, opt.str= fni;
    case 2, opt.cntFlag=1;
    otherwise, error('cnf 4 not in {1,2}')
end




function cnf= get_plot_config( opt )
cnf= {
    'Lin',121,'b.-',1;
    'Pts',121,'bx', 2;
    'lin',122,'b.-',1;
    'pts',122,'bx', 2;
    };
if isfield(opt, 'cstrList')
    for i=1:size(cnf,1), cnf{i,3}= opt.cstr{1,1}{i}; end
end
if isfield(opt, 'cstrForAll')
    for i=1:size(cnf,1), cnf{i,3}= opt.cstrForAll; end
end


function draw_square
W= 2880; m= [0 0; 0 W; W W; W 0; 0 0]';
plot(m(1,:),m(2,:));

