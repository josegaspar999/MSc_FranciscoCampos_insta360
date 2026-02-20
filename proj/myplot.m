function myplot(X, opt)
% 2D or 3D plot from array
% X : 2xN or 3xN

if ischar(opt)
    cstr= opt;
    opt= [];
else
    cstr= opt.cstr;
end

% plot points or lines, 2D or 3D
if size(X,1)==2
    plot(X(1,:),X(2,:),cstr)
elseif size(X,1)==3
    plot3(X(1,:),X(2,:),X(3,:),cstr)
else
    error('size(X,1) not 2 nor 3');
end

% tag the lines with given strings
if isfield(opt, 'str')
    m= mean(X,2);
    if size(X,1)==2
        text(m(1),m(2),opt.str)
    elseif size(X,1)==3
        text(m(1),m(2),m(3),opt.str)
    end
end

% tag the points with a counting
if isfield(opt, 'cntFlag') && opt.cntFlag
    for i=1:size(X,2)
        t= num2str(i);
        if size(X,1)==2
            text(X(1,i),X(2,i),t)
        elseif size(X,1)==3
            text(X(1,i),X(2,i),X(3,i),t)
        end
    end
end
