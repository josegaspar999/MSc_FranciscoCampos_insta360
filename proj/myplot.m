function myplot(X, cstr)
% 2D or 3D plot from array
% X : 2xN or 3xN

if size(X,1)==2
    plot(X(1,:),X(2,:),cstr)
elseif size(X,1)==3
    plot3(X(1,:),X(2,:),X(3,:),cstr)
else
    error('size(X,1) not 2 nor 3');
end
