function vec8_tst( tstId )
% usages: vec8_tst(1); vec8_tst(2);
if nargin<1
    tstId= 1;
end
switch tstId
    case 1, vec8_tst_main( 'cmp1234' );
    case 2, vec8_tst_main( 'cmp2143' );
    otherwise, error('inv tstId')
end


function vec8_tst_main( dataId )
% get trajectories data as in "data_plot.m"
% [x, xname]= data_load('cmp1234');
% [x, xname]= data_load('cmp2143');

fprintf('-- dataId=%s\n', dataId);
[x, xname]= data_load(dataId);

% x = { [7x1487 double]    [7x1487 double]    [7x1487 double]    [7x1487 double] }
% xname = { 'CFisheye'    'CPPinhole'    'MFisheye'    'MPinhole' }
% data_plot( 'no_transf', x, xname );
% data_plot( 'procrustes', x, xname );

% split each trajectory into 10 trajectory parts
% (all trajectories in data_plot.m demo have the same length)
[y, yname]= split_and_match(x,xname, 10);

% for each trajectory part, compute procrustes and from it get a vec8
% for a pair of trajectories, get their start and end poses, and get two vec8
[z,zname]= transf_apply( x,xname, y,yname );

% % interpolate all vec8 from start to end, redefine trajectories, show with
data_plot( 'no_transf', z, zname );
figure(203)
switch dataId
    case 'cmp1234', view( -168.4714, 10.3429 )
    case 'cmp2143', view( 0, 90 )
end

return % end of main function


% --------------------------------------------------------------------
function [z,zname]= transf_apply( x,xname, y,yname )
% final point clouds, well close to the reference
%   x            1x4             333536  cell               
%   y           10x4             337568  cell  

% x = 
%     [7x1487 double]    [7x1487 double]    [7x1487 double]    [7x1487 double]

z= x; zname= xname;

% find interpolations foreach experiment, use "transf" and "i1x2"
for i=2:length(z)
    xi= y(:,1);
    yi= y(:,i);
    zi= [];
    for j=1:length(yi)
        xij= xi{j}(1:3,:);
        yij= yi{j}(1:3,:);
        [~,zij,~]= procrustes(xij', yij');
        zi= [zi zij'];
    end
    z{i}= zi;
end

return


% --------------------------------------------------------------------
function [z,zname]= transf_apply_v2( x,xname, y,yname )

%       ** WORK IN PROGRESS, YET TO COMPLETE **

% final point clouds, well close to the reference
%   x            1x4             333536  cell               
%   y           10x4             337568  cell  

% x = 
%     [7x1487 double]    [7x1487 double]    [7x1487 double]    [7x1487 double]

transf= transf_calc( y );
%   transf      12x4               6432  cell               

z= x; zname= xname;
i1x2= indexes_parts_for_transf( y(:,1) );

% compute a global x alignment, and give it to all z entries

% find interpolations foreach experiment, use "transf" and "i1x2"
for i=1:length(z)
    z{i}= transf_apply_impl( x{i}, transf(:,i), i1x2 );
end

return


function zi= transf_apply_impl( xi, T, i1x2 )
%   xi         7x1487            83272  double 
%   T         12x1                2112  cell                
%   i1x2      12x3                 288  double              

% T(1) :    [8x1 double]

% i1x2 =
%            1           1           1
%            2          76         149
%          150         224         298
%          299         373         447
%          448         522         596
%          597         671         745
%          746         820         894
%          895         969        1043
%         1044        1118        1191
%         1192        1266        1339
%         1340        1413        1486
%         1487        1487        1487

zi= xi;

for i=2:length(T)
    % vecOut = vec8_interpolate(vecStart, vecEnd, t)
    i1= i1x2(i-1,2); i2= i1x2(i,2);

    %vt= vec8_interpolate( cell2mat(T(i-1)), cell2mat(T(i)), ind2t(i1, i2) );
    %xt= vec8_operation( 'a*b', vt, hset(xi(:,i1:i2)) );

    xt= vec8_operation( 'a*b', repmat(cell2mat(T(i)),1,i2-i1+1), hset(xi(:,i1:i2)) );
    zi(:,i1:i2)= vec8_operation( 'vec8_to_vec7', xt);
end

return


function t= ind2t(i1, i2)
% 0= a*i1+b, 1= a*i2+b
% 1= a*i2 -a*i1 -> a= 1/(i2-i1) -> b= -i1/(i2-i1)
t= ((i1:i2)-i1)/(i2-i1);


function i1x2= indexes_parts_for_transf( yc )
for i=1:size(yc,1)
    n= size(yc{i}, 2);
    if i==1
        i1x2= [1 n];
    else
        m= i1x2(i-1,2);
        i1x2(i,:)= [m+1 m+n];
    end
end
i1x2= [1 1; i1x2; repmat(i1x2(end,2), 1,2)];
i1x2(2,1)= 2;
i1x2(end-1,2)= i1x2(end-1,2)-1;
i1x2= [i1x2(:,1) round(sum(i1x2,2)/2) i1x2(:,2)];
return


function vec8= vec8_identity
% Identity vec8 transformation (Does nothing)
vec8 = [0; 0; 0; 1; 0; 0; 0; 1];


% --------------------------------------------------------------------
function transf= transf_calc( y )
% y : cell Nparts x 4, e.g. 10 x 4
% transf : cell (Nparts+2) x 4

% y = 
%     [7x149 double]    [7x149 double]    [7x149 double]    [7x149 double]
%     [7x149 double]    [7x149 double]    [7x149 double]    [7x149 double]
%     [7x149 double]    [7x149 double]    [7x149 double]    [7x149 double]
%     [7x149 double]    [7x149 double]    [7x149 double]    [7x149 double]
%     [7x149 double]    [7x149 double]    [7x149 double]    [7x149 double]
%     [7x149 double]    [7x149 double]    [7x149 double]    [7x149 double]
%     [7x149 double]    [7x149 double]    [7x149 double]    [7x149 double]
%     [7x148 double]    [7x148 double]    [7x148 double]    [7x148 double]
%     [7x148 double]    [7x148 double]    [7x148 double]    [7x148 double]
%     [7x148 double]    [7x148 double]    [7x148 double]    [7x148 double]

% transf = 
%     []    [8x1 double]    [8x1 double]    [8x1 double]
%     []    [8x1 double]    [8x1 double]    [8x1 double]
%     []    [8x1 double]    [8x1 double]    [8x1 double]
%     []    [8x1 double]    [8x1 double]    [8x1 double]
%     []    [8x1 double]    [8x1 double]    [8x1 double]
%     []    [8x1 double]    [8x1 double]    [8x1 double]
%     []    [8x1 double]    [8x1 double]    [8x1 double]
%     []    [8x1 double]    [8x1 double]    [8x1 double]
%     []    [8x1 double]    [8x1 double]    [8x1 double]
%     []    [8x1 double]    [8x1 double]    [8x1 double]
%     []    [8x1 double]    [8x1 double]    [8x1 double]
%     []    [8x1 double]    [8x1 double]    [8x1 double]

% vec8= vec8_identity;
% vec8Inv = vec8_invert(vec8);
% vec8'=         0     0     0     1     0     0     0     1
% vec8Inv'=      0     0     0     1     0     0     0     1
% vec8 : 3 vals position, 4 vals quaternion, 1 val scale

% first and last lines, pose to pose computation
% middle lines, apply procrustes (3D or 7D?)

transf= {};

% transf first column is just an identity
N= size(y,1); M= size(y,2);
for j=1:N+2
    transf{j,1}= vec8_identity;
end

for i=2:M % go column by column

    % make similar the poses of sub-sections of the trajectory
    for j=1:N % go row by row, compare column i with column 1 (one)
        a= y{j,1}; b= y{j,i}; 
        transf{j+1,i}= transf_calc_with_orient( a, b );
    end
    
    % make equal de first pose
    a= y{1,1}(:,1);
    b= y{1,i}(:,1); bscale= transf{2,i}(8);
    transf{1,i}= vec8_relative_transf( [a;1], [b; bscale] );

    % make equal de last pose
    a= y{end,1}(:,end);
    b= y{end,i}(:,end); bscale= transf{N+1,i}(8);
    transf{size(y,1)+2,i}= vec8_relative_transf( [a;1], [b;bscale] );

end

return


function vec8= transf_calc_with_orient( a, b )
%   a           7x149             8344  double              
%   b           7x149             8344  double   
%   A         149x3               3576  double              
%   B         149x3               3576  double              
if 0
    A=a(1:3,:)'; % ** should use 7 coords **
    B=b(1:3,:)';
else
    A= vec8_pose_to_axes_points( hset(a), 0.01*distOneToEnd(a(1:3,:)) );
    B= vec8_pose_to_axes_points( hset(b), 0.01*distOneToEnd(b(1:3,:)) );
end

vec8= vec8_from_procrustes(A,B);

if 0 % debug
    vec8_operation('z-x*y', vec8, hset(b(:,1)), hset(a(:,1)))
end
return


function d= distOneToEnd(a)
% a : 3xN
d= a(1:3,end)-a(1:3,1);
d= sqrt(sum(d.*d));


% --------------------------------------------------------------------
function [y, yname]= split_and_match(x, xname, Nparts)
y= {};
for i=1:size(x,2)
    yy= splitArrayIntoNParts(x{i}, Nparts);
    for j=1:length(yy)
        y{j,i}= yy{j};
    end
end

yname= xname;
for i=2:size(y,1)
    yname(i,:)= yname(1,:);
end

return


function y = splitArrayIntoNParts(x, N)
% SPLITARRAYINTONPARTS Splits a 7xM matrix into N column-wise parts.
% Inputs:
%   x - 7xM matrix to be split
%   N - Number of parts to split into
% Output:
%   y - 1xN cell array where each cell contains a 7xMi sub-matrix

[rows, M] = size(x);

% Validate inputs
if N > M
    error('Number of parts (N) cannot be greater than the number of columns (M).');
end

% Initialize the cell array to hold the split parts
y = cell(1, N);

% Base number of columns per part
baseCols = floor(M / N);

% Remainder columns to distribute if M is not perfectly divisible by N
remainder = mod(M, N);

% Loop to extract each part
startCol = 1;
for i = 1:N
    % Distribute the remainder columns across the first few parts
    if i <= remainder
        currentCols = baseCols + 1;
    else
        currentCols = baseCols;
    end
    
    endCol = startCol + currentCols - 1;
    
    % Extract the sub-matrix yi = x(:, A:B)
    y{i} = x(:, startCol:endCol);
    
    % Move the starting index to the next segment
    startCol = endCol + 1;
end
