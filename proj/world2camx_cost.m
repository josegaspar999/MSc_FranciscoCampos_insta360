function cost= world2camx_cost( params, worldLines, imgLines, worldPts, imgPts )
% Use 2D and 3D lines and points to assess the error in "params"
% Use this function in fminsearch to optimize "params" (note that the other
% arguments of this function are kept untouched)

%   params          18x1                144  double              
%   worldLines       1x5              12520  cell                
%   imgLines         1x5             212808  cell                
%   worldPts         3x8                192  double   
%   imgPts           2x8                128  double              

if nargin<1
    mydata_tst(3);
    return
end

global MD0
if isempty(MD0)
    MD0= vars_to_struct( worldLines, imgLines, worldPts, imgPts );
end

params= world2camx_params( 'paramsExpandIfNeeded', params);
model= world2camx_params( 'params2struct', params );
tvec= model.tvec;
R= rodrigues(model.rvec);
err1= 0;

% Lines, project via Scaramuzza model
worldLines2 = worldLines;
imgLines2 = imgLines;
for i = 1:numel(worldLines)
    [imgLines2{i}, worldLines2{i}] = world2camx(worldLines{i}, model, [R tvec(:)]);
    err= line_error( imgLines{i}',  imgLines2{i}', i==1 );
    err1= err1 +err;
end
num1= numel(worldLines);
err1= err1/num1; % normalize by the number of lines

% Points, project via Scaramuzza model & compute residuals
worldPts2 = (R * worldPts + tvec(:));
imgPts2 = world2camx(worldPts, model, [R tvec(:)]);
err2 = imgPts2 - imgPts;
% err2= mean( sqrt(sum(err2.*err2,1)) );
err2= err_calc(err2');
num2= size(imgPts,2);

cost= (num1*err1 +num2*err2)/(num1 + num2);

% Cost tracking
global MyCost
if cost <= MyCost
    figure(124); clf; hold on
    plot_MD( MD0, struct('cstrForAll', 'c.') )
    MD= vars_to_struct( worldLines2, imgLines2, worldPts2, imgPts2 );
    %ts= sprintf('cost=%f', cost);
    ts= sprintf('err1=%f (%d)%cerr2=%f (%d)', err1,num1, 10, err2,num2);
    plot_MD( MD, ts )
    drawnow

    MyCost = cost;
    %     mydata('backup','ocam_model',model);
    %     mydata('backup','params',params);
    %     mydata('backup','worldPts', worldPts);
    %     mydata('backup','worldLines', worldLines);
    %     mydata('backup','cost',cost);
end
return


% ---------------------------------------------------------------
function errValue= err_calc(errVectors)
% usage: err= err_calc(err);
if size(errVectors,2)~=2
    error('errVectors must be Nx2')
end
% errValue= mean(sqrt(sum(errVectors.^2,2)));
errValue= sqrt(mean(sum(errVectors.^2,2))); % RMS error


function err= line_error_v0( line1, line2, isFirstLine )
%   line1      3273x2             52368  double              
%   line2       100x2              1600  double  
D = pdist2(line2, line1);
[~, minIdx] = min(D, [], 2);
closestPts = line1(minIdx, :);
err= line2 - closestPts;
err= err_calc(err);


function err= line_error_v1( line1, line2, isFirstLine )
%   line1      3273x2             52368  double              
%   line2       100x2              1600  double  

% error vectors
D = pdist2(line2, line1);
[~, minIdx] = min(D, [], 2);
closestPts = line1(minIdx, :);
err= closestPts - line2;

% make local normal vectors
% (loose first and last points)
n= line2(3:end,:)-line2(1:end-2,:);
n= [-n(:,2) n(:,1)];
n= n./repmat(sqrt(sum(n.*n,2)),1,2);

% calc orthogonal distances from line2
err= err(2:end-1,:);
err= diag( err*n' );

if 0 %1
    % debug, show error vectors and normals
    figure(125);
    if isFirstLine
        clf; hold on
    end
    x= [closestPts(:,1) line2(:,1) closestPts(:,2) line2(:,2)];
    plot( x(:,1:2)', x(:,3:4)', 'c.-' )
    y= line2(2:end-1,:);
    y= [y y+diag(err)*n];
    y= y(:,[1 3 2 4]);
    plot( y(:,1:2)', y(:,3:4)', 'b.-' )
    axis image; axis ij
end

% return a scalar value, mean of errors
err= mean( abs(err) );

return % end of function


function err= line_error( line1, line2, isFirstLine )
%   line1      3273x2             52368  double              
%   line2       100x2              1600  double  
D = pdist2(line2, line1);
[~, minIdx] = min(D, [], 2);
closestPts = line1(minIdx, :);
err= closestPts - line2;

% % at the end of some too long 3D model lines the real points stay behind;
% % reduce importance of those points whose matching go in the wrong direction
% tgv= diff(line2);
% flg= diag(err(2:end,:)*tgv');

% enforce length of line2 similar to closestPts
ind = tooLongPath(closestPts, line2);
if ~isempty(ind)
    err(ind:end, :) = repmat(err(ind-1, :), size(err,1) - ind + 1, 1);
end
err= err_calc(err);


function ind = tooLongPath(xy1, xy2)
%TOOLONGPATH  Find first index where path xy2 becomes longer than xy1
%
%   ind = tooLongPath(xy1, xy2)
%
%   xy1, xy2 : Nx2 matrices with (x,y) points
%   ind      : first index where cumulative length of xy2 exceeds xy1
%              empty if it never happens

% Ensure same length
n = min(size(xy1,1), size(xy2,1));

% Differences between consecutive points
d1 = diff(xy1(1:n,:), 1, 1);
d2 = diff(xy2(1:n,:), 1, 1);

% Incremental segment lengths
l1 = sqrt(sum(d1.^2, 2));
l2 = sqrt(sum(d2.^2, 2));

% Cumulative lengths (prepend zero for index alignment)
L1 = [0; cumsum(l1)];
L2 = [0; cumsum(l2)];

% First index where xy2 is longer than xy1
ind = find(L2 > L1(end), 1, 'first');


function MD0= vars_to_struct( worldLines, imgLines, worldPts, imgPts )
MD0.Pts= worldPts;
MD0.pts= imgPts;
for i=1:length(worldLines)
    MD0.(sprintf('Line%d',i))= worldLines{i};
    MD0.(sprintf('line%d',i))= imgLines{i};
end
