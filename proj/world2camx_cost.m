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
errAcc= 0;

% Lines, project via Scaramuzza model
worldLines2 = worldLines;
imgLines2 = imgLines;
for i = 1:numel(worldLines)

    imgLines2{i} = world2camx(worldLines{i}, model, [R tvec(:)]);
    worldLines2{i} = (R * worldLines{i} + tvec(:));

    % Compute residuals -> lines
    D = pdist2(imgLines2{i}', imgLines{i}');
    [~, minIdx] = min(D, [], 2);

    % Compute residuals (vector form)
    closestPts = imgLines{i}(:, minIdx);
    err = imgLines2{i} - closestPts;
    errAcc= errAcc +mean(sqrt(sum(err.*err,1)));
end
num1= numel(worldLines);
errAcc= errAcc/num1; % normalize by the number of lines

% Points, project via Scaramuzza model & compute residuals
worldPts2 = (R * worldPts + tvec(:));
imgPts2 = world2camx(worldPts, model, [R tvec(:)]);
err2 = imgPts2 - imgPts;
err2= mean( sqrt(sum(err2.*err2,1)) );
num2= size(imgPts,2);

errAcc= (num1*errAcc +num2*err2)/(num1 + num2);
cost= errAcc;

% Cost tracking
global MyCost
if cost <= MyCost
    figure(124); clf; hold on
    plot_MD( MD0, struct('cstrForAll', 'c.') )
    MD= vars_to_struct( worldLines2, imgLines2, worldPts2, imgPts2 );
    plot_MD( MD, sprintf('cost=%f', cost) )
    drawnow

    MyCost = cost;
    %     mydata('backup','ocam_model',model);
    %     mydata('backup','params',params);
    %     mydata('backup','worldPts', worldPts);
    %     mydata('backup','worldLines', worldLines);
    %     mydata('backup','cost',cost);
end

return


function MD0= vars_to_struct( worldLines, imgLines, worldPts, imgPts )
MD0.Pts= worldPts;
MD0.pts= imgPts;
for i=1:length(worldLines)
    MD0.(sprintf('Line%d',i))= worldLines{i};
    MD0.(sprintf('line%d',i))= imgLines{i};
end
