function cost= world2camx_cost( params, worldLines, imgLines, worldPts, imgPts )
% Use 2D and 3D lines and points to assess the error in "params"
% Use this function in fminsearch to optimize "params" (note that the other
% arguments of this function are kept untouched)

%   params          18x1                144  double              
%   worldLines       1x5              12520  cell                
%   imgLines         1x5             212808  cell                
%   worldPts         3x8                192  double   
%   imgPts           2x8                128  double              

global MD0
if isempty(MD0)
    MD0.Pts= worldPts;
    MD0.pts= imgPts;
    for i=1:length(worldLines)
        MD0.(sprintf('Line%d',i))= worldLines{i};
        MD0.(sprintf('line%d',i))= imgLines{i};
    end
end

params= world2camx_params( 'paramsExpandIfNeeded', params);
model= world2camx_params( 'params2struct', params );
tvec= model.tvec;
R= rodrigues(model.rvec);

% Lines
% Project via Scaramuzza model
r_all = [];
errAcc= 0;
mydata('reset');
for i = 1:numel(worldLines)

    Xc = (R * worldLines{i} + tvec(:));
    proj = world2camx(worldLines{i}, model, [R tvec(:)]);
    mydata('backup', sprintf('Line%d',i), Xc);
    mydata('backup', sprintf('line%d',i), proj);

    % Compute residuals -> lines
    D = pdist2(proj', imgLines{i}');
    [~, minIdx] = min(D, [], 2);

    % Compute residuals (vector form)
    closestPts = imgLines{i}(:, minIdx);
    diff = proj - closestPts;
    errAcc= errAcc +mean(sqrt(sum(diff.*diff,1)));
    r = diff(:);   % flatten to 2N1×1
    r_all = [r_all; r];
end
num1= numel(worldLines);
errAcc= errAcc/num1; % normalize by the number of lines

% Points
% Project via Scaramuzza model
Xc = (R * worldPts + tvec(:));
%proj = world2cam(Xc, ocam_model);
%proj = insta360proj(worldPts, ocam_model, [R tvec(:)]);
proj = world2camx(worldPts, model, [R tvec(:)]);
mydata('backup', 'Pts', Xc);
mydata('backup', 'pts', proj);

% Compute residuals -> pairs of points
diff = proj - imgPts;
r = 37.5 * diff(:);
r_all = [r_all; r];

num2= size(imgPts,2);
errAcc= (num1*errAcc +num2*mean(sqrt(sum(diff.*diff,1))))/(num1 + num2);

% %r_all = norm(r_all);
% cost = norm(r_all)/length(r_all);
cost= errAcc;

% Cost tracking
global MyCost
if cost <= MyCost
    figure(124); clf; hold on
    plot_MD( MD0, struct('cstrForAll', 'c.') )
    MD= mydata('getall');
    plot_MD( MD, sprintf('cost=%f', cost) )
    drawnow

    MyCost = cost;
    mydata('backup','ocam_model',model);
    mydata('backup','params',params);
    mydata('backup','worldPts', worldPts);
    mydata('backup','worldLines', worldLines);
    mydata('backup','cost',cost);
end

return
