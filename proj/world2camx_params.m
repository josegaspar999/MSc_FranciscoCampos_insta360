function ret= world2camx_params( op, a1, a2 )
%
% Manage projection parameters as an array (vector) to help using
% optimization functions as fminsearch

% Typical usages:
% model  = world2camx_params( 'params2struct', params );
% params = world2camx_params( 'struct2params', model );
% params = world2camx_params( 'paramsSelectEntries', params );

if nargin<1
    demo
    return
end

switch op
    case 'struct2params'
        ret= struct2params(a1);
    case 'params2struct'
        ret= params2struct(a1);

    case 'paramsSelectEntries'
        ret= params_select(1, a1, [1:6 12:16]); % rvec, tvec, ss
    case 'paramsSelectEntries2'
        ret= params_select(1, a1, a2);
    case 'paramsAllEntries'
        ret= params_select(2, a1); % expand params
    case 'paramsExpandIfNeeded'
        ret= paramsExpandIfNeeded(a1);

    case {'getRotTrans', 'struct2RotTrans', 'params2RotTrans'}
        ret= getRotTrans(a1);

    otherwise
        error('inv op');
end


function params= struct2params(model)
if isfield(model,'rvec')
    % early versions of model have no extrinsics
    r_t_vec= [model.rvec(:); model.tvec(:)];
else
    r_t_vec= zeros(6,1);
end
params= [
    r_t_vec; ...
    model.c; model.d; model.e; ...
    model.xc; model.yc; ...
    model.ss(:); ...
    model.width; model.height; ...
    ];


function params= paramsExpandIfNeeded(params)
% params_select.m has a global var COEF
global COEF
if isempty(COEF) || length(params)==length(COEF.P)
    return % do nothing
end
if length(params)~=length(COEF.ind)
    warning('params with wrong length')
end
params= params_select(2, params);
return


function model= params2struct(params)
% Unpack intrinsics to fine-tune
% idx = 6;
% model.c  = params(idx+1); idx = idx + 1;
% model.d  = params(idx+1); idx = idx + 1;
% model.e  = params(idx+1); idx = idx + 1;
% model.xc = params(idx+1); idx = idx + 1;
% model.yc = params(idx+1); idx = idx + 1;
% 
% % Update polynomial coefficients (only first n_ss_opt)
% model.ss(1:size(params,1)-idx) = params(idx+1:end);
% model.ss= model.ss(:);

% model.rvec = params(1:3);
% model.tvec = params(4:6);
% model.c  = params(7);
% model.d  = params(8);
% model.e  = params(9);
% model.xc = params(10);
% model.yc = params(11);
% model.ss = params(12:16);
% model.ss = model.ss(:);
% model.width  = params(17);
% model.height = params(18);

model.ss = params(12:16);
model.ss = model.ss(:);
model.xc = params(10);
model.yc = params(11);
model.c  = params(7);
model.d  = params(8);
model.e  = params(9);
model.width  = params(17);
model.height = params(18);
model.rvec = params(1:3);
model.tvec = params(4:6);


function Rt= getRotTrans(a1)
if isstruct(a1)
    model= a1;
    Rt = [rodrigues(model.rvec) model.tvec(:)];
else
    params= a1;
    rvec = params(1:3);
    tvec = params(4:6);
    Rt = [rodrigues(rvec) tvec(:)];
end


function demo
MD= mydata('load'); MD= MD.MD;
m0= MD.ocam_model; m0.rvec= MD.params(1:3); m0.tvec= MD.params(4:6);
p0= world2camx_params('struct2params', m0);
m1= world2camx_params('params2struct', p0);
p1= world2camx_params('struct2params', m1);
err= max(abs(p1-p0));
fprintf(1, 'Estruturas iguais: %d \t (erro entre arrays= %f)\n', isequal(m0,m1), err);
return
