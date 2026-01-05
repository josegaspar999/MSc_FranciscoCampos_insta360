function plotx(op, varargin)

switch op
    %     case 'results1'
    %     case 'results2'

    case 'scene', plot_scene(varargin)
        % plot_scene(X, L, R, tvec, imgfile)

    otherwise
        error('inv op')
end
