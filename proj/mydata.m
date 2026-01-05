function ret= mydata(op, a1, a2, a3)
global MD

ret= [];
switch op
    case 'reset', MD= [];

    case {'set', 'backup'}, MD.(a1)=a2;
    case 'get', ret= MD.(a1);
    case 'getall', ret= MD;

    case 'save'
        % usage: mydata save
        ofname= mkfname( 'mydata_', 'mat', struct('outputFormat',3) );
        MD.ofname= ofname;
        save(ofname, 'MD');
    case 'load'
        % usage: ret= mydata('load');
        fname= filenames_last_only( './mydata_*.mat' );
        ret= load(fname);

    otherwise
        error('inv op: %s', op)
end
