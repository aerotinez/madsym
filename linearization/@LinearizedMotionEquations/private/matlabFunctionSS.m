function matlabFunctionSS(obj,name,parameters)
    arguments
        obj (1,1) LinearizedMotionEquations;
        name (1,1) string;
        parameters (:,1) = sym.empty(0,1);
    end
    
    eom = obj;
    x = prettify(eom.States.state);
    u = prettify(eom.Inputs.state);
    M = prettify(eom.MassMatrix);
    H = prettify(eom.ForcingMatrix);
    G = prettify(eom.InputMatrix);

    if isempty(parameters)
        parameters = setdiff(symvar([M,H,G]),x).';
    end

    dir_name = name + "StateSpace";
    if ~exist(dir_name, 'dir')
        mkdir(dir_name)
    end

    Mstr = dir_name + "/" + name + "SSMassMatrix";
    Hstr = dir_name + "/" + name + "SSForcingMatrix";
    Gstr = dir_name + "/" + name + "SSInputMatrix";

    state_str = "   states = [" + strjoin(string(x).') + "]";
    input_str = "   inputs = [" + strjoin(string(u).') + "]";
    param_str = "   params = [" + strjoin(string(parameters).') + "]";
    comment_str = [state_str,input_str,param_str,""].';

    vars = {parameters};
    matlabFunction(M,"File",Mstr,"Vars",vars,"Comments",comment_str);
    matlabFunction(H,"File",Hstr,"Vars",vars,"Comments",comment_str);
    matlabFunction(G,"File",Gstr,"Vars",vars,"Comments",comment_str);

    f = @(x)arrayfun(@(y)strrep(string(y.state),"(t)",""),x);
    state_names = f(eom.States);
    input_names = f(eom.Inputs);

    np = numel(parameters);
    nz = numel(char(num2str(np)));
    palias = arrayfun(@(k)sym("p_" + sprintf("%0*d",nz,k)),(1:np).');
    pnames = arrayfun(@(k)"p(" + num2str(k) + ",:)",(1:np).');

    fdx = @(x)arrayfun(@(y)replace(string(subs(y.TrimRate,parameters,palias)),string(palias),pnames),x);
    dx0 = fdx(eom.States);

    fx = @(x)arrayfun(@(y)replace(string(subs(y.TrimState,parameters,palias)),string(palias),pnames),x);
    x0 = fx(eom.States);
    u0 = fx(eom.Inputs);

    fid = fopen(dir_name + "/" + name + "StateSpace.m","w");
    fprintf(fid,"function sys = %sStateSpace(p)\n",name);
    fprintf(fid,"   %% %s\n",state_str);
    fprintf(fid,"   %% %s\n",input_str);
    fprintf(fid,"   %% %s\n",param_str);
    fprintf(fid,"   M = %sSSMassMatrix(p);\n",name);
    fprintf(fid,"   H = %sSSForcingMatrix(p);\n",name);
    fprintf(fid,"   G = %sSSInputMatrix(p);\n",name);
    fprintf(fid,"   sys = ss(H,G,eye(size(H)),0);\n\n");
    fprintf(fid,"   sys.E = M;\n\n");
    fprintf(fid,"   sys.Offsets.dx = [\n");
    arrayfun(@(x)fprintf(fid,"      " + x + ";\n"),dx0);
    fprintf(fid,"   ];\n\n");
    fprintf(fid,"   sys.Offsets.x = [\n");
    arrayfun(@(x)fprintf(fid,"      " + x + ";\n"),x0);
    fprintf(fid,"   ];\n\n");
    fprintf(fid,"   sys.Offsets.u = [\n");
    arrayfun(@(x)fprintf(fid,"      " + x + ";\n"),u0);
    fprintf(fid,"   ];\n\n");
    fprintf(fid,"   sys.StateName = [\n");
    arrayfun(@(x)fprintf(fid,"      """ + x + """;\n"),state_names);
    fprintf(fid,"   ];\n\n");
    fprintf(fid,"   sys.InputName = [\n");
    arrayfun(@(x)fprintf(fid,"      """ + x + """;\n"),input_names);
    fprintf(fid,"   ];\n\n");
    fprintf(fid,"   sys.OutputName = [\n");
    arrayfun(@(x)fprintf(fid,"      """ + x + """;\n"),state_names);
    fprintf(fid,"   ];\n\n");
    fprintf(fid,"end\n");
    fclose(fid);
end