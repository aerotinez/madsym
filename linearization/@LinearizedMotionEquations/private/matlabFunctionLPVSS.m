function matlabFunctionLPVSS(obj,name,parameters,varying_parameters)
    arguments
        obj (1,1) LinearizedMotionEquations;
        name (1,1) string;
        parameters (:,1) = sym.empty(0,1);
        varying_parameters (:,1) = sym.empty(0,1);
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

    dir_name = name + "LPVStateSpace";
    if ~exist(dir_name, 'dir')
        mkdir(dir_name)
    end

    Mstr = dir_name + "/" + name + "LPVSSMassMatrix";
    Hstr = dir_name + "/" + name + "LPVSSForcingMatrix";
    Gstr = dir_name + "/" + name + "LPVSSInputMatrix";

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

    fid = fopen(dir_name + "/" + name + "DataFcn.m","w");
    fprintf(fid,"function [A,B,C,D,E,dx0,x0,u0,y0,Delays] = %sDataFcn(~,p)\n",name);
    fprintf(fid,"   %% %s\n",state_str);
    fprintf(fid,"   %% %s\n",input_str);
    fprintf(fid,"   %% %s\n",param_str);
    fprintf(fid,"   M = %sLPVSSMassMatrix(p);\n",name);
    fprintf(fid,"   H = %sLPVSSForcingMatrix(p);\n",name);
    fprintf(fid,"   G = %sLPVSSInputMatrix(p);\n",name);
    fprintf(fid,"   A = H;\n");
    fprintf(fid,"   B = G;\n");
    fprintf(fid,"   C = eye(size(A));\n");
    fprintf(fid,"   D = zeros(size(B));\n");
    fprintf(fid,"   E = M;\n");
    fprintf(fid,"   dx0 = [\n");
    arrayfun(@(x)fprintf(fid,"      " + x + ";\n"),dx0);
    fprintf(fid,"   ];\n\n");
    fprintf(fid,"   x0 = [\n");
    arrayfun(@(x)fprintf(fid,"      " + x + ";\n"),x0);
    fprintf(fid,"   ];\n\n");
    fprintf(fid,"   u0 = [\n");
    arrayfun(@(x)fprintf(fid,"      " + x + ";\n"),u0);
    fprintf(fid,"   ];\n\n");
    fprintf(fid,"   y0 = zeros(size(A,1),1);\n");
    fprintf(fid,"   Delays = [];\n");
    fclose(fid);

    fid = fopen(dir_name + "/" + name + "LPVStateSpace.m","w");
    fprintf(fid,"function sys = %sLPVStateSpace()\n",name);
    fprintf(fid,"   %% %s\n",state_str);
    fprintf(fid,"   %% %s\n",input_str);
    fprintf(fid,"   %% %s\n\n",param_str);
    fprintf(fid,"   pnames = {\n");
    arrayfun(@(x)fprintf(fid,"      '%s';\n",string(x)),parameters);
    fprintf(fid,"   };\n\n");
    fprintf(fid,"   sys = lpvss(pnames,@%sDataFcn);\n\n",name);
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