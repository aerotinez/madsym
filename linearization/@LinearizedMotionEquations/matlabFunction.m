function matlabFunction(obj,name,states)
    arguments
        obj (1,1) LinearizedMotionEquations
        name (1,1) string;
        states (:,1) DynamicVariable = obj.States;
    end

    if numel(states.dependent) > 0
        error("Output states cannot be dependent.")
    end

    eom = obj;
    x = prettify(states.state);
    u = prettify(eom.Inputs.state);
    P = jacobian(states.state,eom.States.state);
    M = prettify(eom.MassMatrix);
    H = prettify(eom.ForcingMatrix);
    G = prettify(eom.InputMatrix);
    p = setdiff(symvar([M,H,G]),x).';

    dir_name = name + "StateSpace";
    if ~exist(dir_name, 'dir')
        mkdir(dir_name)
    end

    Pstr = dir_name + "/" + name + "PermutationMatrix";
    Mstr = dir_name + "/" + name + "MassMatrix";
    Hstr = dir_name + "/" + name + "ForcingMatrix";
    Gstr = dir_name + "/" + name + "InputMatrix";

    state_str = "   states = [" + strjoin(string(x).') + "]";
    input_str = "   inputs = [" + strjoin(string(u).') + "]";
    param_str = "   params = [" + strjoin(string(p).') + "]";
    comment_str = [state_str,input_str,param_str,""].';

    vars = {p};
    matlabFunction(P,"File",Pstr,"Comments",comment_str);
    matlabFunction(M,"File",Mstr,"Vars",vars,"Comments",comment_str);
    matlabFunction(H,"File",Hstr,"Vars",vars,"Comments",comment_str);
    matlabFunction(G,"File",Gstr,"Vars",vars,"Comments",comment_str);

    f = @(x)arrayfun(@(y)strrep(string(y.state),"(t)",""),x);
    state_names = f(obj.States);
    input_names = f(obj.Inputs);

    fid = fopen(dir_name + "/" + name + "StateSpace.m","w");
    fprintf(fid,"function sys = %sStateSpace(p)\n",name);
    fprintf(fid,"   %% %s\n",state_str);
    fprintf(fid,"   %% %s\n",input_str);
    fprintf(fid,"   %% %s\n",param_str);
    fprintf(fid,"   P = %sPermutationMatrix();\n",name);
    fprintf(fid,"   M = %sMassMatrix(p);\n",name);
    fprintf(fid,"   H = %sForcingMatrix(p);\n",name);
    fprintf(fid,"   G = %sInputMatrix(p);\n",name);
    fprintf(fid,"   A = P*(M\\H);\n");
    fprintf(fid,"   B = P*(M\\G);\n");
    fprintf(fid,"   sys = ss(A,B,eye(size(A)),0);\n\n");
    fprintf(fid,"   sys.StateName = [\n");
    arrayfun(@(x)fprintf(fid,"      """ + x + """\n"),state_names);
    fprintf(fid,"   ];\n\n");
    fprintf(fid,"   sys.InputName = [\n");
    arrayfun(@(x)fprintf(fid,"      """ + x + """\n"),input_names);
    fprintf(fid,"   ];\n\n");
    fprintf(fid,"   sys.OutputName = [\n");
    arrayfun(@(x)fprintf(fid,"      """ + x + """\n"),state_names);
    fprintf(fid,"   ];\n\n");
    fprintf(fid,"end\n");
    fclose(fid);
end