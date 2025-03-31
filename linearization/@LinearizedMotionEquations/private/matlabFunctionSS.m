function matlabFunctionSS(obj,name,states,inputs)
    arguments
        obj (1,1) LinearizedMotionEquations
        name (1,1) string;
        states (:,1) DynamicVariable = obj.States;
        inputs (:,1) DynamicVariable = obj.Inputs;
    end
    
    eom = obj;
    x = prettify(states.state);
    u = prettify(eom.Inputs.state);
    P = eom.States.permMatInd;
    Sx = jacobian(states.state,eom.States.independent.state);
    M = prettify(eom.MassMatrix);
    H = prettify(eom.ForcingMatrix);
    G = prettify(eom.InputMatrix);
    Su = jacobian(inputs.state,eom.Inputs.state);
    p = setdiff(symvar([M,H,G]),x).';

    dir_name = name + "StateSpace";
    if ~exist(dir_name, 'dir')
        mkdir(dir_name)
    end

    Sxstr = dir_name + "/" + name + "StateSelectorMatrix";
    Pstr = dir_name + "/" + name + "PermutationMatrix";
    Mstr = dir_name + "/" + name + "MassMatrix";
    Hstr = dir_name + "/" + name + "ForcingMatrix";
    Gstr = dir_name + "/" + name + "InputMatrix";
    Sustr = dir_name + "/" + name + "InputSelectorMatrix";

    state_str = "   states = [" + strjoin(string(x).') + "]";
    input_str = "   inputs = [" + strjoin(string(u).') + "]";
    param_str = "   params = [" + strjoin(string(p).') + "]";
    comment_str = [state_str,input_str,param_str,""].';

    vars = {p};
    matlabFunction(Sx,"File",Sxstr,"Comments",comment_str);
    matlabFunction(P,"File",Pstr,"Comments",comment_str);
    matlabFunction(M,"File",Mstr,"Vars",vars,"Comments",comment_str);
    matlabFunction(H,"File",Hstr,"Vars",vars,"Comments",comment_str);
    matlabFunction(G,"File",Gstr,"Vars",vars,"Comments",comment_str);
    matlabFunction(Su,"File",Sustr,"Comments",comment_str);

    f = @(x)arrayfun(@(y)strrep(string(y.state),"(t)",""),x);
    state_names = f(states);
    input_names = f(inputs);

    fid = fopen(dir_name + "/" + name + "StateSpace.m","w");
    fprintf(fid,"function sys = %sStateSpace(p)\n",name);
    fprintf(fid,"   %% %s\n",state_str);
    fprintf(fid,"   %% %s\n",input_str);
    fprintf(fid,"   %% %s\n",param_str);
    fprintf(fid,"   Sx = %sStateSelectorMatrix();\n",name);
    fprintf(fid,"   P = %sPermutationMatrix();\n",name);
    fprintf(fid,"   M = %sMassMatrix(p);\n",name);
    fprintf(fid,"   H = %sForcingMatrix(p);\n",name);
    fprintf(fid,"   G = %sInputMatrix(p);\n",name);
    fprintf(fid,"   Su = %sInputSelectorMatrix();\n",name);
    fprintf(fid,"   A = Sx*P*(M\\H)*Sx.';\n");
    fprintf(fid,"   B = Sx*P*(M\\G)*Su.';\n");
    fprintf(fid,"   sys = ss(A,B,eye(size(A)),0);\n\n");
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