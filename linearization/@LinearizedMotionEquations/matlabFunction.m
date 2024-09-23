function matlabFunction(obj,name)
    arguments
        obj (1,1) LinearizedMotionEquations
        name (1,1) string
    end
    eom = obj;
    if ~isequal(obj.MassMatrix,eye(numel(obj.States)))
        eom = toODE(obj);
    end
    x = prettify(eom.States.state);
    u = prettify(eom.Inputs.state);
    A = prettify(eom.ForcingMatrix);
    B = prettify(eom.InputMatrix);
    p = setdiff(symvar([A,B]),x).';

    dir_name = name + "StateSpace";
    if ~exist(dir_name, 'dir')
        mkdir(dir_name)
    end

    Astr = dir_name + "/" + name + "StateMatrix";
    Bstr = dir_name + "/" + name + "InputMatrix";

    state_str = "   states = [" + strjoin(string(x).') + "]";
    input_str = "   inputs = [" + strjoin(string(u).') + "]";
    param_str = "   params = [" + strjoin(string(p).') + "]";
    comment_str = [state_str,input_str,param_str,""].';

    vars = {x,p};
    matlabFunction(A,"File",Astr,"Vars",vars,"Comments",comment_str);
    matlabFunction(B,"File",Bstr,"Vars",vars,"Comments",comment_str);

    fid = fopen(name + "StateSpace.m","w");
    fprintf(fid,"function sys = %sStateSpace(x,p)\n",name);
    fprintf(fid,"   %% %s\n",state_str);
    fprintf(fid,"   %% %s\n",input_str);
    fprintf(fid,"   %% %s\n",param_str);
    fprintf(fid,"   A = %sStateMatrix(x,p);\n",name);
    fprintf(fid,"   B = %sInputMatrix(x,p);\n",name);
    fprintf(fid,"   sys = ss(A,B,eye(size(A)),0);\n");
    fprintf(fid,"end\n");
    fclose(fid);
end