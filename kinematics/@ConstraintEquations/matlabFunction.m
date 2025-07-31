function matlabFunction(obj,name,parameters)
    arguments
        obj (1,1) ConstraintEquations;
        name (1,1) string;
        parameters (:,1) = sym.empty(0,1);
    end

    eom = obj;
    q = prettify(eom.States.state);
    u = prettify(eom.Speeds.state);
    M = prettify(eom.Jacobian);
    dJ = prettify(eom.JacobianRate);

    if isempty(parameters)
        parameters = setdiff(symvar(M),[q.',u.']).';
    end

    dir_name = name + "Constraints";
    if ~exist(dir_name, 'dir')
        mkdir(dir_name)
    end

    Mstr = dir_name + "/" + name + "ConstraintMassMatrix";
    Jstr = dir_name + "/" + name + "ConstraintJacobianRate";

    state_str = "   states = [" + strjoin(string(q).') + "]";
    input_str = "   inputs = [" + strjoin(string(u).') + "]";
    param_str = "   params = [" + strjoin(string(parameters).') + "]";
    comment_str = [state_str,input_str,param_str,""].';

    matlabFunction(M, ...
        "File",Mstr, ...
        "Vars",{q,parameters}, ...
        "Comments",comment_str);

    matlabFunction(dJ, ...
        "File",Jstr, ...
        "Vars",{q,u,parameters}, ...
        "Comments",comment_str);

    fid = fopen(dir_name + "/" + name + "ConstraintForcingVector.m","w");
    fprintf(fid,"function f = %sConstraintForcingVector(x,u,p)\n",name);
    fprintf(fid,"   %% %s\n",state_str);
    fprintf(fid,"   %% %s\n",input_str);
    fprintf(fid,"   %% %s\n",param_str);
    fprintf(fid,"   \n");
    fprintf(fid,"   f = -%sConstraintJacobianRate(x,u,p)*u;\n",name);
    fprintf(fid,"   \n");
    fprintf(fid,"end\n");
    fclose(fid);
end