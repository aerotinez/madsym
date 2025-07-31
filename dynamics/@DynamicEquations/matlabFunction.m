function matlabFunction(obj,name,x,parameters)
    arguments
        obj (1,1) DynamicEquations;
        name (1,1) string;
        x (:,1) DynamicVariable = obj.States;
        parameters (:,1) = sym.empty(0,1);
    end

    eom = obj;
    x = prettify(state(x));
    u = prettify(eom.Inputs.state);
    M = prettify(eom.MassMatrix);
    f0 = prettify(eom.f0);
    f1 = prettify(eom.f1);
    f2 = prettify(eom.f2);

    if isempty(parameters)
        parameters = setdiff(symvar([M(:).',f0.',f1.',f2.']),[x.',u.']).';
    end

    dir_name = name + "Dynamics";
    if ~exist(dir_name, 'dir')
        mkdir(dir_name)
    end

    Mstr = dir_name + "/" + name + "DynamicsMassMatrix";
    f0str = dir_name + "/" + name + "DynamicsCentrifugalForcingVector";
    f1str = dir_name + "/" + name + "DynamicsCoriolisForcingVector";
    f2str = dir_name + "/" + name + "DynamicsActiveForcingVector";

    state_str = "   states = [" + strjoin(string(x).') + "]";
    input_str = "   inputs = [" + strjoin(string(u).') + "]";
    param_str = "   params = [" + strjoin(string(parameters).') + "]";
    comment_str = [state_str,input_str,param_str,""].';

    matlabFunction(M, ...
        "File",Mstr, ...
        "Vars",{x,parameters}, ...
        "Comments",comment_str);

    matlabFunction(f0, ...
        "File",f0str, ...
        "Vars",{x,parameters}, ...
        "Comments",comment_str);

    matlabFunction(f1, ...
        "File",f1str, ...
        "Vars",{x,parameters}, ...
        "Comments",comment_str);

    matlabFunction(f2, ...
        "File",f2str, ...
        "Vars",{x,u,parameters}, ...
        "Comments",comment_str);

    fid = fopen(dir_name + "/" + name + "DynamicsForcingVector.m","w");
    fprintf(fid,"function f = %sDynamicsForcingVector(x,u,p)\n",name);
    fprintf(fid,"   %% %s\n",state_str);
    fprintf(fid,"   %% %s\n",input_str);
    fprintf(fid,"   %% %s\n",param_str);
    fprintf(fid,"   \n");
    fprintf(fid,"   f0 = %sDynamicsCentrifugalForcingVector(x,p);\n",name);
    fprintf(fid,"   f1 = %sDynamicsCoriolisForcingVector(x,p);\n",name);
    fprintf(fid,"   f2 = %sDynamicsActiveForcingVector(x,u,p);\n",name);
    fprintf(fid,"   f = f0 + f1 + f2;\n");
    fprintf(fid,"   \n");
    fprintf(fid,"end\n");
    fclose(fid);
end