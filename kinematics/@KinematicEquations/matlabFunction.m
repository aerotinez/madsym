function matlabFunction(obj,name,parameters)
    arguments
        obj (1,1) KinematicEquations;
        name (1,1) string;
        parameters (:,1) = sym.empty(0,1);
    end
    
    eom = obj;
    q = prettify(eom.States.state);
    u = prettify(eom.Inputs.state);
    M = prettify(eom.MassMatrix);
    J = prettify(eom.Jacobian);

    if isempty(parameters)
        parameters = setdiff(symvar(J),[q.',u.']).';
    end

    dir_name = name + "Kinematics";
    if ~exist(dir_name, 'dir')
        mkdir(dir_name)
    end

    Mstr = dir_name + "/" + name + "KinematicMassMatrix";
    Jstr = dir_name + "/" + name + "KinematicJacobian";

    state_str = "   states = [" + strjoin(string(q).') + "]";
    input_str = "   inputs = [" + strjoin(string(u).') + "]";
    param_str = "   params = [" + strjoin(string(parameters).') + "]";
    comment_str = [state_str,input_str,param_str,""].';

    vars = {q,parameters};
    matlabFunction(M,"File",Mstr,"Comments",comment_str);
    matlabFunction(J,"File",Jstr,"Vars",vars,"Comments",comment_str);

    fid = fopen(dir_name + "/" + name + "KinematicForcingVector.m","w");
    fprintf(fid,"function f = %sKinematicForcingVector(x,u,p)\n",name);
    fprintf(fid,"   %% %s\n",state_str);
    fprintf(fid,"   %% %s\n",input_str);
    fprintf(fid,"   %% %s\n",param_str);
    fprintf(fid,"   \n");
    fprintf(fid,"   f = %sKinematicJacobian(x,p)*u;\n",name);
    fprintf(fid,"   \n");
    fprintf(fid,"end\n");
    fclose(fid);
end