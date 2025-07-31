function matlabFunction(obj,name,parameters)
    arguments
        obj (1,1) MotionEquations;
        name (1,1) string;
        parameters (:,1) = sym.empty(0,1);
    end

    eom = obj;
    x = prettify(eom.States.state);
    u = prettify(eom.Inputs.state);
    M = prettify(eom.MassMatrix);
    f = prettify(eom.ForcingVector);

    if isempty(parameters)
        parameters = setdiff(symvar([M(:).',f(:).']),[x.',u.']).';
    end

    dir_name = name + "DAE";
    if ~exist(dir_name, 'dir')
        mkdir(dir_name)
    end

    Mstr = dir_name + "/" + name + "MassMatrix";
    fstr = dir_name + "/" + name + "ForcingVector";

    state_str = "   states = [" + strjoin(string(x).') + "]";
    input_str = "   inputs = [" + strjoin(string(u).') + "]";
    param_str = "   params = [" + strjoin(string(parameters).') + "]";
    comment_str = [state_str,input_str,param_str,""].';

    vars = {x,u,parameters};
    matlabFunction(M,"File",Mstr,"Vars",vars,"Comments",comment_str);
    matlabFunction(f,"File",fstr,"Vars",vars,"Comments",comment_str);
end