function generateODEFunction(obj,function_name)
    arguments
        obj (1,1) GibbsAppell;
        function_name (1,1) string;
    end

    f = obj.dae2ode();

    x = prettify(obj.Equations.States);
    u = prettify(obj.Equations.Inputs);
    t = sym('t');
    p = symvar(f).';
    p(p == t) = [];

    state_str = strjoin(["    states = [",strjoin(string(x).'),"]"],'');
    input_str = strjoin(["    inputs = [",strjoin(string(u).'),"]"],'');
    param_str = strjoin(["    params = [",strjoin(string(p).'),"]"],'');
    comment_str = [state_str,input_str,param_str,""].';
    
    matlabFunction(prettify(f), ... 
        "File",function_name, ...
        "Vars",{x,u,p}, ...
        "Comments",comment_str);
end