function generateStateSpace(obj,function_name)
    arguments
        obj (1,1) GibbsAppell;
        function_name (1,1) string;
    end
    q = obj.States.Coordinates.Independent;
    u = obj.States.Speeds.Independent;
    v = obj.States.Auxiliary;
    x = prettify([q;u;v]);

    M = obj.LinearizedEquations.MassMatrix;
    H = obj.LinearizedEquations.ForcingMatrix;
    G = obj.LinearizedEquations.InputMatrix;
    P = obj.LinearizedEquations.PermutationMatrix;

    Minv = syminv(M);
    A = P.'*(Minv*H);
    B = P.'*(Minv*G);

    p = symvar([A,B]).';
    p = p(p ~= sym('t'));

    state_str = strjoin(["    states = [",strjoin(string(x).'),"]"],'');
    param_str = strjoin(["    params = [",strjoin(string(p).'),"]"],'');
    comment_str = [state_str,param_str,""].';
    Astr = strcat(function_name,"StateMatrix");
    Bstr = strcat(function_name,"InputMatrix");

    matlabFunction(prettify(A), ... 
        "File",Astr, ...
        "Vars",{x,p}, ...
        "Comments",comment_str);

    matlabFunction(prettify(B), ...
        "File",Bstr, ...
        "Vars",{x,p}, ...
        "Comments",comment_str ...
        ); 
end