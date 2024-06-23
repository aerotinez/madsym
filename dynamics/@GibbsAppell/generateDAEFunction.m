function generateDAEFunction(obj,function_name)
    arguments
        obj (1,1) GibbsAppell;
        function_name (1,1) string;
    end
    x = prettify(obj.Equations.States);
    u = prettify(obj.Equations.Inputs);

    M = obj.Equations.MassMatrix;
    f = obj.Equations.ForcingVector;

    p = symvar([M,f]).';
    t = sym('t');
    p(p == t) = [];

    state_str = strjoin(["    states = [",strjoin(string(x).'),"]"],'');
    input_str = strjoin(["    inputs = [",strjoin(string(u).'),"]"],'');
    param_str = strjoin(["    params = [",strjoin(string(p).'),"]"],'');
    comment_str = [state_str,input_str,param_str,""].';
    mstr = strcat(function_name,"MassMatrix");
    fstr = strcat(function_name,"ForcingVector");

    matlabFunction(prettify(M), ... 
        "File",mstr, ...
        "Vars",{x,u,p}, ...
        "Comments",comment_str);

    matlabFunction(prettify(f), ...
        "File",fstr, ...
        "Vars",{x,u,p}, ...
        "Comments",comment_str ...
        );
        
    fid = fopen(strcat(function_name,".m"),'w');
    fprintf(fid,"function x_dot = %s(x,u,p)\n",function_name);
    fprintf(fid,"%%" + state_str + "\n");
    fprintf(fid,"%%" + input_str + "\n");
    fprintf(fid,"%%" + param_str + "\n");
    fprintf(fid,"M = %s(x,u,p);\n",mstr);
    fprintf(fid,"f = %s(x,u,p);\n",fstr);
    fprintf(fid,"x_dot = M\\f;\n");
    fprintf(fid,"end");
    fclose(fid);
end