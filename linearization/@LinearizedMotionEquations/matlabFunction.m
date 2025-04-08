function matlabFunction(obj,name,parameters,varying_parameters)
    arguments
        obj (1,1) LinearizedMotionEquations
        name (1,1) string;
        parameters (:,1) = sym.empty(0,1);
        varying_parameters (:,1) = sym.empty(0,1);
    end
    
    switch isempty(varying_parameters)
        case true
            obj.matlabFunctionSS(name,parameters);
        case false
            obj.matlabFunctionLPVSS(name,parameters,varying_parameters);
    end
end