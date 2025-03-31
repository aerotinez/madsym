function matlabFunction(obj,name,states,inputs,varying_parameters)
    arguments
        obj (1,1) LinearizedMotionEquations
        name (1,1) string;
        states (:,1) DynamicVariable = obj.States;
        inputs (:,1) DynamicVariable = obj.Inputs;
        varying_parameters (:,1) = sym.empty(0,1);
    end

    if numel(states.dependent) > 0
        error("Output states cannot be dependent.")
    end

    switch isempty(varying_parameters)
        case true
            obj.matlabFunctionSS(name,states,inputs);
        case false
            obj.matlabFunctionLPVSS(name,states,inputs,varying_parameters);
    end
end