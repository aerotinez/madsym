function eoml = linearize(obj,lin_states,trim_point)
    arguments
        obj (1,1) MotionEquations;
        lin_states (:,1) sym;
        trim_point (1,1) TrimPoint = TrimPoint.empty();
    end

    if ~all(isDynamicVariable(lin_states))
        error('Linearization states must be dynamic variables.')
    end

    lin_rates = diff(lin_states);

    f0 = obj.MassMatrix*obj.Rates;
    f1 = -obj.ForcingVector;

    M = subsTrim(jacobian(f0,lin_rates),trim_point);

    Jf0 = subsTrim(jacobian(f0,lin_states),trim_point);
    Jf1 = subsTrim(jacobian(f1,lin_states),trim_point);

    G = -subsTrim(jacobian(f1,obj.Inputs),trim_point);
    H = -(Jf0 + Jf1);

    eoml = LinearizedMotionEquations(lin_states,M,H,obj.Inputs,G);
end