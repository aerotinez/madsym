function eoml = linearize(obj,lin_states,trim_point)
    arguments
        obj (1,1) DynamicMotionEquations;
        lin_states (:,1) sym;
        trim_point (1,1) TrimPoint = TrimPoint.empty();
    end

    if ~all(isDynamicVariable(lin_states))
        error('Linearization states must be dynamic variables.')
    end

    lin_rates = diff(lin_states);

    M = subsTrim(jacobian(obj.MassMatrix*obj.Rates,lin_rates),trim_point);

    Jf0 = subsTrim(jacobian(-obj.f0,lin_states),trim_point);
    Jf1 = subsTrim(jacobian(-obj.f1,lin_states),trim_point);
    Jf2 = subsTrim(jacobian(-obj.f2,lin_states),trim_point);

    G = subsTrim(jacobian(obj.f2,obj.Inputs),trim_point);
    H = -(Jf0 + Jf1 + Jf2);

    eoml = LinearizedMotionEquations(lin_states,M,H,obj.Inputs,G);
end