function eom_r = split(obj,xr,ur)
    arguments
        obj (1,1) LinearizedMotionEquations;
        xr (:,1) DynamicVariable;
        ur (:,1) DynamicVariable = obj.Inputs;
    end
    eqns = sym(obj);
    Px = jacobian(obj.States.state,xr.state);
    eqns_r = Px.'*eqns;
    [M,f] = massMatrixForm(eqns_r,xr.state);
    [H,g] = equationsToMatrix(f,xr.state);
    G = jacobian(-g,ur.state);
    eom_r = LinearizedMotionEquations(xr,M,H,G,ur);
end
