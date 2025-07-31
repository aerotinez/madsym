function eom = subsParams(obj,ovals,nvals)
    arguments
        obj (1,1) KinematicEquations
        ovals sym;
        nvals sym;
    end
    x = obj.States;
    u = obj.Inputs;
    J = subs(obj.Jacobian,ovals,nvals);
    kdes = rate(x) - J*state(u);
    eom = KinematicEquations(x,kdes,u);
end