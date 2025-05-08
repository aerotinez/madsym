function eom = subsParams(obj,ovals,nvals)
    arguments
        obj (1,1) LinearizedMotionEquations
        ovals sym;
        nvals sym;
    end
    x = obj.States;
    u = obj.Inputs;
    f = @(x)subs(x,ovals,nvals);
    M = f(obj.MassMatrix);
    H = f(obj.ForcingMatrix);
    G = f(obj.InputMatrix);
    eom = LinearizedMotionEquations(x,M,H,G,u);
end