function eom = subsParams(obj,ovals,nvals)
    arguments
        obj (1,1) MotionEquations
        ovals sym;
        nvals sym;
    end
    x = obj.States;
    u = obj.Inputs;
    f = @(x)subs(x,ovals,nvals);
    eom = MotionEquations(x,f(obj.MassMatrix),f(obj.ForcingVector),u);
end