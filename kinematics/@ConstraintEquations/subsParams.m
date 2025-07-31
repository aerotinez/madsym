function eom = subsParams(obj,ovals,nvals)
    arguments
        obj (1,1) ConstraintEquations
        ovals sym;
        nvals sym;
    end
    f = @(x)subs(x,ovals,nvals);
    eom = obj;
    eom.Configuration = f(obj.Configuration);
    eom.Jacobian = f(obj.Jacobian);
    eom.JacobianRate = f(obj.JacobianRate);
end