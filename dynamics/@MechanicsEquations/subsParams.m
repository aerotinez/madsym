function eom = subsParams(obj,ovals,nvals)
    arguments
        obj (1,1) MechanicsEquations
        ovals sym;
        nvals sym;
    end
    eomc = subsParams(obj.Constraints,ovals,nvals);
    eomk = subsParams(obj.Kinematics,ovals,nvals);
    eomd = arrayfun(@(eom)subsParams(eom,ovals,nvals),obj.BodyDynamics);

    if isempty(obj.Auxiliary)
        eom = MechanicsEquations(eomk,eomd,eomc);
        return;
    end

    eomv = subsParams(obj.Auxiliary,ovals,nvals);
    eom = MechanicsEquations(eomk,eomd,eomc,eomv);
end