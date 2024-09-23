function eom = auxiliaryEquations(v,eqns,eomk,F)
    arguments
        v (:,1) DynamicVariable;
        eqns (:,1) sym;
        eomk (1,1) KinematicEquations;
        F (:,1) DynamicVariable = DynamicVariable.empty(0,1);
    end
    ades = subs(eqns,eomk.States.rate(),eomk.ForcingVector);
    [M,f] = massMatrixForm(ades,v.state);
    eom = MotionEquations(v,M,f,F);
end