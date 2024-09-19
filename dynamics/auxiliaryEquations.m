function eom = auxiliaryEquations(states,eqns,eomk,inputs)
    arguments
        states (1,1) StateVector;
        eqns (:,1) sym;
        eomk (1,1) KinematicEquations;
        inputs (1,1) GeneralizedCoordinates = GeneralizedCoordinates();
    end
    t = sym('t');
    qd = diff(eomk.States.All,t);
    ades = subs(eqns,qd,eomk.ForcingVector);

    x = [
        states.Coordinates;
        eomk.Inputs;
        states.Auxiliary
        ];

    [M,f] = massMatrixForm(ades,x.All);

    eom = MotionEquations(x,M,f,inputs);
end