function Vdbar = partialRate(obj,eomk)
    arguments
        obj (1,1) Twist;
        eomk (1,1) KinematicEquations;
    end
    Vbar = partial(obj,eomk);
    q = eomk.States;
    fk = eomk.ForcingVector;
    f = @(vb)jacobian(vb,q)*fk;
    Vdbar = reshape(arrayfun(f,reshape(Vbar,[],1)),size(Vbar));
end