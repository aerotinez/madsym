function Vdbar = partialRate(obj,eomk,pose)
    arguments
        obj (1,1) Twist;
        eomk (1,1) KinematicEquations;
        pose (1,1) Pose = obj.Pose;
    end
    q = eomk.States;
    Vbar = obj.partial(eomk,pose);
    f = @(vbar)jacobian(vbar,q.state)*eomk.ForcingVector;
    Vdbar = reshape(arrayfun(f,reshape(Vbar,[],1)),size(Vbar));
end

