function Vm = matrix(obj,pose)
    arguments
        obj (1,1) Twist;
        pose (1,1) Pose = obj.Pose;
    end
    V = obj.vector(pose);
    wm = vec2skew(V(1:3));
    v = V(4:6);
     
    Vm = [
        wm,v;
        zeros(1,4)
        ];
end