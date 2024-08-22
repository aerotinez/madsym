function tform = transform(obj,T)
    arguments
        obj (1,1) Pose;
        T (1,1) Pose = Pose();
    end
    tform = simplify(expand(T.inv().Transform*obj.Transform));
end