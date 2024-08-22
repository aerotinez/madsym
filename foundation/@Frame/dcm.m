function R = dcm(obj,N)
    arguments
        obj (1,1) Frame;
        N (1,1) Frame = Frame();
    end
    Rb = [obj.x,obj.y,obj.z];
    Rs = [N.x,N.y,N.z];
    R = simplify(expand(Rs.'*Rb));
end