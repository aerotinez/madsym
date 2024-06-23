function w = angVel(obj,N)
    arguments
        obj (1,1) Frame;
        N (1,1) Frame = Frame();
    end 
    w = simplify(expand(N.dcm.'*obj.dcm*obj.w));
end