function wd = angAccel(obj,N)
    arguments
        obj (1,1) Frame;
        N (1,1) Frame = Frame();
    end 
    wd = simplify(expand(N.dcm.'*obj.dcm*obj.wd));
end