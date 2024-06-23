function J = jacobian(obj,qd,N)
    arguments
        obj (1,1) Frame;
        qd (1,:) sym;
        N (1,1) Frame = Frame();
    end 
    J = N.dcm*jacobian(obj.w,qd);
end