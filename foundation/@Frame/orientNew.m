function Nnew = orientNew(obj,axis,angle)
    arguments
        obj (1,1) Frame;
        axis (1,:) char;
        angle (1,:) sym;
    end
    if ~(length(axis) == length(angle))
        error('axis and angle must be the same length');
    end
    R = obj.dcm;
    for i = 1:length(axis)
        switch axis(i)
        case 'x'
            R = R*Rx(angle(i));
        case 'y'
            R = R*Ry(angle(i));
        case 'z'
            R = R*Rz(angle(i));
        end
    end
    Nnew = Frame(simplify(expand(R)));
end