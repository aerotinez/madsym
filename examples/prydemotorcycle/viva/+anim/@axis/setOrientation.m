function setOrientation(obj, R)
    setOrientation@anim.element(obj,R);

    p = obj.Direction*R';
    
    obj.Handle.UData = p(1);
    obj.Handle.VData = p(2);
    obj.Handle.WData = p(3);
end