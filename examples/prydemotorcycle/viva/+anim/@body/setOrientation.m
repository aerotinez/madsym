function setOrientation(obj,R)
    setOrientation@anim.element(obj,R);
    obj.Handle.Vertices = obj.Handle.UserData*R';
end