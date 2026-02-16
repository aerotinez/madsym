function setPosition(obj,p)
    setPosition@anim.element(obj,p);
    obj.Handle.Vertices = obj.Handle.Vertices + p;
end