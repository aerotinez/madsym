function setPosition(obj, p)
    setPosition@anim.element(obj, p);
    setPosition(obj.XAxis, p);
    setPosition(obj.YAxis, p);
    setPosition(obj.ZAxis, p);
end