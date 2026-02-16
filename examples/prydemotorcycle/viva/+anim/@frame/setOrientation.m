function setOrientation(obj, R)
    setOrientation@anim.element(obj, R)
    setOrientation(obj.XAxis, R);
    setOrientation(obj.YAxis, R);
    setOrientation(obj.ZAxis, R);
end