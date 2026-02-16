function setColor(obj, c)
    setColor@anim.element(obj ,c);
    setColor(obj.XAxis, c);
    setColor(obj.YAxis, c);
    setColor(obj.ZAxis, c);
end