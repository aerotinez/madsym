function setAlpha(obj, alpha)
    setAlpha@anim.element(obj ,alpha);
    setAlpha(obj.XAxis, alpha);
    setAlpha(obj.YAxis, alpha);
    setAlpha(obj.ZAxis, alpha);
end