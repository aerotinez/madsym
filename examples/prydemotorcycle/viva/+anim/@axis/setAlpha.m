function setAlpha(obj, alpha)
    setAlpha@anim.element(obj, alpha);
    obj.Handle.Visible = "on";
    obj.Handle.Color = alpha * obj.Color;
end