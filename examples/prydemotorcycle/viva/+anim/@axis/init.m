function init(obj, opts)
    arguments
        obj
        opts.Parent (1,1) matlab.graphics.axis.Axes = gca
        opts.Direction (1,3) char {mustBeMember(opts.Direction,'xyzXYZ')} = 'x'
        opts.Color (1,3) double {mustBeInRange(opts.Color, 0, 1)} = [0,0,0]
    end

    idx = 0.5 * double(lower(opts.Direction) == 'xyz');

    h = quiver3(0,0,0,idx(1),idx(2),idx(3),'off',"filled", ...
        'LineWidth',5, ...
        'ShowArrowHead','on', ...
        'AutoScale','off', ...
        "Parent",opts.Parent, ...
        "MaxHeadSize", 1, ...
        "Visible", "off");

    idxl = double(lower(opts.Direction) == 'xyz');
    hl = plot3([0,idxl(1)],[0,idxl(2)],[0,idxl(3)],'w--', ...
        "Visible","off", ...
        "LineWidth",2 ...
        );

    uistack(gca, "top");

    obj.Handle = h;
    obj.Direction = idx;
    obj.DimensionLine = hl;
end