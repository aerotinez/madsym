function h = plotFrame(axe,pose,options)
    arguments
        axe (1,1) matlab.graphics.axis.Axes;
        pose (1,1) rigidtform3d;
    end
    arguments (Repeating)
        options
    end
    h.x = plotAxis(axe,1,pose,options{:});
    h.y = plotAxis(axe,2,pose,options{:});
    h.z = plotAxis(axe,3,pose,options{:});
end