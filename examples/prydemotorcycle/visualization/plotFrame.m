function h = plotFrame(axe,pose,color,style)
    arguments
        axe (1,1) matlab.graphics.axis.Axes;
        pose (1,1) rigidtform3d;
        color (1,3) double;
        style (1,1) string = "-";
    end
    scale = 0.3;
    h.x = plotAxis(axe,1,pose,scale,color,style);
    h.y = plotAxis(axe,2,pose,scale,color,style);
    h.z = plotAxis(axe,3,pose,scale,color,style);
end