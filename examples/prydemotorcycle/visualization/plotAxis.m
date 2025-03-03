function ax = plotAxis(axe,idx,pose,scale,color,style)
    arguments
        axe (1,1) matlab.graphics.axis.Axes;
        idx (1,1) double;
        pose (1,1) rigidtform3d;
        scale (1,1) double;
        color (1,3) double;
        style (1,1) string = "-";
    end
    x = pose.Translation(1);
    y = pose.Translation(2);
    z = pose.Translation(3);
    u = scale.*pose.R(1,idx);
    v = scale.*pose.R(2,idx);
    w = scale.*pose.R(3,idx);

    hold(axe,'on');

    ax = quiver3(axe,x,y,z,u,v,w,'filled', ...
        'AutoScale','off', ...
        'MaxHeadSize',1, ...
        'Color',color, ...
        'LineWidth',3.5, ...
        'LineStyle',char(style) ...
        );

    hold(axe,'off');
    uistack(ax,'top');
end