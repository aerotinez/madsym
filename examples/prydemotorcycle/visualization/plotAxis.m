function ax = plotAxis(axe,idx,pose,options)
    arguments
        axe (1,1) matlab.graphics.axis.Axes;
        idx (1,1) double;
        pose (1,1) rigidtform3d;
        options.Scale (1,1) double = 0.3;
        options.Color (1,3) double = [0,0,0];
        options.LineStyle (1,1) string = "-";
    end
    x = pose.Translation(1);
    y = pose.Translation(2);
    z = pose.Translation(3);
    u = options.Scale.*pose.R(1,idx);
    v = options.Scale.*pose.R(2,idx);
    w = options.Scale.*pose.R(3,idx);

    hold(axe,'on');

    ax = quiver3(axe,x,y,z,u,v,w,'filled', ...
        'AutoScale','off', ...
        'MaxHeadSize',1, ...
        'Color',options.Color, ...
        'LineWidth',3.5, ...
        'LineStyle',char(options.LineStyle) ...
        );


    % str = 'xyz';
    % if ~isempty(char(options.Label))
    %     offset = pose.R*(0.05.*eye(3));
    % 
    %     p = [x,y,z].' + options.Scale.*pose.R(:,idx) + offset(:,idx);
    % 
    %     text(p(1),p(2),p(3), ...
    %         "$\mathbf{\hat{" + string(str(idx)) + "}_{" + options.Label + "}}$", ...
    %         "FontSize",22, ...
    %         "FontWeight","bold", ...
    %         "Color",options.Color, ...
    %         "Interpreter","latex");
    % end

    hold(axe,'off');
    uistack(ax,'top');
end