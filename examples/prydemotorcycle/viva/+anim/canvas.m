function fig = canvas()
    fig = figure;
    fig.WindowState = "fullscreen";
    
    set(fig,'Color','k');
    axe = axes(fig);
    box("on");
    view(axe,3);
    light(axe);
    axis(axe,'equal');
    camproj(axe,'perspective');

    d = 1;

    xlim(axe,d*[-0.4,2]);
    ylim(axe,d*[-1.5,0.5]);
    zlim(axe,d*[0,1.6]);
    xticklabels(axe,'');
    yticklabels(axe,'');
    zticklabels(axe,'');
    set(axe,'Color','k');
    set(axe,'InnerPosition',[0,0,1,1]);
    set(axe,'XColor','k');
    set(axe,'YColor','k');
    set(axe,'ZColor','k');
    pause(0.25);
end