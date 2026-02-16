function fig = canvas()
    fig = figure("Position",[100,100,640,640]);
    set(fig,'Color','k');
    axe = axes(fig);
    box("on");
    view(axe,3);
    light(axe);
    axis(axe,'equal');
    camproj(axe,'perspective');
    xticklabels(axe,'');
    yticklabels(axe,'');
    zticklabels(axe,'');
    set(axe,'Color','k');
    set(axe,'InnerPosition',[0,0,1,1]);
    set(axe,'XColor','k');
    set(axe,'YColor','k');
    set(axe,'ZColor','k');
end