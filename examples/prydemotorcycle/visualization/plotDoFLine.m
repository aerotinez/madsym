function plotDoFLine(axe,pose_a,pose_b,dir,len)
    arguments
        axe (1,1) matlab.graphics.axis.Axes;
        pose_a (1,1) rigidtform3d;
        pose_b (1,1) rigidtform3d;
        dir (1,1) char {mustBeMember(dir,'xyz')};
        len (1,2) double = [1,1];
    end
    dict = dictionary(["x","y","z"],{[1,0,0],[0,1,0],[0,0,1]});

    p0 = [
        0,0,0;
        cell2mat(dict(dir))
        ];

    pa = transformPointsForward(pose_a,len(1).*p0);
    pb = transformPointsForward(pose_b,len(2).*p0);

    hold(axe,'on');
    ha = plot3(axe,pa(:,1),pa(:,2),pa(:,3),'k--','LineWidth',1.5);
    hb = plot3(axe,pb(:,1),pb(:,2),pb(:,3),'k--','LineWidth',1.5);
    hold(axe,'off');

    uistack(ha,'bottom');
    uistack(hb,'bottom');
end