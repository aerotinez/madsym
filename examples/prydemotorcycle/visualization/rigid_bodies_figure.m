close("all"); clear; clc;
setmadsympath();

%% Figure setup
fig = figure('Position',[100,100,720,640]);

axe = axes(fig);
set(axe,'Color','none');

hold(axe,'on');
axis(axe,'equal');
axis(axe,'tight');
zlim(axe,[0,1.6]);
hold(axe,'off');

box(axe,'on');
light(axe);
view(180,0);
xticks(axe,[]);
yticks(axe,[]);
zticks(axe,[]);
axe.XColor = 'none';
axe.YColor = 'none';
axe.ZColor = 'none';

% title(axe,'Motorcycle rigid bodies','FontSize',24);

%% Plot and format motorcycle
q = [0,0,0,0];

mf = bigSportsFigure();
setPose(mf,q);

m = matlabColors;

mf.RearTire.Patch.FaceColor = m.blue;
mf.SwingArm.Patch.FaceColor = m.orange;
mf.Base.Patch.FaceColor = m.yellow;
mf.Body.Patch.FaceColor = m.yellow;
mf.Handlebars.Patch.FaceColor = m.purple;
mf.ForkTop.Patch.FaceColor = m.purple;
mf.FrontFender.Patch.FaceColor = m.green;
mf.Fork.Patch.FaceColor = m.green;
mf.FrontTire.Patch.FaceColor = m.cyan;
mf.LeftLeg.Patch.FaceColor = m.red;
mf.RightLeg.Patch.FaceColor = m.red;
mf.Driver.Patch.FaceColor = [0,0,0];
mf.LeftArm.Patch.FaceColor = [0,0,0];
mf.RightArm.Patch.FaceColor = [0,0,0];

%% Configure legend
names = {
    'Rear tire';
    'Swing arm';
    'Sprung mass';
    '';
    'Steering system';
    '';
    'Fork';
    '';
    'Front tire';
    'Rider lower body';
    '';
    'Rider upper body';
    '';
    ''
    };

legend(axe,names{:}, ...
    'Fontsize', 12, ...
    'Orientation','horizontal', ...
    'Location','SouthOutside', ...
    'NumColumns',4 ...
    );

%% Save figure