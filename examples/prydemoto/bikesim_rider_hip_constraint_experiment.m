close("all"); clear; clc;
setmadsympath();

vx = fliplr([30,50,80,110,130]);
results_path = "G:\My Drive\BikeSimResults\BigSports\Chicane";

fig = figure("Position",[100,50,640,240]);
axe = axes(fig);
title("Rider hip motion","FontSize",12);
xlabel("Y_Rd_Mov (mm)","FontSize",12,'Interpreter','none');
ylabel("Z_Rd_Mov (mm)","FontSize",12,'Interpreter','none');
box(axe,'on');

xmes = cell(1,numel(vx));

hold(axe,'on');
for k = 1:numel(vx)
    speed_path = "\Vx" + num2str(vx(k)) + "Kph\";
    file_name = "bikesim_results_" + num2str(vx(k)) + "kph.csv";
    results = readtable(results_path + speed_path + file_name);
    plot(axe,results.Y_Rd_Mov,results.Z_Rd_Mov,"LineWidth",2);
    xmes{k} = results;
end
hold(axe,'off');
axis(axe,'tight');
% axis(axe,'equal');

leg_str = arrayfun(@num2str,vx,'UniformOutput',false);

lgd = legend(axe,leg_str, ...
    "Orientation","vertical", ...
    "Location","SouthEast", ...
    "FontSize",12);

title(lgd,"Speed (kph)","FontSize",12);
