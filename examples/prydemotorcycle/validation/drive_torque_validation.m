close("all"); clear; clc;
setmadsympath();

Vx = [30,50,80,110,130];

t = cell(1,5);
md_ref = cell(1,5);
md_est = cell(1,5);

for k = 1:5
    bs = bigSportsParameters();
    results_path = "G:\My Drive\BikeSimResults\BigSports\OpenLoop";
    speed_path = "\Vx" + num2str(Vx(k)) + "Kph\";
    file_name = "bikesim_results_" + num2str(Vx(k)) + "kph.csv";
    results = readtable(results_path + speed_path + file_name);
    
    rr = bs.RearTire.EffectiveRollingRadius;
    iy = bs.RearTire.SpinInertia;
    
    fx = results.Fx_2;
    dz = results.CmpT_2*1E-03;
    mb = results.My_Bk_2;
    mrr = results.My_RR_2;
    wd = results.AAy_W2;
    
    t{k} = results.Time;
    md_ref{k} = results.My_DR_2;
    md_est{k} = iy*wd - (mrr - (fx.*(rr - dz) + mb));
end

fig = figure('Position',[100,100,1720,240]);

tl = tiledlayout(1,5,'Parent',fig);

for k = 1:5
    axe = nexttile(tl,k);
    if k < 2
        ylabel(axe,'\tau_{dr} (Nm)');
    end
    hold(axe,'on');
    plot(axe,t{k},md_ref{k},'LineWidth',2);
    plot(axe,t{k},md_est{k},':','LineWidth',2);
    hold(axe,'off');
    box(axe,'on');
    axis(axe,'tight');
    title(axe,"Speed: " + num2str(Vx(k)) + "km/h");
    xlabel(axe,'Time (s)');
end

sgtitle('Rear wheel driving torque');
lg = legend('ref','est','Orientation','horizontal');
lg.Layout.Tile = 'South';