close("all"); clear; clc;
setmadsympath();

Vx = [30,50,80,110,130];
k = 5;

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

mdr = iy*wd - (mrr - (fx.*(rr - dz) + mb));

hold on;
plot(mdr)
plot(results.My_DR_2)
hold off;