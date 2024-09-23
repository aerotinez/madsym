close('all'); clear; clc;
setmadsympath();
load("poles_lagrange");
load("poles_gibbs_appell");

xg = sort(poles_lagrange(:));
xl = sort(poles_gibbs_appell(:));

figure();
hold("on");
c = matplotlibcolors;
scatter(real(xg),imag(xg),'filled','SizeData',100);
scatter(real(xl),imag(xl),'filled','SizeData',10);
hold("off");
axe = gca;
% set(axe,'Color','k')
box("on");
grid("on");
% axe.GridColor = ones(1,3);
xlim([-255,10]);
ylim([-63,63]);
xlabel("Re");
ylabel("Im");
leg = legend("Lagrange","Gibbs-Appell","Location","northwest");
set(leg,'color',ones(1,3));