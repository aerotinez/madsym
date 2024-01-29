close('all'); clear; clc;
setmadsympath();
load("poles_lagrange");
load("poles_gibbs_appell");

xl = sort(poles_lagrange(:));
xg = sort(poles_gibbs_appell(:));

figure();
hold("on");
c = matplotlibcolors;
scatter(real(xg),imag(xg),'filled','SizeData',50,"CData",[0,0,0]);
scatter(real(xl),imag(xl),'filled','SizeData',5,"CData",[1,1,1]);
hold("off");
axe = gca;
% set(axe,'Color','k')
box("on");
grid("on");
% axe.GridColor = ones(1,3);
xlim([-252,10]);
ylim([-63,63]);
xlabel("Re");
ylabel("Im");
leg = legend("Gibbs-Appell","Lagrange","Location","northwest");
set(leg,'color',ones(1,3));