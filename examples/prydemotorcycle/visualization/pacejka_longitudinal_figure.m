close("all"); clear; clc;
setmadsympath();

m = matlabColors();

ns = 1000;
k = linspace(-1,1,ns);

fz = linspace(0,3E03,ns);
fx = pacejkaLongitudinalForce(k,fz);

X = repmat(k,ns,1);
Y = fx;
Z = repmat(fz,ns,1);

fig = figure('Position',[100,100,640,240]);
axe = axes(fig);
hold(axe,'on');
h = surf(axe,X.',Y.',Z,'EdgeColor','none');
hold(axe,'off');
view(axe,[0,90]);
box(axe,'on');
title(axe,'Magic formula longitudinal force');
xlabel('Longitudinal slip (normalized)');
ylabel('Force (kN)');
yticklabels(axe,arrayfun(@string,axe.YTick./1E03));
cb = colorbar(axe);
cb.Label.String = "Vertical force (kN)";
cb.TickLabels = arrayfun(@string,cb.Ticks/1E03);
grid(axe,'on');

dir = "C:\Users\marti\PhD\Thesis\MotorcycleDynamics\ForcesAndMoments\Figures\";
saveas(fig,dir + "longitudinal_force.eps",'epsc');

function fx = pacejkaLongitudinalForce(k,fz)
    arguments
        k (1,:) double;
        fz (:,1) double;
    end
    fz0 = 1.6E03;
    dfz = (fz - fz0)./fz0;

    C = 1.6064;
    pDx1 = 1.2017;
    pDx2 = -0.0922;
    pEx1 = 0.0263;
    pEx2 = 0.27056;
    pEx3 = -0.0769;
    pEx4 = 1.1268;
    pKx1 = 25.94;
    pKx2 = -4.233;
    pKx3 = 0.3369;

    D = fz.*(pDx1 + pDx2.*dfz);
    E = (pEx1 + pEx2.*dfz + pEx3.*dfz.^2).*(1 - pEx4.*sign(k));
    K = fz.*(pKx1 + pKx2.*dfz).*exp(pKx3.*dfz);
    B = K./(C.*D);

    fx = D.*sin(C.*atan(B*k - E.*(B.*k - atan(B.*k))));
end