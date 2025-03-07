close("all"); clear; clc;
setmadsympath();

m = matlabColors();

ns = 1000;
k = linspace(-1,1,ns);

fz = linspace(1E03,3E03,ns).';
fx = zeros(ns);

for i = 1:ns
    fx(i,:) = pacejkaLongitudinalForce(k,fz(i));
end


% X = [
%     k;
%     k
%     ];
% 
% Y = [
%     fxmin;
%     fxmax
%     ];
% 
% Z = zeros(size(Y));
% 
% nVert = numel(X);
% 
% C = [repmat(fzmin,1,numel(k)), repmat(fzmax,1,numel(k))];
% 
% fig = figure('Position',[100,100,640,240]);
% axe = axes(fig);
% 
% hold(axe,'on');
% 
% h = surf(X,Y,Z);
% 
% % h.FaceVertexCData = 255.*linspace(0,1,nVert).';
% 
% plot(axe,k,fxmin,'k','LineWidth',1);
% plot(axe,k,pacejkaLongitudinalForce(k,1.5E03),'k','LineWidth',1);
% plot(axe,k,pacejkaLongitudinalForce(k,2E03),'k','LineWidth',1);
% plot(axe,k,pacejkaLongitudinalForce(k,2.5E03),'k','LineWidth',1);
% plot(axe,k,fxmax,'k','LineWidth',1);
% 
% hold(axe,'off');
% 
% box(axe,'on');
% 
% xlabel(axe,'\kappa (normalized)', ...
%     'FontSize',12, ...
%     'Interpreter','tex');
% 
% ylabel(axe,'f_{x} (N)', ...
%     'FontSize',12, ...
%     'Interpreter','tex');
% 
% title(axe,"Pacejka's magic formula");
% 
% dir = "C:\Users\marti\PhD\Thesis\MotorcycleDynamics\ForcesAndMoments\Figures\";
% saveas(fig,dir + "magic_formula.eps",'epsc');
% 
function fx = pacejkaLongitudinalForce(k,fz)
    arguments
        k (1,:) double;
        fz (1,1) double;
    end
    fz0 = 1.6E03;
    dfz = (fz - fz0)/fz0;

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

    D = fz*(pDx1 + pDx2*dfz);
    E = (pEx1 + pEx2*dfz + pEx3*dfz.^2)*(1 - pEx4*sign(k));
    K = fz*(pKx1 + pKx2*dfz)*exp(pKx3*dfz);
    B = K/(C*D);

    fx = D*sin(C*atan(B*k - E.*(B*k - atan(B*k))));
end