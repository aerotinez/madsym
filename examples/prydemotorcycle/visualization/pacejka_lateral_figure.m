close("all"); clear; clc;
setmadsympath();
m = matlabColors();

ns = 1000;
slip = deg2rad(linspace(-10,10,ns));

%% Slip vs vertical force
fz = linspace(0,3E03,ns);
fy = pacejkaLateralForce(slip,0,fz.'); 

X = rad2deg(repmat(slip,ns,1));
Y = fy./1E03;
Z = repmat(fz,ns,1)./1E03;

fig = figure('Position',[100,100,640,240]);
axe = axes(fig);
hold(axe,'on');
surf(axe,X.',Y.',Z,'EdgeColor','none');
hold(axe,'off');
view(axe,0,90);
box(axe,'on');
axis(axe,'tight');
grid(axe,'on');
title(axe,'Magic formula: slip angle vs normal force vs side force');
xlabel(axe,'Slip angle (deg)');
ylabel(axe,'Force (kN)');
cb = colorbar(axe);
cb.Label.String = "Normal force (kN)";

dir = "C:\Users\marti\PhD\Thesis\MotorcycleDynamics\ForcesAndMoments\Figures\";
saveas(fig,dir + "lateral_force_slip_vertical_force.eps",'epsc');

%% Slip vs camber
camber = deg2rad(linspace(-50,50,ns));

[X,Y] = meshgrid(slip,camber);
Z = pacejkaLateralForce(X,Y,3E03);

fig = figure('Position',[100,100,640,240]);
axe = axes(fig);
camproj(axe,'perspective');
hold(axe,'on');

surf(axe,rad2deg(X),rad2deg(Y),Z./1E03, ...
    'EdgeColor','none', ...
    'FaceColor',m.blue, ...
    'FaceLighting','gouraud');

hold(axe,'off');
view(axe,3);
box(axe,'on');
title(axe,'Magic formula: slip angle vs camber vs side force');
xlabel('Slip angle (deg)');
ylabel('Camber (deg)');
zlabel('Force (kN)');
grid(axe,'on');
axis(axe,'tight');
light(axe,'Position',rotx(45)*[0,0,10].');

dir = "C:\Users\marti\PhD\Thesis\MotorcycleDynamics\ForcesAndMoments\Figures\";
saveas(fig,dir + "lateral_force_slip_camber.eps",'epsc');

%% Helper functions

function fy = pacejkaLateralForce(slip, camber, fz)
    arguments
        slip double;
        camber double;
        fz double;
    end

    fz0 = 1.6E03;
    dfz = (fz - fz0)./fz0;

    Cy = 0.93921;
    pDy1 = 1.1524;
    pDy2 = -0.01794;
    pDy3 = -0.06531;
    pEy1 = -0.94635;
    pEy2 = -0.09845;
    pEy4 = -1.6416;
    pKy1 = 26.601;
    pKy2 = 1.0167;
    pKy3 = 1.4989;
    pKy4 = 0.52567;
    pKy5 = -0.24064;
    Cg = 0.50732;
    pKy6 = 0.7667;
    pKy7 = 0;
    Eg = -4.7481;
    
    % Compute Dy
    Dy = fz.*pDy1.*exp(pDy2.*dfz)./(1 + pDy3*camber.^2);
    
    % Compute Ey
    Ey = pEy1 + pEy2*camber.^2 + pEy4*camber.*sign(slip);
    
    % Compute Kya
    ang = atan2(fz,(pKy3 + pKy4*camber.^2).*fz0);
    Kya = pKy1*fz0.*sin(pKy2*ang)./(1 + pKy5*camber.^2);
    
    % Compute By
    By = Kya./(Cy*Dy);
    
    % Compute Kyy
    Kyg = (pKy6 + pKy7*dfz).*fz;
    
    % Compute final lateral force coefficient
    Bg = Kyg./(Cg*Dy);
    
    % Compute lateral force Fy0
    fy = Dy.*sin(Cy*atan(By.*slip - Ey.*(By.*slip - atan(By.*slip)))) + ...
        Cg*atan(Bg.*camber - Eg.*(Bg.*camber - atan(Bg.*camber)));
end
