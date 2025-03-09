close("all"); clear; clc;
setmadsympath();
m = matlabColors();

ns = 1000;
slip = deg2rad(linspace(-10,10,ns));

%% Slip vs vertical force
fz = linspace(0,3E03,ns);
tz = pacejkaAligningTorque(slip,0,fz.'); 

X = rad2deg(repmat(slip,ns,1));
Y = tz;
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
title(axe,'Magic formula: slip angle vs vertical force vs aligning torque');
xlabel(axe,'Slip angle (deg)');
ylabel(axe,'Torque (Nm)');
cb = colorbar(axe);
cb.Label.String = "Vertical force (kN)";

dir = "C:\Users\marti\PhD\Thesis\MotorcycleDynamics\ForcesAndMoments\Figures\";
saveas(fig,dir + "aligning_torque_slip_vertical_force.eps",'epsc');

%% Slip vs camber
camber = deg2rad(linspace(-50,50,ns));

[X,Y] = meshgrid(slip,camber);
Z = pacejkaAligningTorque(X,Y,3E03);

fig = figure('Position',[100,100,640,240]);
axe = axes(fig);
hold(axe,'on');

surf(axe,rad2deg(X),rad2deg(Y),Z, ...
    'EdgeColor','none', ...
    'FaceColor',m.blue, ...
    'FaceLighting','gouraud');

hold(axe,'off');
view(axe,3);
box(axe,'on');
title(axe,'Magic formula: slip angle vs camber vs aligning torque');
xlabel('Slip angle (deg)');
ylabel('Camber (deg)');
zlabel('Torque (Nm)');
grid(axe,'on');
axis(axe,'tight');
light(axe,'Position',rotx(45)*[0,0,10].');

dir = "C:\Users\marti\PhD\Thesis\MotorcycleDynamics\ForcesAndMoments\Figures\";
saveas(fig,dir + "aligning_torque_slip_camber.eps",'epsc');

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

function tz = pacejkaAligningTorque(slip, camber, fz)
    arguments
        slip double;
        camber double;
        fz double;
    end

    % Reference load and normalized load variation
    fz0 = 1.6E03;
    dfz = (fz - fz0) ./ fz0;

    % Lateral force coefficients
    Cy = 0.93921;
    pDy1 = 1.1524;
    pDy2 = -0.01794;
    pDy3 = -0.06531;
    pKy1 = 26.601;
    pKy2 = 1.0167;
    pKy3 = 1.4989;
    pKy4 = 0.52567;
    pKy5 = -0.24064;

    % Compute Dy
    Dy = fz.*pDy1.*exp(pDy2.*dfz)./(1 + pDy3*camber.^2);
    
    % Compute Kya
    ang = atan2(fz,(pKy3 + pKy4*camber.^2).*fz0);
    Kya = pKy1*fz0.*sin(pKy2*ang)./(1 + pKy5*camber.^2);
    
    % Compute By
    By = Kya./(Cy*Dy);

    % Aligning torque coefficients
    Ct = 1.3115;
    qBz1 = 10.354;
    qBz2 = 4.3004;
    qBz5 = -0.34033;
    qBz6 = -0.13202;
    qBz9 = 10.118;
    qBz10 = -1.0508;
    qDz1 = 0.20059;
    qDz2 = 0.05282;
    qDz3 = -0.21116;
    qDz4 = -0.15944;
    qDz8 = 0.30941;
    qDz9 = 0;
    qDz10 = 0.10037;
    qDz11 = 0;
    qEz1 = -3.9247;
    qEz2 = 10.809;
    qEz5 = 0.9836;
    qHz3 = -0.04908;
    qHz4 = 0;

    % Compute Bt
    Bt = (qBz1 + qBz2*dfz) .* (1 + qBz5*abs(camber) + qBz6*camber.^2);

    % Compute Dt
    R0 = 80E-03;
    Dt = fz .* (R0 / fz0) .* (qDz1 + qDz2*dfz) .* (1 + qDz3*abs(camber) + qDz4*camber.^2);

    % Compute Et
    Et = (qEz1 + qEz2*dfz) .* (1 + qEz5*camber*(2/pi) .* atan(Bt*Ct*slip));

    % Compute Br
    Br = qBz9 + qBz10*By.*Cy;

    % Compute Dr
    Dr = fz * R0 .* ((qDz8 + qDz9*dfz) .* camber + (qDz10 + qDz11*dfz) .* camber .* abs(camber)) ...
         ./ sqrt(1 + slip.^2);

    % Compute Shr
    Shr = (qHz3 + qHz4*dfz) .* camber;

    % Compute Mzt0
    fy0 = pacejkaLateralForce(slip,0,fz);
    Mzt0 = -(Dt .* cos(Ct .* atan(Bt .* slip - Et .* (Bt .* slip - atan(Bt .* slip)))) ...
           ./ sqrt(1 + slip.^2)) .* fy0;

    % Compute Mzr0
    Mzr0 = Dr .* cos(atan(Br .* (slip + Shr)));

    % Output total aligning torque
    tz = Mzt0 + Mzr0;
end