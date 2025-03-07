close("all"); clear; clc;
setmadsympath();

m = matlabColors();

ns = 100;
slip = deg2rad(linspace(-14,14,ns));
camber = deg2rad(linspace(-30,5,ns));

%% Pure camber, zero slip
X = meshgrid(slip);
Y = meshgrid(camber).';
Z = pacejkaLateralForce(X,Y,3E03);

fig = figure('Position',[200,200,640,640]);
axe = axes(fig);
hold(axe,'on');
h = surf(axe,rad2deg(X),rad2deg(Y),Z./1E03,'EdgeColor','none');
hold(axe,'off');
view(axe,0,0);
box(axe,'on');
title(axe,'Magic formula lateral force');
xlabel('Slip angle (deg)');
ylabel('Camber (deg)');
zlabel('Force (kN)');
grid(axe,'on');
axis(axe,'tight');
% daspect(axe,[max(slip)/max(camber),1,0.1]);

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
    Dy = fz.*pDy1.*exp(pDy2.*dfz)./(1 + pDy3.*camber.^2);
    
    % Compute Ey
    Ey = pEy1 + pEy2.*camber.^2 + pEy4.*sign(slip);
    
    % Compute Kya
    ang = fz./((pKy3 + pKy4.*camber.^2).*fz0);
    Kya = pKy1*fz0.*sin(pKy2.*atan(ang))./(1 + pKy5*camber.^2);
    
    % Compute By
    By = Kya./(Cy*Dy);
    
    % Compute Kyy
    Kyg = (pKy6 + pKy7*dfz).*fz;
    
    % Compute final lateral force coefficient
    Bg = Kyg./(Cg*Dy);
    
    % Compute lateral force Fy0
    fy = Dy.*sin(Cy.*atan(By.*slip - Ey.*(By.*slip - atan(By.*slip)))) + ...
        Cg.*atan(Bg.*camber - Eg.*(Bg.*camber - atan(Bg.*camber)));
end
