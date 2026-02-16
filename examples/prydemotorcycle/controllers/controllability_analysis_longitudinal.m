close('all'); clear; clc;
setmadsympath();

%% Parameters
bs = bigSportsParameters;
s2m = @(x)cell2mat(struct2cell(x));
disp(bikeSimToPrydeParameters(bs,30/3.6));
params = @(v)s2m(bikeSimToPrydeParameters(bs,v/3.6));

%% BikeSim results

ns = 100;
Vx = linspace(30,130,ns);
idx = 1:3;

I = eye(3);

f = @(vx)prydeMotorcycleLongitudinalStateSpace(params(vx));
g = hsvHelper(f,Vx,I);
gwr = hsvHelper(f,Vx,I(1:2,:));
gvx = hsvHelper(f,Vx,I(1,:));

%% Plot results

fig = figure('Position',[100,100,[640,440]/1.3]);
axe = axes(fig);

hold(axe,'on');
for k = 1:3
    plot(axe,Vx,g(:,k),'LineWidth',1.5,'Color',[0, 0.4470, 0.7410]);
    plot(axe,Vx,gwr(:,k),'--','LineWidth',1.5,'Color',[0.8500, 0.3250, 0.0980]);
    plot(axe,Vx,gvx(:,k),':','LineWidth',1.5,'Color',[0.9290, 0.6940, 0.1250]);
end
hold(axe,'off');

box(axe,'on');
axis(axe,'tight');
grid(axe,'minor');

title(axe,'Longitudinal: Hankel singular values','FontSize',12);
xlabel(axe,'speed (km/h)','FontSize',12);
ylabel(axe,'HSV','FontSize',12);

leg_str = [
    "$\left[v_{x},\omega_{r},\omega_{f}\right]$";
    strings(3,1);
    "$\left[v_{x},\omega_{r}\right]$";
    strings(3,1);
    "$v_{x}$"
    ];

leg = legend(axe,leg_str(:), ...
    'Orientation','horizontal', ...
    'Location','southoutside', ...
    'FontSize',12, ...
    'Interpreter','latex' ...
    );

leg.ItemTokenSize = 10;

saveas(fig,'C:\Users\marti\PhD\Thesis\MotorcycleDynamics\LinearModeling\Figures\longitudinal_hsv.eps','epsc');

%% Ellipsoids

sys = f(130);
A = sys.E\sys.A;
B = sys.E\sys.B;

C = eye(3);

sys = ss(A,B,C,0);

Wc = gram(sys,'c');
[Fc,Vc] = gramellipse(Wc\eye(size(Wc)));

Wo = gram(sys,'o');
[Fo,Vo] = gramellipse(Wo);

%% Plot results

fig = figure('Position',fig.Position + 200*[1,1,0,0]);
axe = axes(fig);

hold(axe,'on');

patch(axe, ...
    'Faces',Fc, ...
    'Vertices',Vc, ...
    'FaceColor',[0,0.4470,0.7410], ...
    'FaceAlpha',0.25, ...
    'EdgeColor','none', ...
    'FaceLighting','gouraud' ...
    );

patch(axe, ...
    'Faces',Fo, ...
    'Vertices',Vo, ...
    'FaceColor',[0.8500,0.3250,0.0980], ...
    'FaceAlpha',0.5, ...
    'EdgeColor','none', ...
    'FaceLighting','gouraud' ...
    );

hold(axe,'off');
box(axe,'on');
% axis(axe,'equal');
axis(axe,'tight');
view(axe,3);
light(axe);
camproj(axe,'perspective');

title(axe,"Longitudinal: Controllability and Observability ellipsoids",'FontSize',12);
xlabel(axe,"v_{x} (m/s)",'FontSize',12,'Interpreter','tex');
ylabel(axe,"\omega_r (rad/s)",'FontSize',12,'Interpreter','tex');
zlabel(axe,"\omega_f (rad/s)",'FontSize',12,'Interpreter','tex');

legend(axe,'Controllability ellipsoid','Observability ellipsoid', ...
    'FontSize',12, ...
    'Location','southoutside', ...
    'Orientation','horizontal' ...
    );

saveas(fig,'C:\Users\marti\PhD\Thesis\MotorcycleDynamics\LinearModeling\Figures\longitudinal_ellipsoid.eps','epsc');

%% Helpers

function g = hsvHelper(f,vx,C)
    arguments
        f (1,1) function_handle;
        vx (1,:) double;
        C (:,:) double;
    end

    g = zeros(3,numel(vx));

    for k = 1:numel(vx)
        sys = f(vx(k));
        A = sys.E\sys.A;
        
        % B = sys.E\[
        %     0,0;
        %     0,0;
        %     0,0;
        %     0,0;
        %     1,0;
        %     0,1
        %     ];

        B = sys.E\sys.B;
        
        syso = ss(A,B,C,0, ...
            'StateName',sys.StateName, ...
            'InputName',sys.InputName, ...
            'OutputName',sys.OutputName(logical(sum(C,1))) ...
            );
        
        [~,g(:,k)] = balreal(syso);
    end
    
    g = g';
end