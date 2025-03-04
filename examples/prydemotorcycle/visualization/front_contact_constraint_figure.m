close("all"); clear; clc;
setmadsympath();

%% Figure setup
fig = figure('Position',[10,100,640,640]);

axe = axes(fig);
% set(axe,'Color','none');

hold(axe,'on');
axis(axe,'equal');
axis(axe,'tight');
hold(axe,'off');

box(axe,'on');
light(axe);
view(axe,135,20);
xlabel(axe,'x (m)');
ylabel(axe,'y (m)');
zlabel(axe,'z (m)');
% xticks(axe,[]);
% yticks(axe,[]);
% zticks(axe,[]);
% axe.XColor = 'none';
% axe.YColor = 'none';
% axe.ZColor = 'none';

% title(axe,'Front contact point constraint','FontSize',24);

%% Geometric parameters
caster = 24;
a = 749E-03;
an = 84.8E-03;
b = 572.9E-03;
d = 26.5E-03;
e = 28.7E-03;
f = 234.4E-03;
h = 575E-03;
rf = 290E-03;
tf = 50E-03;
rr = 290E-03;
tr = 70E-03;

s = sind(caster);
c = cosd(caster);

lx = rf*s - an;
lz = -(rf*c^2 + (an - a)*s)/c;

%% Place and orient frames
m = matlabColors;
q = [60,20,0,-40];
pitch_r = 12;
pitch_f = 12;

% World frame
I = eye(3);
p0 = [-0.3,-0.3,0];
Tw = rigidtform3d(I,p0);

% Yaw frame
Ryaw = rotz(q(1));
pCr = [0,0,0];
Tyaw = rigidtform3d(Ryaw,pCr);

% Camber frame
Rcamber = Ryaw*rotx(q(2));
ptr = tr.*Ryaw(:,3);
Tcamber = rigidtform3d(Rcamber,ptr);

% Rear tire frame
Rpitch_r = Rcamber*roty(pitch_r);
pr = rr.*Rcamber(:,3);
Tpitch_r = rigidtform3d(Rpitch_r,pr);

% Pitch frame
Rpitch = Rcamber*roty(q(3));
Tpitch = rigidtform3d(Rpitch,pr);

% Rear chassis frame
pb = tr*Ryaw(:,3) + (h - tr)*Rpitch(:,3) + b*Rpitch(:,1);
Tb = rigidtform3d(Rpitch,pb);

% Caster frame
Rcaster = Rpitch*roty(caster).';
p_caster = tr*Ryaw(:,3) - tr*Rpitch(:,3) + b*Rpitch(:,1) + a*Rcaster(:,1);
Tcaster = rigidtform3d(Rcaster,p_caster);

% Front chassis frame
Rsteer = Rcaster*rotz(q(4));
ph = p_caster + e.*Rsteer(:,1) + f.*Rsteer(:,3);
Th = rigidtform3d(Rsteer,ph);

% Front tire frame
Rpitch_f = Rsteer*roty(caster)*roty(pitch_f);
pf = p_caster + lx.*Rsteer(:,1) - lz.*Rsteer(:,3);
Tf = rigidtform3d(Rpitch_f,pf);

% Front camber frame
Rf = Rsteer*roty(caster);
[yaw_f,camber_f,theta_f] = dcm2angle(Rf.','ZXY');

Rcamber_f = Rf*roty(rad2deg(theta_f)).';
ptf = pf - (rf - tf).*Rcamber_f(:,3);
Tcamber_f = rigidtform3d(Rcamber_f,ptf);

% Front yaw frame
Ryaw_f = Rcamber_f*rotx(rad2deg(camber_f)).';
pCf = ptf - tf.*Ryaw_f(:,3);
Tyaw_f = rigidtform3d(Ryaw_f,pCf);

%% Compute reach angle
ny = Rf(:,2);
nz = Ryaw(:,3);
ra = -rf.*Rf(:,3);
rb = rf.*cross(ny,cross(ny,nz)/norm(cross(ny,nz)));
reach = acosd((ra.'*rb)./(norm(ra)*norm(rb)));

%% Plot frames
plotFrame(axe,rigidtform3d(I,pf), ...
    'Color',[0,0,0]);

Cf0 = pf - (rf - tf).*Rf(:,3) - tf.*I(:,3);

plotFrame(axe,rigidtform3d(Rf,pf), ...
    'Color',m.blue);

R = Rf*roty(reach).';
Cf = pf - (rf - tf).*R(:,3) - tf.*I(:,3);

plotFrame(axe,rigidtform3d(R,pf), ...
    'Color',m.orange);

plotFrame(axe,rigidtform3d(Rf*rotx(rad2deg(camber_f)).',Cf0), ...
    'Color',m.blue, ...
    'LineStyle',':');

plotFrame(axe,rigidtform3d(R*rotx(rad2deg(camber_f)).',Cf), ...
    'Color',m.orange, ...
    'LineStyle',':');

nms = {
    '$\mathbf{T}_{w}$';
    '';
    '';
    '$\mathbf{T}_{f}$';
    '';
    '';
    '$\mathbf{T}_{\beta}$';
    };

legend(axe,nms{:}, ...
    'FontSize',12, ...
    'Position',[0.73,0.66,0.15,0.1], ...
    'Orientation','Vertical',...
    'AutoUpdate','off', ...
    'Interpreter','latex');

%% Plot dimension lines
plotDoFLine(axe, ...
    rigidtform3d(Rf,pf), ...
    rigidtform3d(Rf*roty(reach).',pf), ...
    'z', ...
    -0.5*[1,1]);

grid(axe,'on');

%% Plot and format motorcycle
mf = bigSportsFigure();
setPose(mf,q);

n0 = 0.75.*ones(1,3);
setColor(mf,n0,n0,n0,n0);
setAlpha(mf,0.25,0.25,0.25,0.25);
% delete(axe.Children([1:5,11:end - 1]));
axis(axe,'tight');

xlim(axe,[0.4,1.1]);
ylim(axe,[0.8,1.5]);
zlim(axe,[-0.1,0.6]);

%% Plot road surface

p = 2.*[
    -1,-1;
    -1,1;
    1,1;
    1,-1
    ];

hold(axe,'on');
h = fill3(axe,p(:,1),p(:,2),zeros(size(p,1),1),'k');
h.FaceAlpha = 0.25;
uistack(h,'bottom');
hold(axe,'off');

%% Save figure
dir = 'C:\Users\marti\PhD\Thesis\MotorcycleDynamics\NonlinearModeling\Figures\';
saveas(fig,string(dir) + "front_contact_constraint_unlabled.eps",'epsc');