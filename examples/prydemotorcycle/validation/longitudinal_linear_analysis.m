close("all"); clear; clc;
setmadsympath();

%% Parameters
bs = bigSportsParameters;
s2m = @(x)cell2mat(struct2cell(x));
disp(bikeSimToPrydeParameters(bs,30/3.6));
params = @(v)s2m(bikeSimToPrydeParameters(bs,v/3.6));

%% Model
nv = 1000;
vmin = 1;
vmax = 300;
vx = linspace(vmin,vmax,nv);
plant = @prydeMotorcycleLongitudinalStateSpace;
f = @(v)plant(params(v));
sys = arrayfun(f,vx.',"uniform",0);

%% Poles
poles = sortpoles(cell2mat(cellfun(@pole,sys,"uniform",0).'));

%% Natural frequencies
m = matlabColors();

fig = figure('Position',[100,480,640,240]);
tl = tiledlayout(fig,1,2);
axe = nexttile(tl);
h = plot(axe,vx,real(poles),'LineWidth',2);
axis(axe,'tight');
box(axe,'on');
xlabel(axe,'Speed (km/h)');
ylabel(axe,'Re (1/s)');
grid(axe,'on');
grid(axe,'minor');

axe = nexttile(tl);
plot(axe,vx,imag(poles),'LineWidth',2);
axis(axe,'tight');
box(axe,'on');
xlabel(axe,'Speed (km/h)');
ylabel(axe,'Im (rad/s)');
grid(axe,'on');
grid(axe,'minor');

sgtitle('Speed vs natural fequency');

% dir = 'C:\Users\marti\PhD\Thesis\MotorcycleDynamics\Validation\Figures\';
% saveas(fig,string(dir) + "speed_natural_frequency.eps",'epsc');

%% Nyquist plot
fig = figure('Position',[100,100,640,240]);
axe = axes(fig);
c = linspace(0,1,size(poles,2));

hold(axe,'on');
for k = 1:3
    p = poles(k,:);
    h = scatter(axe,real(p),imag(p),20,c,"filled");
end
hold(axe,'off');

xticks(axe,[]);
yticks(axe,[]);
title(axe,"Nyquist response");
xlabel(axe,"Re (1/s)");
ylabel(axe,"Im (rad/s)");
box(axe,"on");
xlim(axe,[-110,5])
sgrid(axe);
cb = colorbar(axe);
f = @(x)string((vmax - vmin)*double(string(x)) + vmin);
cb.TickLabels = cellfun(f,cb.TickLabels,'uniform',0);
cb.Label.String = "Speed (km/h)";

% dir = 'C:\Users\marti\PhD\Thesis\MotorcycleDynamics\Validation\Figures\';
% saveas(fig,string(dir) + "nyquist.eps",'epsc');

function poles = sortpoles(poles)
    [n,m] = size(poles);
    for k = 2:m
        p0 = poles(:,k - 1);
        p = poles(:,k);
        min_cost = Inf;
        all_perms = perms(1:n);
        best_perm = [];
        for i = 1:size(all_perms,1)
            perm = all_perms(i,:);
            cost = sum(abs(p0 - p(perm)));
            if cost < min_cost
                min_cost = cost;
                best_perm = perm;
            end
        end
        poles(:,k) = p(best_perm);
    end
end