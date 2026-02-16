close('all'); clear; clc;
% setmadsympath;

%% System

A = [
    0,-1.25;
    4,-6
    ];

B = [
    1.2;
    4.48
    ];

C = [20,0];

sys = ss(A,B,C,0);

%% Grammians
Wc = gram(sys,'c');
[Fc,Vc] = gramellipse(blkdiag(Wc\eye(size(Wc)),1));

Wo = gram(sys,'o');
[Fo,Vo] = gramellipse(blkdiag(Wo,1));

%% Plot results

fig = figure;
axe = axes(fig);

hold(axe,'on');

patch(axe, ...
    'Faces',Fc, ...
    'Vertices',Vc, ...
    'FaceColor',[0,0.4470,0.7410], ...
    'FaceAlpha',0.5, ...
    'EdgeColor','none' ...
    );

patch(axe, ...
    'Faces',Fo, ...
    'Vertices',Vo, ...
    'FaceColor',[0.8500,0.3250,0.0980], ...
    'FaceAlpha',0.5, ...
    'EdgeColor','none' ...
    );

hold(axe,'off');
box(axe,'on');
% axis(axe,'equal');
