close("all"); clear; clc;
setmadsympath();

%% BikeSim parameters
bs = bigSportsParameters();
p = bikeSimToPrydeV2Parameters(bs,130/3.6);

%% Rear contact point
Tyaw0 = eye(4);
Tx0 = eye(4);
Ty0 = eye(4);

%% Rear camber joint
Tcamber0 = [
    eye(3),[0,0,p.tr].';
    0,0,0,1
    ];

%% Rear axle joint
Tpitchr0 = [
    eye(3),[0,0,p.Rr].';
    0,0,0,1
    ];

%% Rear tire
Tr0 = Tpitchr0;

%% Rear chassis
Tb0 = [
    eye(3),[p.h,0,p.b].';
    0,0,0,1
    ];

%% Steering joint
Nc = roty(rad2deg(p.caster)).';
nx = [1,0,0].';
ncx = Nc(:,1);

Tsteer0 = [
    Nc,p.b*nx + p.a*ncx;
    0,0,0,1
    ];

%% Front chassis