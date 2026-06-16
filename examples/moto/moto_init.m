close("all"); clear; clc;
setmadsympath();

%% Setup
bs = bigSportsParameters();
p = bikeSimToPrydeV2Parameters(bs,130/3.6);

nx = [1,0,0].';
nz = [0,0,1].';

Nc = roty(rad2deg(p.caster)).';
ncx = Nc(:,1);

%% Screw Axes

% Rear yaw
p_yaw0 = zeros(3,1);
Syaw = rscrew('z',p_yaw0);

% Rear longitudinal
p_x0 = zeros(3,1);
Sx = pscrew('x');

% Rear lateral
p_y0 = zeros(3,1);
Sy = pscrew('y');

% Rear camber
p_camber0 = p.tr*nz;
Scamber = rscrew('x',p_camber0);

% Rear pitch
p_pitchr0 = p.Rr*nz;
Spitchr = rscrew('y',p_pitchr0);

% Pitch
p_pitch0 =  p_pitchr0;
Spitch = rscrew('y',p_pitch0);

% Steer
a = p.k*cos(p.caster) - p.e + p.j*sin(p.caster);
p_steer0 = p.b*nx + a*ncx;
Ssteer = rscrew('z',p_steer0,Nc);

% Front yaw
p_yawf0 = zeros(3,1);
Syawf = rscrew('z',p_yawf0);

% Front longitudinal
p_xf0 = p.p*nx;
Sxf = pscrew('x');

% Front lateral;
p_yf0 = p_xf0;
Syf = pscrew('y');

% Front camber
p_camberf0 = p_xf0 + p.tf*nz;
Scamberf = rscrew('x', p_camberf0);

% Front pitch
p_pitchf0 = p_xf0 + p.Rf*nz;
Sf = rscrew('y', p_pitchf0);

%% Helpers
function s = jointAxis(axis)
    arguments
        axis (1,1) char {mustBeMember(axis,'xyzXYZ')};
    end
    I = eye(3);
    s = I(:,lower(axis) == 'xyz');
end

function S = rscrew(axis,position,orientation)
    arguments
        axis;
        position (3,1) double;
        orientation (3,3) double = eye(3);
    end
    w = orientation*jointAxis(axis);
    S = [w.',cross(position,w).'].';
end

function S = pscrew(axis,orientation)
    arguments
        axis;
        orientation (3,3) double = eye(3);
    end
    S = [zeros(1,3),(orientation*jointAxis(axis)).'].';
end