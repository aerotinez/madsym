close("all"); clear; clc;
setmadsympath();

%% Magic formula parameters
tpf = simscape.multibody.tirread("front_tire.tir");
tpr = simscape.multibody.tirread("rear_tire.tir");

%% Setup
bs = bigSportsParameters();
bs.FrontTire.EffectiveRollingRadius = tpf.DIMENSION.UNLOADED_RADIUS;
bs.FrontTire.Mass = tpf.INERTIA.MASS;
bs.FrontTire.SpinInertia = tpf.INERTIA.IYY;
bs.FrontTire.UndeflectedCrownRadius = tpf.DIMENSION.UNLOADED_RADIUS - tpf.DIMENSION.RIM_RADIUS;
bs.RearTire.EffectiveRollingRadius = tpr.DIMENSION.UNLOADED_RADIUS;
bs.RearTire.Mass = tpr.INERTIA.MASS;
bs.RearTire.SpinInertia = tpr.INERTIA.IYY;
bs.RearTire.UndeflectedCrownRadius = tpr.DIMENSION.UNLOADED_RADIUS - tpr.DIMENSION.RIM_RADIUS;
vx = 130/3.6;
p = bikeSimToPrydeV2Parameters(bs,vx + 1);
caster = deg2rad(bs.SteeringHead.Caster);

%% Initialize pose
N = eye(3);
O = zeros(3,1);

%% helper functions
body = @(N,O,I,m)struct('N',N,'O',O,'I',I,'m',m);

%% Sprung mass
sm = bs.SprungMass;

Ism = [
    sm.Ixx, 0, sm.Ixz;
    0, sm.Iyy, 0;
    sm.Ixz, 0, sm.Izz
    ];

Osm = [
    -sm.CoMOffset;
    0;
    sm.CoMHeight
    ];

sprung_mass = body(N,Osm,Ism,sm.Mass);

%% Rider lower body
rl = bs.RiderLowerBody;

Irl = [
    rl.Ixx, 0, rl.Ixz;
    0, rl.Iyy, 0;
    rl.Ixz, 0, rl.Izz
    ];

Orl = [
    -rl.CoMOffset;
    0;
    rl.CoMHeight
    ];
    
rider_lower_body = body(N,Orl,Irl,rl.Mass);

%% Rider upper body
ru = bs.RiderUpperBody;

Iru = [
    ru.Ixx, 0, ru.Ixz;
    0, ru.Iyy, 0;
    ru.Ixz, 0, ru.Izz
    ];

Nru = N*roty(ru.ForwardLean);

Oru = -ru.CoMOffset.*N(:,1) + ru.LeanAxisHeight.*N(:,3) + ...
    ru.CoMHeight.*Nru(:,3);

rider_upper_body = body(Nru,Oru,Iru,ru.Mass);

%% Swing arm
sa = bs.SwingArm;

Isa = zeros(3);

Nsa = N*roty(-asind((sa.PivotHeight - sa.AxelHeight)/sa.Length));

Osa = -sm.Wheelbase.*N(:,1) + sa.AxelHeight.*N(:,3) + ...
    sa.CoMOffset.*Nsa(:,1) + sa.CoMHeight.*Nsa(:,3);

swing_arm = body(Nsa,Osa,Isa,sa.Mass);

%% Rear tire
rt = bs.RearTire;

Irt = diag([tpr.INERTIA.IXX,tpr.INERTIA.IYY,tpr.INERTIA.IXX]);

Ort = -sm.Wheelbase.*N(:,1) + tpr.DIMENSION.UNLOADED_RADIUS*N(:,3);

rear_tire = body(N,Ort,Irt,tpr.INERTIA.MASS);

%% Front tire
ft = bs.FrontTire;

Ift = diag([tpf.INERTIA.IXX,tpf.INERTIA.IYY,tpf.INERTIA.IXX]);

Oft = tpf.DIMENSION.UNLOADED_RADIUS.*N(:,3);

front_tire = body(N,Oft,Ift,tpf.INERTIA.MASS);

%% Steering head
sh = bs.SteeringHead;

Ish = [
    sh.Ixx, 0, sh.Ixz;
    0, sh.Iyy, 0;
    sh.Ixz, 0, sh.Izz
    ];

Nsh = N*roty(sh.Caster).';

H = Oft - sh.Rake.*Nsh(:,1) + sh.ForkLength.*Nsh(:,3);
Osh = H + sh.CoMOffset.*Nsh(:,1) + sh.CoMHeight.*Nsh(:,3);

steering_head = body(Nsh,Osh,Ish,sh.Mass);

%% Fork
ff = bs.Fork;

Iff = zeros(3);

Off = H + ff.CoMOffset.*Nsh(:,1) - ff.CoMHeight.*Nsh(:,3);

fork = body(Nsh,Off,Iff,ff.Mass);

%% Rear body

rear_bodies = [
    sprung_mass;
    swing_arm;
    rider_upper_body;
    rider_lower_body
    ];

% total mass
mb = sum([rear_bodies.m]);

% mass center
Ob = sum([rear_bodies.O].*[rear_bodies.m],2)/p.mb;
rb = Ob - [rear_bodies.O];

% inertia tensor
I = zeros(3,3,numel(rear_bodies));
for k = 1:numel(rear_bodies)
    m = rear_bodies(k).m;
    d = rb(:,k);
    Nk = rear_bodies(k).N;
    Ik = Nk.'*rear_bodies(k).I*Nk;
    I(:,:,k) = Ik + m*(d.'*d*eye(3) - d*d.');
end
Ib = sum(I,3);

%% Front body

front_bodies = [
    steering_head
    fork
    ];

% total mass
mh = sum([front_bodies.m]);

% mass center
Oh = sum([front_bodies.O].*[front_bodies.m],2)/mh;
rf = Oh - [front_bodies.O];
l = abs(Ob(1));
an = tpf.DIMENSION.UNLOADED_RADIUS*sin(caster) - sh.Rake;
a = l*cos(caster) + an;
B = O - l.*N(:,1) + a.*Nsh(:,1);
Gf = Nsh.'*(Oh - B);
f = p.j*cos(caster) - p.k*sin(caster);

% inertia tensor
I = zeros(3,3,numel(front_bodies));
for k = 1:numel(front_bodies)
    m = front_bodies(k).m;
    d = rf(:,k);
    Nk = front_bodies(k).N;
    Ik = Nk.'*front_bodies(k).I*Nk;
    I(:,:,k) = Ik + m*(d.'*d*eye(3) - d*d.');
end
If = sum(I,3);

dR = tpf.DIMENSION.UNLOADED_RADIUS - tpr.DIMENSION.UNLOADED_RADIUS;
lb = sm.Wheelbase*cos(caster) + dR*sin(caster) - sh.Rake;
lh = sm.Wheelbase*sin(caster) - dR*cos(caster);
lz = l*sin(caster) - tpf.DIMENSION.UNLOADED_RADIUS*cos(caster);

m = p.mf + p.mb + p.mh + p.mr;

%% Initial conditions
z0 = 0.31567;
wr0 = vx/tpr.DIMENSION.UNLOADED_RADIUS;
wf0 = vx/tpf.DIMENSION.UNLOADED_RADIUS;