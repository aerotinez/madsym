function p = bikeSimToPrydeParameters(bikesim_params,vx)
    arguments
        bikesim_params (1,1) BikeSimMotorcycleParameters;
        vx (1,1) double {mustBePositive};
    end
    bs = bikesim_params;
    p = prydeMotorcycleParameters;

    % normal forces
    p.g = 9.80665;

    %% Geometric parameters
    p.tf = bs.FrontTire.UndeflectedCrownRadius;
    p.rf = bs.FrontTire.EffectiveRollingRadius - p.tf;
    p.tr = bs.RearTire.UndeflectedCrownRadius;
    p.rr = bs.RearTire.EffectiveRollingRadius - p.tr;
    p.caster = deg2rad(bs.SteeringHead.Caster);
    p.an = (p.rf + p.tf)*sin(p.caster) - bs.SteeringHead.Rake;
    p.v = vx;

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

    Oru = -ru.CoMOffset.*N(:,1) + ru.LeanAxisHeight.*N(:,3) + ru.CoMHeight.*Nru(:,3);

    rider_upper_body = body(Nru,Oru,Iru,ru.Mass);

    %% Swing arm
    sa = bs.SwingArm;

    Isa = zeros(3);
    
    Nsa = N*roty(-asind((sa.PivotHeight - sa.AxelHeight)/sa.Length));

    Osa = -sm.Wheelbase.*N(:,1) + sa.AxelHeight.*N(:,3) + sa.CoMOffset.*Nsa(:,1) + sa.CoMHeight.*Nsa(:,3);

    swing_arm = body(Nsa,Osa,Isa,sa.Mass);

    %% Rear tire
    rt = bs.RearTire;

    Irt = zeros(3);
    Irt(2,2) = rt.SpinInertia;
    p.iry = rt.SpinInertia;

    Ort = -sm.Wheelbase.*N(:,1) + rt.EffectiveRollingRadius*N(:,3);

    rear_tire = body(N,Ort,Irt,rt.Mass);

    %% Front tire
    ft = bs.FrontTire;

    Ift = zeros(3);
    Ift(2,2) = ft.SpinInertia;
    p.ify = ft.SpinInertia;

    Oft = ft.EffectiveRollingRadius.*N(:,3);

    front_tire = body(N,Oft,Ift,ft.Mass);

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

    p.Cdelta = rad2deg(sh.Damping);

    %% Fork
    ff = bs.Fork;

    Iff = zeros(3);

    Off = H + ff.CoMOffset.*Nsh(:,1) - ff.CoMHeight.*Nsh(:,3);

    fork = body(Nsh,Off,Iff,ff.Mass);

    %% Rear body

    rear_bodies = [
        sprung_mass
        rider_lower_body
        rider_upper_body
        swing_arm
        rear_tire
        ];

    % total mass
    p.mb = sum([rear_bodies.m]);

    % mass center
    Ob = sum([rear_bodies.O].*[rear_bodies.m],2)/p.mb;
    rb = Ob - [rear_bodies.O];
    p.b = sm.Wheelbase - abs(Ob(1));
    p.h = Ob(3);

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

    p.Ibxx = Ib(1,1);
    p.Ibxz = Ib(1,3);
    p.Ibzz = Ib(3,3);


    %% Front body

    front_bodies = [
        steering_head
        fork
        front_tire
        ];

    % total mass
    p.mh = sum([front_bodies.m]);

    % mass center
    Oh = sum([front_bodies.O].*[front_bodies.m],2)/p.mh;
    rf = Oh - [front_bodies.O];
    l = abs(Ob(1));
    p.a = l*cos(p.caster) + p.an;
    B = O - l.*N(:,1) + p.a.*Nsh(:,1);
    Gf = Nsh.'*(Oh - B);
    p.e = Gf(1);
    p.f = Gf(3);

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

    p.Ihxx = If(1,1);
    p.Ihzz = If(3,3);

    %% Tire parameters
    s = sin(p.caster);
    c = cos(p.caster);
    k = (p.a + p.e)*c + p.f*s;
    l = (p.a - p.an)/c;
    wb = p.b + l;

    %% Front tire parameters

    % Normal force
    fz = (p.g/wb).*[p.b + k, wb - l]*[p.mh;p.mb];
    fz0 = ft.Pacejka.fz0;

    % Longitudinal stiffness
    p.Ckf = 0;

    % Lateral sideslip stiffness
    pky1 = ft.Pacejka.pky1;
    pky2 = ft.Pacejka.pky2;
    pky3 = ft.Pacejka.pky3;
    p.Caf = -fz0*pky1*sin(pky2*atan(fz/(fz0*pky3)));

    % Lateral camber stiffness
    pky6 = ft.Pacejka.pky6;
    pky7 = ft.Pacejka.pky7;
    p.Cgf = fz*(pky6 - pky7) + (pky7*fz^2)/fz0;

    % Self-aligning sideslip stiffness
    R0 = ft.UndeflectedCrownRadius;
    qdz1 = ft.Pacejka.qdz1;
    qdz2 = ft.Pacejka.qdz2;
    ang = atan2(fz,fz0*pky3);
    p.Kaf = -R0*fz*pky1*sin(pky2*ang)*(fz0*(qdz1 - qdz2) + fz*qdz2)/fz0;

    % Self-aligning camber stiffness
    qdz8 = ft.Pacejka.qdz8;
    qdz9 = ft.Pacejka.qdz9;
    p.Kgf = R0*fz*(fz0*(qdz8 - qdz9) + fz*qdz9)/fz0;

    %% Rear tire parameters

    % Normal force
    fz = (p.g/wb).*[wb - p.b - k, l]*[p.mh;p.mb];
    fz0 = rt.Pacejka.fz0;

    % Longitudinal stiffness
    p.Ckr = 0;

    % Lateral sideslip stiffness
    pky1 = rt.Pacejka.pky1;
    pky2 = rt.Pacejka.pky2;
    pky3 = rt.Pacejka.pky3;
    p.Car = -fz0*pky1*sin(pky2*atan(fz/(fz0*pky3)));

    % Lateral camber stiffness
    pky6 = rt.Pacejka.pky6;
    pky7 = rt.Pacejka.pky7;
    p.Cgr = fz*(pky6 - pky7) + (pky7*fz^2)/fz0;

    % Self-aligning sideslip stiffness
    R0 = rt.UndeflectedCrownRadius;
    qdz1 = rt.Pacejka.qdz1;
    qdz2 = rt.Pacejka.qdz2;
    ang = atan2(fz,fz0*pky3);
    p.Kar = -R0*fz*pky1*sin(pky2*ang)*(fz0*(qdz1 - qdz2) + fz*qdz2)/fz0;

    % Self-aligning camber stiffness
    qdz8 = rt.Pacejka.qdz8;
    qdz9 = rt.Pacejka.qdz9;
    p.Kgr = R0*fz*(fz0*(qdz8 - qdz9) + fz*qdz9)/fz0;

end