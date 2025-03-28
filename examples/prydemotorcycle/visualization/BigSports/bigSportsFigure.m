function out = bigSportsFigure()
    dir = "C:\Users\marti\madsym\examples\prydemotorcycle\visualization\BigSports\STL\";
    out = MotoFigure();

    %% Global geometric parameters
    geom_params = bikeSimGeometricParameters();
    geom_params.RearCrownRadius = 70E-03;
    geom_params.RearRadius = 290E-03;
    geom_params.Trim = 10;
    geom_params.SwingArmOffset = 70E-03;
    geom_params.Wheelbase = 1300E-03;
    geom_params.Caster = 24;
    geom_params.Rake = 26.5E-03;
    geom_params.ForkLength = 650E-03;
    geom_params.HandlebarOffset = 40E-03;
    geom_params.HandlebarHeight = 30E-03;
    geom_params.RiderLean = 45;
    geom_params.RiderWidth = 404E-03;
    geom_params.RiderDepth = 296E-03;
    geom_params.StepOffset = 940E-03;
    geom_params.StepHeight = 400E-03;
    geom_params.ThighLength = 500E-03;
    geom_params.ShinHeight = 500E-03;
    geom_params.LegHeight = 170E-03;
    geom_params.BackHeight = 500E-03;

    %% Rear tire
    stl_file = dir + "rear_tire";
    ang = [0,0,0];
    scale = 0.95.*[1,1,1];
    coord = [0,0,0];
    ref = [0,0,0];
    anim_params = bikeSimAnimatorParameters(ang,scale,coord,ref);
    out.RearTire = RearTire(stl_file,anim_params,geom_params);

    %% Swing arm
    stl_file = dir + "swing_arm";
    ang = [0,0,0];
    scale = [1,1,1];
    coord = [1300E-03,0,-5E-03];
    ref = [330E-03,283E-03,474E-03];
    anim_params = bikeSimAnimatorParameters(ang,scale,coord,ref);
    out.SwingArm = SwingArm(stl_file,anim_params,geom_params);

    %% Base
    stl_file = dir + "base";
    ang = [0,0,0];
    scale = [1,1,1];
    coord = [0,0,300E-03];
    ref = [1300E-03,525E-03,905E-03];
    anim_params = bikeSimAnimatorParameters(ang,scale,coord,ref);
    out.Base = BikeBase(stl_file,anim_params,geom_params);

    %% Body
    stl_file = dir + "body";
    ang = [0,0,0];
    scale = [1,1,1];
    coord = [0,0,300E-03];
    ref = [1300E-03,525E-03,905E-03];
    anim_params = bikeSimAnimatorParameters(ang,scale,coord,ref);
    out.Body = BikeBody(stl_file,anim_params,geom_params);
    
    %% Handlebars
    stl_file = dir + "handlebars_no_arms";
    ang = [0,0,0];
    scale = [1,1,1];
    coord = [30E-03,0,50E-03];
    ref = [153E-03,548E-03,169E-03];
    anim_params = bikeSimAnimatorParameters(ang,scale,coord,ref);
    out.Handlebars = Handlebars(stl_file,anim_params,geom_params);

    %% Fork top
    stl_file = dir + "fork_top";
    ang = [0,0,0];
    scale = [1,1,1];
    coord = [0,0,0];
    ref = [512E-03,275E-03,650E-03];
    anim_params = bikeSimAnimatorParameters(ang,scale,coord,ref);
    out.ForkTop = ForkTop(stl_file,anim_params,geom_params);

    %% Front fender
    stl_file = dir + "front_fender";
    ang = [0,1.5,0];
    scale = [1,1,1];
    coord = [0,0,-45E-03];
    ref = [512E-03,275E-03,585E-03];
    anim_params = bikeSimAnimatorParameters(ang,scale,coord,ref);
    out.FrontFender = Fork(stl_file,anim_params,geom_params);

    %% Fork
    stl_file = dir + "fork";
    ang = [0,0,0];
    scale = [1,1,1];
    coord = [0,0,0];
    ref = [512E-03,275E-03,585E-03];
    anim_params = bikeSimAnimatorParameters(ang,scale,coord,ref);
    out.Fork = Fork(stl_file,anim_params,geom_params);

    %% Front tire
    stl_file = dir + "front_tire";
    ang = [0,0,0];
    scale = [1,1,1];
    coord = [0,0,0];
    ref = [0,0,0];
    anim_params = bikeSimAnimatorParameters(ang,scale,coord,ref);
    out.FrontTire = FrontTire(stl_file,anim_params,geom_params);

    %% Left leg
    stl_file = dir + "driver_leg_left";
    ang = [3,6,0];
    scale = [1,1,1];
    coord = [-20E-03,0,0];
    ref = [500E-03,180E-03,500E-03];
    anim_params = bikeSimAnimatorParameters(ang,scale,coord,ref);
    out.LeftLeg = LeftLeg(stl_file,anim_params,geom_params);

    %% Right leg
    stl_file = dir + "driver_leg_right";
    ang = [-3,6,0];
    scale = [1,1,1];
    coord = [-20E-03,0,0];
    ref = [500E-03,180E-03,500E-03];
    anim_params = bikeSimAnimatorParameters(ang,scale,coord,ref);
    out.RightLeg = RightLeg(stl_file,anim_params,geom_params);

    %% Driver
    stl_file = dir + "driver";
    ang = [0,0,0];
    scale = [1,1,1];
    coord = [-85E-03,0,-85E-03];
    ref = [296E-03,404E-03,500E-03];
    anim_params = bikeSimAnimatorParameters(ang,scale,coord,ref);
    out.Driver = Driver(stl_file,anim_params,geom_params);

    %% Left arm
    stl_file = dir + "left_arm";
    ang = [0,0,0];
    scale = [1.05,1,1];
    coord = [-50E-03,0,10E-03];
    ref = [484E-03,180E-03,180E-03];
    anim_params = bikeSimAnimatorParameters(ang,scale,coord,ref);
    out.LeftArm = LeftArm(stl_file,anim_params,geom_params);

    %% Right arm
    stl_file = dir + "right_arm";
    ang = [0,0,0];
    scale = [1.05,1,1];
    coord = [-50E-03,0,10E-03];
    ref = [484E-03,180E-03,180E-03];
    anim_params = bikeSimAnimatorParameters(ang,scale,coord,ref);
    out.RightArm = RightArm(stl_file,anim_params,geom_params);

end