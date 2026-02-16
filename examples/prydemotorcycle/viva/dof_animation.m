close("all"); clear; clc;
setmadsympath;
dir = "C:\Users\marti\madsym\examples\prydemotorcycle\visualization\BigSports\STL\";

fig = animationCanvas;
axe = fig.Children;

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
rt = RearTire(stl_file,anim_params,geom_params);