close("all"); clear; clc;
setmadsympath();
moto_init;

mdl = "moto_linearization";
open_system(mdl);

opspec = operspec(mdl);
opspec.Inputs(1).Known(2:3) = true(2,1);
opspec.Inputs(1).Min(1:3) = zeros(3,1);

opspec.Outputs(1).Known([1:12,14:20,22:28]) = true(26,1);

opspec.Outputs(1).y([1:12,14:20,22:28]) = [
    zeros(9,1);
    p.p;
    zeros(6,1);
    vx;
    zeros(4,1);
    vx/p.Rf;
    0;
    vx;
    zeros(2,1)
    ];

opspec.Outputs(1).y(21) = vx/p.Rr;

opt = findopOptions(OptimizerType="graddescent-proj",DisplayReport="iter");
opt.OptimizationOptions.MaxFunEvals = 20000;
op = findop(mdl,opspec,opt);
