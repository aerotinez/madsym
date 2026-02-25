close('all'); clear; clc;
setmadsympath();

vx = 80/3.6;
pars = bikeSimToPrydeV2Parameters(bigSportsParameters(),vx);
p = cell2mat(struct2cell(pars));

sys = prydeMotoLateralStateSpace(p);
pzmap(sys);