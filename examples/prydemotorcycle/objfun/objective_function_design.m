close('all'); clear; clc;
setmadsympath();

bike = bigSportsParameters();
params = bikeSimToPrydeParameters(bike,130/3.6);

Q = prydeMotorcycleSafetyCostFcn(params);
disp(cond(Q));
