close('all'); clear; clc;
setmadsympath();

bike = bigSportsParameters();
params = bikeSimToPrydeParameters(bike,130/3.6);

sys = prydeMotorcycleLateralStateSpace(cell2mat(struct2cell(params)));

A = sys.E\sys.A;
B = sys.E\sys.B;