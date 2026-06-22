close("all"); clear; clc;
setmadsympath();

%% Setup
bs = bigSportsParameters();
p = bikeSimToPrydeV2Parameters(bs,130/3.6);

s = linspace(-pi,pi,36)';
cs = unique(p.tr*[cos(s),sin(s)] + [p.Rr,p.tr],"rows","stable");