close("all"); clear; clc;
setmadsympath();

%% Setup
bs = bigSportsParameters();
p = bikeSimToPrydeV2Parameters(bs,130/3.6);

s = linspace(-pi,pi,36)';
cs = unique(p.tr*[cos(s),sin(s)] + [p.Rr,p.tr],"rows","stable");

lb = p.p*cos(p.caster) + (p.Rf - p.Rr)*sin(p.caster);
lh = p.p*sin(p.caster) - (p.Rf - p.Rr)*cos(p.caster);