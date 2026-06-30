close("all"); clear; clc;
setmadsympath();

r = 0.227;
t = 0.07;
m = 14.7;
Ixx = m*(5*r^2 + 4*t^2)/8;
Iyy = m*(4*r^2 + 3*t^2)/4;
Izz = Ixx;

bs = bigSportsParameters();
pr = struct2array(table2struct(bs.RearTire.Pacejka))';
tr = bs.RearTire.UndeflectedCrownRadius;
Lx = bs.RearTire.LongitudinalRelaxationLength;
Ly = bs.RearTire.LateralRelaxationLength;
Rr = bs.RearTire.EffectiveRollingRadius;
Vlow = 5/3.6;