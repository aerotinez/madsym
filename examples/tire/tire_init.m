close("all"); clear; clc;
setmadsympath();

r = 0.227;
t = 0.07;
m = 14.7;
Ixx = m*(5*r^2 + 4*t^2)/8;
Iyy = m*(4*r^2 + 3*t^2)/4;
Izz = Ixx;

tp = tireModel.new("MF");
tp.export('my_tire.tir','overwrite',true);
tire_params = simscape.multibody.tirread('my_tire.tir');
tire_params.MODEL.TYRESIDE = "Left";

