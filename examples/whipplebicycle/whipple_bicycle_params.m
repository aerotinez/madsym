close("all"); clear; clc;

I_bx = 9.2;
I_bxz = -2.4;
I_by = 11;
I_bz = 2.8;
I_fx = 0.1405;
I_fy = 0.28;
I_fz = I_fx;
I_hx = 0.05841337701;
I_hxz = -0.009119225262;
I_hy = 0.06;
I_hz = 0.007586623002;
I_rx = 0.0603;
I_ry = 0.12;
I_rz = I_rx;
b = 0.3;
e = 0.01;
f = 0.3;
g = 9.81;
h = 0.9;
m_b = 85;
m_f = 3;
m_h = 4;
m_r = 2;
p = 1.02;
r_f = 0.35; 
r_r = 0.3;
varepsilon = -pi/10;
d = (0.08 - r_f*tan(varepsilon))*cos(varepsilon);

params = [I_bx I_bxz I_by I_bz I_fx I_fy I_fz I_hx I_hxz I_hy I_hz I_rx I_ry I_rz b d e f g h m_b m_f m_h m_r p r_f r_r varepsilon].';

v = 80/3.6;

x0 = [
    zeros(6,1);
    p;
    zeros(7,1);
    -v/r_r
    ];