function out1 = roadRelativeKinematics(in1,in2,kappa)
%roadRelativeKinematics
%    OUT1 = roadRelativeKinematics(IN1,IN2,KAPPA)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    25-Feb-2024 21:56:18

%    states = [s d xi]
%    inputs = [v_x v_y omega_psi]
%    params = [kappa]
%
d = in1(2,:);
omega_psi = in2(3,:);
v_x = in2(1,:);
v_y = in2(2,:);
xi = in1(3,:);
t2 = cos(xi);
t3 = sin(xi);
t4 = d.*kappa;
t5 = t2.*v_x;
t6 = t3.*v_y;
t7 = t4-1.0;
t8 = -t6;
t9 = 1.0./t7;
t10 = t5+t8;
out1 = [-t9.*t10;t3.*v_x+t2.*v_y;omega_psi+kappa.*t9.*t10];
