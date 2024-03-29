function out1 = travelingInvertedPendulum(in1,in2,in3)
%travelingInvertedPendulum
%    OUT1 = travelingInvertedPendulum(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    17-Jan-2024 17:44:06

%    states = [x y psi varphi u_1 u_2 u_3]
%    inputs = [F_x M_z]
%    params = [I_xx I_yy I_zz g l m]
%
F_x = in2(1,:);
I_xx = in3(1,:);
I_yy = in3(2,:);
I_zz = in3(3,:);
M_z = in2(2,:);
g = in3(4,:);
l = in3(5,:);
m = in3(6,:);
psi = in1(3,:);
u_1 = in1(5,:);
u_2 = in1(6,:);
u_3 = in1(7,:);
varphi = in1(4,:);
t2 = cos(varphi);
t3 = sin(varphi);
t4 = l.^2;
t5 = m.^2;
t6 = u_2.^2;
t7 = varphi.*2.0;
t8 = sin(t7);
t9 = t3.^2;
t15 = l.*m.*t2.*u_2.*u_3.*2.0;
t10 = I_yy.*t9;
t11 = I_zz.*t9;
t12 = I_yy.*t8.*u_2.*u_3;
t13 = I_zz.*t8.*u_2.*u_3;
t16 = m.*t4.*t9;
t17 = -t15;
t19 = t4.*t5.*t9;
t21 = m.*t4.*t8.*u_2.*u_3;
t14 = -t11;
t18 = -t12;
t20 = F_x+t17;
t22 = -t19;
t23 = -t21;
t24 = I_zz+t10+t14+t16;
t26 = M_z+t13+t18+t23;
t25 = m.*t24;
t27 = t22+t25;
t28 = -1.0./(t19-t25);
out1 = [u_1.*cos(psi);u_1.*sin(psi);u_2;u_3;t20.*t24.*t28+(l.*m.*t3.*t26)./(t19-t25);m.*t26.*t28+(l.*m.*t3.*t20)./(t19-t25);((I_yy.*t6.*t8)./2.0-(I_zz.*t6.*t8)./2.0+g.*l.*m.*t3+(m.*t4.*t6.*t8)./2.0+l.*m.*t2.*u_1.*u_2)./(I_xx+m.*t4)];
