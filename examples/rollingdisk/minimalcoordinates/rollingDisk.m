function out1 = rollingDisk(in1,in2,in3)
%rollingDisk
%    OUT1 = rollingDisk(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    23-Jan-2024 02:21:14

%    states = [x y psi varphi theta u_1 u_2 u_3]
%    inputs = [tau_y tau_z]
%    params = [g m r]
%
g = in3(1,:);
m = in3(2,:);
psi = in1(3,:);
r = in3(3,:);
tau_y = in2(1,:);
tau_z = in2(2,:);
u_1 = in1(6,:);
u_2 = in1(7,:);
u_3 = in1(8,:);
varphi = in1(4,:);
t2 = cos(varphi);
t3 = sin(varphi);
t4 = m.^2;
t5 = r.^2;
t6 = t5.^2;
t7 = t3.^2;
t8 = t2.*tau_z;
t9 = t3.*tau_y;
t10 = (m.*t5)./4.0;
t11 = (m.*t2.*t5.*u_2.*u_3)./2.0;
t12 = m.*t2.*t5.*u_1.*u_2.*(5.0./2.0);
t19 = m.*t2.*t3.*t5.*u_1.*u_2.*(-5.0./2.0);
t13 = m.*t5.*t7.*(5.0./4.0);
t14 = -t11;
t15 = -t12;
t16 = t3.*t12;
t18 = t4.*t6.*t7.*(9.0./4.0);
t17 = t15+tau_y;
t20 = t10+t13;
t23 = t8+t9+t14+t19;
t21 = m.*t5.*t20.*(3.0./2.0);
t22 = -t21;
t24 = t18+t22;
t25 = 1.0./t24;
out1 = [r.*u_3.*cos(psi);r.*u_3.*sin(psi);u_1;u_2;u_3;m.*t5.*t23.*t25.*(-3.0./2.0)-m.*t3.*t5.*t25.*(t12-tau_y).*(3.0./2.0);((g.*m.*r.*t3+m.*t2.*t5.*u_1.*u_3.*(3.0./2.0)+m.*t2.*t3.*t5.*u_1.^2.*(5.0./4.0)).*(4.0./5.0))./(m.*t5);t20.*t25.*(t12-tau_y)+m.*t3.*t5.*t23.*t25.*(3.0./2.0)];