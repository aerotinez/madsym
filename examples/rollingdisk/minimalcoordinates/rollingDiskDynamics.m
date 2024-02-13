function fd = rollingDiskDynamics(in1,in2,in3,in4)
%rollingDiskDynamics
%    FD = rollingDiskDynamics(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    13-Feb-2024 01:55:08

g = in4(1,:);
m = in4(2,:);
r = in4(3,:);
tau_x = in3(2,:);
tau_y = in3(1,:);
u_1 = in2(1,:);
u_2 = in2(2,:);
u_3 = in2(3,:);
varphi = in1(4,:);
t2 = cos(varphi);
t3 = sin(varphi);
t4 = r.^2;
t5 = 1.0./m;
t6 = 1.0./t4;
t7 = 1.0./t2;
fd = [t7.*u_2.*u_3.*-2.0;(g.*t3.*(4.0./5.0))./r+t2.*t3.*u_1.^2+t5.*t6.*tau_x.*(4.0./5.0)+t2.*u_1.*u_3.*(6.0./5.0);(t5.*t6.*t7.*(t2.*tau_y.*2.0+m.*t3.*t4.*u_2.*u_3.*6.0-m.*t4.*1.0./t7.^2.*u_1.*u_2.*5.0))./3.0];
