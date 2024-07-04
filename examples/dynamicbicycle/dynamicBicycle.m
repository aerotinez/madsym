function out1 = dynamicBicycle(in1,in2,in3)
%dynamicBicycle
%    OUT1 = dynamicBicycle(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    04-Jul-2024 17:57:08

%    states = [x y psi v_x v_y omega_z delta]
%    inputs = [a_x omega_delta F_yf F_yr]
%    params = [I_zz l_f l_r m]
%
F_yf = in2(3,:);
F_yr = in2(4,:);
I_zz = in3(1,:);
a_x = in2(1,:);
delta = in1(7,:);
l_f = in3(2,:);
l_r = in3(3,:);
m = in3(4,:);
omega_z = in1(6,:);
omega_delta = in2(2,:);
psi = in1(3,:);
v_x = in1(4,:);
v_y = in1(5,:);
t2 = cos(delta);
t3 = cos(psi);
t4 = sin(psi);
t5 = 1.0./m;
out1 = [t3.*v_x-t4.*v_y;t4.*v_x+t3.*v_y;omega_z;t5.*(a_x.*m-F_yf.*sin(delta).*2.0+m.*omega_z.*v_y);t5.*(F_yr.*2.0+F_yf.*t2.*2.0-m.*omega_z.*v_x);-(F_yr.*l_r.*2.0-F_yf.*l_f.*t2.*2.0)./I_zz;omega_delta];
