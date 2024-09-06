function Fp = predictionJacobian(in1,in2,in3)
%predictionJacobian
%    Fp = predictionJacobian(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    06-Sep-2024 21:32:09

a = in3(1,:);
b = in3(2,:);
delta = in1(4,:);
e = in3(4,:);
f = in3(5,:);
omega_x = in2(1,:);
omega_y = in2(2,:);
omega_z = in2(3,:);
phi = in1(3,:);
theta = in1(2,:);
v_x = in2(4,:);
v_y = in2(5,:);
v_z = in2(6,:);
varepsilon = in3(3,:);
t2 = cos(delta);
t3 = cos(phi);
t4 = sin(delta);
t5 = cos(theta);
t6 = cos(varepsilon);
t7 = sin(phi);
t8 = sin(theta);
t9 = sin(varepsilon);
t10 = varepsilon.*2.0;
t11 = t3.^2;
t12 = t3.^3;
t14 = t5.^2;
t15 = t6.^2;
t16 = sin(t10);
t17 = b.*t6;
t18 = 1.0./t5;
t21 = 1.0./t7;
t13 = t11.^2;
t19 = 1.0./t14;
t20 = t18.^3;
t22 = a+t17;
t23 = t11-1.0;
t24 = 1.0./t22;
t25 = 1.0./t23;
et1 = t2.*t3.*t15.*v_x.*-2.0+t3.*t4.*t15.*v_y.*2.0+t3.*t6.*t9.*v_z.*2.0+a.*omega_y.*t2.*t5.*t8+a.*omega_x.*t4.*t5.*t8+e.*omega_y.*t3.*t6.*t9.*2.0-e.*omega_z.*t3.*t4.*t15.*2.0+f.*omega_y.*t2.*t3.*t15.*2.0+f.*omega_x.*t3.*t4.*t15.*2.0+omega_y.*t2.*t5.*t8.*t17+omega_x.*t4.*t5.*t8.*t17+t2.*t3.*t14.*t15.*v_x-t3.*t4.*t14.*t15.*v_y-t3.*t6.*t9.*t14.*v_z+t5.*t8.*t11.*t15.*v_z-e.*omega_y.*t3.*t6.*t9.*t14+e.*omega_z.*t3.*t4.*t14.*t15+e.*omega_y.*t5.*t8.*t11.*t15-f.*omega_y.*t2.*t3.*t14.*t15-f.*omega_x.*t3.*t4.*t14.*t15+a.*omega_y.*t2.*t3.*t6.*t9.*2.0+a.*omega_x.*t3.*t4.*t6.*t9.*2.0-a.*omega_y.*t2.*t5.*t8.*t11-a.*omega_x.*t4.*t5.*t8.*t11-a.*omega_y.*t2.*t3.*t6.*t9.*t14-a.*omega_x.*t3.*t4.*t6.*t9.*t14;
et2 = a.*omega_y.*t2.*t5.*t8.*t11.*t15+a.*omega_x.*t4.*t5.*t8.*t11.*t15+t3.*t4.*t5.*t6.*t7.*t8.*v_x+t2.*t5.*t6.*t8.*t9.*t11.*v_x+t2.*t3.*t5.*t6.*t7.*t8.*v_y-t4.*t5.*t6.*t8.*t9.*t11.*v_y+a.*omega_x.*t2.*t3.*t5.*t7.*t8.*t9-a.*omega_y.*t3.*t4.*t5.*t7.*t8.*t9-e.*omega_z.*t2.*t3.*t5.*t6.*t7.*t8+e.*omega_z.*t4.*t5.*t6.*t8.*t9.*t11+f.*omega_x.*t2.*t3.*t5.*t6.*t7.*t8-f.*omega_y.*t3.*t4.*t5.*t6.*t7.*t8-f.*omega_y.*t2.*t5.*t6.*t8.*t9.*t11-f.*omega_x.*t4.*t5.*t6.*t8.*t9.*t11;
et3 = t4.*t5.*t6.*v_x+t2.*t5.*t6.*v_y+a.*omega_x.*t2.*t5.*t9-a.*omega_y.*t4.*t5.*t9-e.*omega_z.*t2.*t5.*t6+f.*omega_x.*t2.*t5.*t6-f.*omega_y.*t4.*t5.*t6-t4.*t5.*t6.*t11.*v_x.*2.0+t4.*t5.*t6.*t13.*v_x-t2.*t7.*t8.*t15.*v_x-t2.*t5.*t6.*t11.*v_y.*2.0+t2.*t5.*t6.*t13.*v_y+t4.*t7.*t8.*t15.*v_y+t3.*t5.*t7.*t15.*v_z.*2.0+t6.*t7.*t8.*t9.*v_z-t5.*t7.*t12.*t15.*v_z+e.*omega_z.*t2.*t5.*t6.*t11.*2.0-e.*omega_z.*t2.*t5.*t6.*t13+e.*omega_y.*t3.*t5.*t7.*t15.*2.0+e.*omega_y.*t6.*t7.*t8.*t9-e.*omega_z.*t4.*t7.*t8.*t15-e.*omega_y.*t5.*t7.*t12.*t15-f.*omega_x.*t2.*t5.*t6.*t11.*2.0+f.*omega_x.*t2.*t5.*t6.*t13+f.*omega_y.*t4.*t5.*t6.*t11.*2.0-f.*omega_y.*t4.*t5.*t6.*t13+f.*omega_y.*t2.*t7.*t8.*t15;
et4 = f.*omega_x.*t4.*t7.*t8.*t15+omega_y.*t2.*t3.*t5.*t7.*t17+omega_x.*t3.*t4.*t5.*t7.*t17-a.*omega_y.*t2.*t3.*t5.*t7-a.*omega_x.*t3.*t4.*t5.*t7-a.*omega_x.*t2.*t5.*t9.*t11.*2.0+a.*omega_y.*t2.*t5.*t7.*t12+a.*omega_x.*t4.*t5.*t7.*t12+a.*omega_x.*t2.*t5.*t9.*t13+a.*omega_y.*t4.*t5.*t9.*t11.*2.0-a.*omega_y.*t4.*t5.*t9.*t13+a.*omega_y.*t2.*t3.*t5.*t7.*t15.*2.0+a.*omega_y.*t2.*t6.*t7.*t8.*t9+a.*omega_x.*t3.*t4.*t5.*t7.*t15.*2.0+a.*omega_x.*t4.*t6.*t7.*t8.*t9-a.*omega_y.*t2.*t5.*t7.*t12.*t15-a.*omega_x.*t4.*t5.*t7.*t12.*t15+t2.*t3.*t5.*t6.*t7.*t9.*v_x.*2.0-t2.*t5.*t6.*t7.*t9.*t12.*v_x-t3.*t4.*t5.*t6.*t7.*t9.*v_y.*2.0+t4.*t5.*t6.*t7.*t9.*t12.*v_y+e.*omega_z.*t3.*t4.*t5.*t6.*t7.*t9.*2.0-e.*omega_z.*t4.*t5.*t6.*t7.*t9.*t12;
et5 = f.*omega_y.*t2.*t3.*t5.*t6.*t7.*t9.*-2.0-f.*omega_x.*t3.*t4.*t5.*t6.*t7.*t9.*2.0+f.*omega_y.*t2.*t5.*t6.*t7.*t9.*t12+f.*omega_x.*t4.*t5.*t6.*t7.*t9.*t12;
et6 = -a.*omega_x.*t2.*t5+a.*omega_y.*t4.*t5-omega_x.*t2.*t5.*t17+omega_y.*t4.*t5.*t17+a.*omega_x.*t2.*t5.*t11-a.*omega_y.*t4.*t5.*t11-t3.*t4.*t8.*t15.*v_x-t2.*t3.*t8.*t15.*v_y+e.*omega_z.*t2.*t3.*t8.*t15-f.*omega_x.*t2.*t3.*t8.*t15+f.*omega_y.*t3.*t4.*t8.*t15-t2.*t3.*t5.*t6.*t7.*v_x+t4.*t5.*t6.*t9.*t11.*v_x+t3.*t4.*t5.*t6.*t7.*v_y+t2.*t5.*t6.*t9.*t11.*v_y-a.*omega_x.*t2.*t5.*t11.*t15+a.*omega_y.*t4.*t5.*t11.*t15+a.*omega_y.*t2.*t3.*t5.*t7.*t9-a.*omega_x.*t2.*t3.*t6.*t8.*t9+a.*omega_x.*t3.*t4.*t5.*t7.*t9+a.*omega_y.*t3.*t4.*t6.*t8.*t9-e.*omega_z.*t3.*t4.*t5.*t6.*t7-e.*omega_z.*t2.*t5.*t6.*t9.*t11+f.*omega_y.*t2.*t3.*t5.*t6.*t7+f.*omega_x.*t3.*t4.*t5.*t6.*t7;
et7 = f.*omega_x.*t2.*t5.*t6.*t9.*t11-f.*omega_y.*t4.*t5.*t6.*t9.*t11;
et8 = t2.*t5.*t6.*v_x-t4.*t5.*t6.*v_y-a.*omega_y.*t2.*t5.*t9-a.*omega_x.*t4.*t5.*t9+e.*omega_z.*t4.*t5.*t6-f.*omega_y.*t2.*t5.*t6-f.*omega_x.*t4.*t5.*t6-t2.*t5.*t6.*t11.*v_x+t4.*t7.*t8.*t15.*v_x+t4.*t5.*t6.*t11.*v_y+t2.*t7.*t8.*t15.*v_y-e.*omega_z.*t4.*t5.*t6.*t11-e.*omega_z.*t2.*t7.*t8.*t15+f.*omega_y.*t2.*t5.*t6.*t11+f.*omega_x.*t4.*t5.*t6.*t11+f.*omega_x.*t2.*t7.*t8.*t15-f.*omega_y.*t4.*t7.*t8.*t15-a.*omega_x.*t2.*t3.*t5.*t7+a.*omega_y.*t3.*t4.*t5.*t7+a.*omega_y.*t2.*t5.*t9.*t11+a.*omega_x.*t4.*t5.*t9.*t11+a.*omega_x.*t2.*t3.*t5.*t7.*t15+a.*omega_x.*t2.*t6.*t7.*t8.*t9-a.*omega_y.*t3.*t4.*t5.*t7.*t15-a.*omega_y.*t4.*t6.*t7.*t8.*t9-t3.*t4.*t5.*t6.*t7.*t9.*v_x;
et9 = -t2.*t3.*t5.*t6.*t7.*t9.*v_y+e.*omega_z.*t2.*t3.*t5.*t6.*t7.*t9-f.*omega_x.*t2.*t3.*t5.*t6.*t7.*t9+f.*omega_y.*t3.*t4.*t5.*t6.*t7.*t9;
et10 = -t5.*v_z-e.*omega_y.*t5+t5.*t15.*v_z+e.*omega_y.*t5.*t15+omega_y.*t2.*t5.*t17+omega_x.*t4.*t5.*t17+t5.*t11.*t15.*v_z-a.*omega_y.*t2.*t5.*t11-a.*omega_x.*t4.*t5.*t11+a.*omega_y.*t2.*t5.*t15+a.*omega_x.*t4.*t5.*t15+e.*omega_y.*t5.*t11.*t15+t2.*t5.*t6.*t9.*v_x-t2.*t3.*t8.*t15.*v_x.*2.0-t4.*t5.*t6.*t9.*v_y+t3.*t4.*t8.*t15.*v_y.*2.0+t3.*t6.*t8.*t9.*v_z.*2.0+e.*omega_z.*t4.*t5.*t6.*t9+e.*omega_y.*t3.*t6.*t8.*t9.*2.0-e.*omega_z.*t3.*t4.*t8.*t15.*2.0-f.*omega_y.*t2.*t5.*t6.*t9-f.*omega_x.*t4.*t5.*t6.*t9+f.*omega_y.*t2.*t3.*t8.*t15.*2.0+f.*omega_x.*t3.*t4.*t8.*t15.*2.0+t3.*t4.*t5.*t6.*t7.*v_x+t2.*t5.*t6.*t9.*t11.*v_x+t2.*t3.*t5.*t6.*t7.*v_y-t4.*t5.*t6.*t9.*t11.*v_y+a.*omega_y.*t2.*t5.*t11.*t15;
et11 = a.*omega_x.*t4.*t5.*t11.*t15+a.*omega_x.*t2.*t3.*t5.*t7.*t9+a.*omega_y.*t2.*t3.*t6.*t8.*t9.*2.0-a.*omega_y.*t3.*t4.*t5.*t7.*t9+a.*omega_x.*t3.*t4.*t6.*t8.*t9.*2.0-e.*omega_z.*t2.*t3.*t5.*t6.*t7+e.*omega_z.*t4.*t5.*t6.*t9.*t11+f.*omega_x.*t2.*t3.*t5.*t6.*t7-f.*omega_y.*t3.*t4.*t5.*t6.*t7-f.*omega_y.*t2.*t5.*t6.*t9.*t11-f.*omega_x.*t4.*t5.*t6.*t9.*t11;
et12 = t2.*t7.*t14.*v_x+t2.*t7.*t15.*v_x-t4.*t7.*t14.*v_y-t4.*t7.*t15.*v_y-t6.*t7.*t9.*v_z-e.*omega_y.*t6.*t7.*t9+e.*omega_z.*t4.*t7.*t14+e.*omega_z.*t4.*t7.*t15-f.*omega_y.*t2.*t7.*t14-f.*omega_x.*t4.*t7.*t14-f.*omega_y.*t2.*t7.*t15-f.*omega_x.*t4.*t7.*t15-t4.*t5.*t6.*t8.*v_x-t2.*t7.*t14.*t15.*v_x.*2.0-t2.*t5.*t6.*t8.*v_y+t4.*t7.*t14.*t15.*v_y.*2.0+t3.*t5.*t7.*t8.*v_z+t6.*t7.*t9.*t14.*v_z.*2.0+b.*omega_y.*t2.*t7.*t9.*t14+b.*omega_x.*t4.*t7.*t9.*t14+e.*omega_z.*t2.*t5.*t6.*t8+e.*omega_y.*t3.*t5.*t7.*t8+e.*omega_y.*t6.*t7.*t9.*t14.*2.0-e.*omega_z.*t4.*t7.*t14.*t15.*2.0-f.*omega_x.*t2.*t5.*t6.*t8+f.*omega_y.*t4.*t5.*t6.*t8+f.*omega_y.*t2.*t7.*t14.*t15.*2.0;
et13 = f.*omega_x.*t4.*t7.*t14.*t15.*2.0+t4.*t5.*t6.*t8.*t11.*v_x.*2.0-t4.*t5.*t6.*t8.*t13.*v_x+t2.*t5.*t6.*t8.*t11.*v_y.*2.0-t2.*t5.*t6.*t8.*t13.*v_y-t3.*t5.*t7.*t8.*t15.*v_z.*3.0+t5.*t7.*t8.*t12.*t15.*v_z-a.*omega_x.*t2.*t5.*t8.*t9-a.*omega_y.*t2.*t6.*t7.*t9-a.*omega_x.*t4.*t6.*t7.*t9+a.*omega_y.*t4.*t5.*t8.*t9+a.*omega_y.*t2.*t3.*t5.*t7.*t8.*2.0+a.*omega_x.*t3.*t4.*t5.*t7.*t8.*2.0+a.*omega_x.*t2.*t5.*t8.*t9.*t11.*2.0-a.*omega_y.*t2.*t5.*t7.*t8.*t12-a.*omega_x.*t4.*t5.*t7.*t8.*t12-a.*omega_x.*t2.*t5.*t8.*t9.*t13-a.*omega_y.*t4.*t5.*t8.*t9.*t11.*2.0+a.*omega_y.*t2.*t6.*t7.*t9.*t14.*2.0+a.*omega_x.*t4.*t6.*t7.*t9.*t14.*2.0+a.*omega_y.*t4.*t5.*t8.*t9.*t13-e.*omega_z.*t2.*t5.*t6.*t8.*t11.*2.0;
et14 = e.*omega_z.*t2.*t5.*t6.*t8.*t13-e.*omega_y.*t3.*t5.*t7.*t8.*t15.*3.0+e.*omega_y.*t5.*t7.*t8.*t12.*t15+f.*omega_x.*t2.*t5.*t6.*t8.*t11.*2.0-f.*omega_x.*t2.*t5.*t6.*t8.*t13-f.*omega_y.*t4.*t5.*t6.*t8.*t11.*2.0+f.*omega_y.*t4.*t5.*t6.*t8.*t13-omega_y.*t2.*t3.*t5.*t7.*t8.*t17-omega_x.*t3.*t4.*t5.*t7.*t8.*t17-t2.*t3.*t5.*t6.*t7.*t8.*t9.*v_x.*3.0+t2.*t5.*t6.*t7.*t8.*t9.*t12.*v_x+t3.*t4.*t5.*t6.*t7.*t8.*t9.*v_y.*3.0-t4.*t5.*t6.*t7.*t8.*t9.*t12.*v_y-a.*omega_y.*t2.*t3.*t5.*t7.*t8.*t15.*3.0-a.*omega_x.*t3.*t4.*t5.*t7.*t8.*t15.*3.0+a.*omega_y.*t2.*t5.*t7.*t8.*t12.*t15+a.*omega_x.*t4.*t5.*t7.*t8.*t12.*t15-e.*omega_z.*t3.*t4.*t5.*t6.*t7.*t8.*t9.*3.0+e.*omega_z.*t4.*t5.*t6.*t7.*t8.*t9.*t12+f.*omega_y.*t2.*t3.*t5.*t6.*t7.*t8.*t9.*3.0;
et15 = f.*omega_x.*t3.*t4.*t5.*t6.*t7.*t8.*t9.*3.0-f.*omega_y.*t2.*t5.*t6.*t7.*t8.*t9.*t12-f.*omega_x.*t4.*t5.*t6.*t7.*t8.*t9.*t12;
et16 = -t3.*t4.*t14.*v_x-t3.*t4.*t15.*v_x-t2.*t3.*t14.*v_y-t2.*t3.*t15.*v_y+b.*omega_y.*t2.*t7.*t14+b.*omega_x.*t4.*t7.*t14+e.*omega_z.*t2.*t3.*t14+e.*omega_z.*t2.*t3.*t15-f.*omega_x.*t2.*t3.*t14-f.*omega_x.*t2.*t3.*t15+f.*omega_y.*t3.*t4.*t14+f.*omega_y.*t3.*t4.*t15-omega_x.*t2.*t5.*t8.*t17+omega_y.*t4.*t5.*t8.*t17+t2.*t7.*t9.*t14.*v_x+t3.*t4.*t14.*t15.*v_x.*2.0+t2.*t3.*t14.*t15.*v_y.*2.0-t4.*t7.*t9.*t14.*v_y+b.*omega_x.*t2.*t3.*t9.*t14-b.*omega_y.*t3.*t4.*t9.*t14-e.*omega_z.*t2.*t3.*t14.*t15.*2.0+e.*omega_z.*t4.*t7.*t9.*t14-f.*omega_y.*t2.*t7.*t9.*t14+f.*omega_x.*t2.*t3.*t14.*t15.*2.0-f.*omega_x.*t4.*t7.*t9.*t14-f.*omega_y.*t3.*t4.*t14.*t15.*2.0+t4.*t5.*t6.*t8.*t9.*v_x;
et17 = t2.*t5.*t6.*t8.*t9.*v_y-a.*omega_x.*t2.*t3.*t6.*t9+a.*omega_y.*t3.*t4.*t6.*t9+a.*omega_x.*t2.*t5.*t8.*t11-a.*omega_y.*t4.*t5.*t8.*t11-a.*omega_x.*t2.*t5.*t8.*t15+a.*omega_y.*t2.*t6.*t7.*t14+a.*omega_x.*t4.*t6.*t7.*t14+a.*omega_y.*t4.*t5.*t8.*t15+a.*omega_x.*t2.*t3.*t6.*t9.*t14.*2.0-a.*omega_y.*t3.*t4.*t6.*t9.*t14.*2.0-a.*omega_x.*t2.*t5.*t8.*t11.*t15+a.*omega_y.*t4.*t5.*t8.*t11.*t15-e.*omega_z.*t2.*t5.*t6.*t8.*t9+f.*omega_x.*t2.*t5.*t6.*t8.*t9-f.*omega_y.*t4.*t5.*t6.*t8.*t9-t2.*t3.*t5.*t6.*t7.*t8.*v_x+t4.*t5.*t6.*t8.*t9.*t11.*v_x+t3.*t4.*t5.*t6.*t7.*t8.*v_y+t2.*t5.*t6.*t8.*t9.*t11.*v_y+a.*omega_y.*t2.*t3.*t5.*t7.*t8.*t9+a.*omega_x.*t3.*t4.*t5.*t7.*t8.*t9-e.*omega_z.*t3.*t4.*t5.*t6.*t7.*t8;
et18 = -e.*omega_z.*t2.*t5.*t6.*t8.*t9.*t11+f.*omega_y.*t2.*t3.*t5.*t6.*t7.*t8+f.*omega_x.*t3.*t4.*t5.*t6.*t7.*t8+f.*omega_x.*t2.*t5.*t6.*t8.*t9.*t11-f.*omega_y.*t4.*t5.*t6.*t8.*t9.*t11;
mt1 = [0.0,0.0,0.0,0.0,t20.*t21.*t24.*(et1+et2),-t19.*t24.*((t16.*v_z)./2.0+(e.*omega_y.*t16)./2.0-t2.*t15.*v_x+t4.*t15.*v_y-e.*omega_z.*t4.*t15+f.*omega_y.*t2.*t15+f.*omega_x.*t4.*t15+a.*omega_y.*t2.*t6.*t9+a.*omega_x.*t4.*t6.*t9),t20.*t21.*t24.*(et10+et11),-t19.*t21.*t24.*(t9.*v_z+e.*omega_y.*t9-t2.*t6.*v_x+t4.*t6.*v_y+a.*omega_y.*t2.*t9+a.*omega_x.*t4.*t9-e.*omega_z.*t4.*t6+f.*omega_y.*t2.*t6+f.*omega_x.*t4.*t6),t19.*t21.*t24.*t25.*(et3+et4+et5)];
mt2 = [-t21.*t24.*(-t15.*v_z+a.*omega_y.*t2+a.*omega_x.*t4-e.*omega_y.*t15+t11.*t15.*v_z-a.*omega_y.*t2.*t11-a.*omega_x.*t4.*t11-a.*omega_y.*t2.*t15-a.*omega_x.*t4.*t15+e.*omega_y.*t11.*t15-t2.*t6.*t9.*v_x+t4.*t6.*t9.*v_y+a.*omega_y.*t2.*t11.*t15+a.*omega_x.*t4.*t11.*t15-e.*omega_z.*t4.*t6.*t9+f.*omega_y.*t2.*t6.*t9+f.*omega_x.*t4.*t6.*t9+t3.*t4.*t6.*t7.*v_x+t2.*t6.*t9.*t11.*v_x+t2.*t3.*t6.*t7.*v_y-t4.*t6.*t9.*t11.*v_y-e.*omega_z.*t2.*t3.*t6.*t7+e.*omega_z.*t4.*t6.*t9.*t11+f.*omega_x.*t2.*t3.*t6.*t7-f.*omega_y.*t3.*t4.*t6.*t7-f.*omega_y.*t2.*t6.*t9.*t11-f.*omega_x.*t4.*t6.*t9.*t11+a.*omega_x.*t2.*t3.*t7.*t9-a.*omega_y.*t3.*t4.*t7.*t9)];
mt3 = [-t19.*t21.*t24.*t25.*(et12+et13+et14+et15),-t18.*t24.*t25.*(t5.*t6.*v_z+b.*omega_y.*t2.*t5+b.*omega_x.*t4.*t5+e.*omega_y.*t5.*t6+t2.*t5.*t9.*v_x-t4.*t5.*t9.*v_y+t3.*t8.*t9.*v_z+a.*omega_y.*t2.*t5.*t6+a.*omega_x.*t4.*t5.*t6+e.*omega_z.*t4.*t5.*t9+e.*omega_y.*t3.*t8.*t9-f.*omega_y.*t2.*t5.*t9-f.*omega_x.*t4.*t5.*t9-t2.*t3.*t6.*t8.*v_x+t3.*t4.*t6.*t8.*v_y-e.*omega_z.*t3.*t4.*t6.*t8+f.*omega_y.*t2.*t3.*t6.*t8+f.*omega_x.*t3.*t4.*t6.*t8+a.*omega_y.*t2.*t3.*t8.*t9+a.*omega_x.*t3.*t4.*t8.*t9),-t19.*t21.*t24.*(et6+et7),-t18.*t21.*t24.*(et8+et9),-t19.*t21.*t24.*(et16+et17+et18)];
mt4 = [-t18.*t21.*t24.*(t2.*t5.*t7.*v_x+t4.*t6.*t8.*v_x+t2.*t6.*t8.*v_y-t4.*t5.*t7.*v_y+a.*omega_x.*t2.*t8.*t9-a.*omega_y.*t4.*t8.*t9+b.*omega_x.*t2.*t3.*t5-b.*omega_y.*t3.*t4.*t5-e.*omega_z.*t2.*t6.*t8+e.*omega_z.*t4.*t5.*t7-f.*omega_y.*t2.*t5.*t7+f.*omega_x.*t2.*t6.*t8-f.*omega_x.*t4.*t5.*t7-f.*omega_y.*t4.*t6.*t8-t3.*t4.*t5.*t9.*v_x-t2.*t3.*t5.*t9.*v_y+b.*omega_y.*t2.*t5.*t7.*t9+b.*omega_x.*t4.*t5.*t7.*t9+e.*omega_z.*t2.*t3.*t5.*t9-f.*omega_x.*t2.*t3.*t5.*t9+f.*omega_y.*t3.*t4.*t5.*t9+a.*omega_x.*t2.*t3.*t5.*t6-a.*omega_y.*t3.*t4.*t5.*t6)];
Fp = reshape([mt1,mt2,mt3,mt4],4,4);
