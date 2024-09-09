function Hp = motorcycleKinematicsMeasurementJacobian(in1,in2)
%measurementJacobian
%    Hp = measurementJacobian(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    08-Sep-2024 23:22:31

delta = in1(4,:);
psi = in1(1,:);
roll = in1(3,:);
theta = in1(2,:);
varepsilon = in2(3,:);
t2 = cos(delta);
t3 = cos(psi);
t4 = cos(roll);
t5 = sin(delta);
t6 = cos(theta);
t7 = cos(varepsilon);
t8 = sin(psi);
t9 = sin(roll);
t10 = sin(theta);
t11 = sin(varepsilon);
t12 = t10.*t11;
t13 = t2.*t3.*t4;
t14 = t2.*t4.*t8;
t15 = t3.*t4.*t5;
t16 = t2.*t6.*t9;
t17 = t4.*t6.*t7;
t18 = t2.*t7.*t10;
t19 = t4.*t5.*t8;
t20 = t5.*t6.*t9;
t21 = t5.*t7.*t10;
t22 = t2.*t3.*t6.*t7;
t23 = t2.*t6.*t7.*t8;
t24 = t3.*t5.*t6.*t7;
t25 = t2.*t4.*t6.*t11;
t26 = t2.*t3.*t9.*t10;
t27 = t2.*t3.*t9.*t11;
t28 = t5.*t6.*t7.*t8;
t29 = t4.*t5.*t6.*t11;
t30 = t2.*t8.*t9.*t10;
t31 = t3.*t5.*t9.*t10;
t32 = t2.*t8.*t9.*t11;
t33 = t3.*t5.*t9.*t11;
t36 = t5.*t8.*t9.*t10;
t38 = t5.*t8.*t9.*t11;
t34 = -t14;
t35 = -t15;
t37 = -t18;
t39 = t12.*t13;
t40 = t12.*t14;
t41 = t12.*t15;
t42 = -t23;
t43 = -t24;
t44 = t12.*t19;
t45 = -t29;
t46 = -t36;
t47 = -t38;
t48 = t12+t17;
t49 = t12.*t34;
t50 = t12.*t35;
t51 = t16+t21+t45;
t52 = t20+t25+t37;
t53 = t26+t34+t43+t47+t50;
t54 = t27+t35+t42+t46+t49;
mt1 = [t54,-t19+t22+t31+t32+t39,0.0,-t13+t28-t30-t33+t44,t53,0.0,t3.*t7.*t9+t6.*t8.*t11-t4.*t7.*t8.*t10,-t3.*t6.*t11+t7.*t8.*t9+t3.*t4.*t7.*t10,0.0,t3.*t52,t8.*t52,-t2.*t6.*t7-t2.*t4.*t12-t5.*t9.*t10,t3.*t51,t8.*t51,t5.*t6.*t7-t2.*t9.*t10+t4.*t5.*t12,t3.*t48,t8.*t48,t6.*t11-t4.*t7.*t10,t10.*t15+t11.*t14+t5.*t8.*t9-t2.*t3.*t9.*t12,-t11.*t13+t10.*t19-t3.*t5.*t9-t2.*t8.*t9.*t12,t6.*(t4.*t5-t2.*t9.*t11),t10.*t13-t11.*t19+t2.*t8.*t9+t3.*t5.*t9.*t12];
mt2 = [t10.*t14+t11.*t15-t2.*t3.*t9+t5.*t8.*t9.*t12,t6.*(t2.*t4+t5.*t9.*t11),t4.*t7.*t8-t3.*t7.*t9.*t10,-t3.*t4.*t7-t7.*t8.*t9.*t10,-t6.*t7.*t9,t53,t13-t28+t30+t33-t44,t51,t19-t22-t31-t32-t39,t54,t18-t20-t25,0.0,0.0,0.0];
Hp = reshape([mt1,mt2],9,4);
