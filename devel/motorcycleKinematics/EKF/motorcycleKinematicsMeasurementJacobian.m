function Hp = motorcycleKinematicsmeasurementJacobian(in1,in2)
%motorcycleKinematicsmeasurementJacobian
%    Hp = motorcycleKinematicsmeasurementJacobian(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    09-Sep-2024 15:02:51

delta = in1(4,:);
psi = in1(1,:);
theta = in1(3,:);
varphi = in1(2,:);
varepsilon = in2(4,:);
t2 = cos(delta);
t3 = cos(psi);
t4 = sin(delta);
t5 = cos(theta);
t6 = cos(varphi);
t7 = cos(varepsilon);
t8 = sin(psi);
t9 = sin(theta);
t10 = sin(varphi);
t11 = sin(varepsilon);
t12 = -varepsilon;
t13 = t2.*t10;
t14 = t4.*t10;
t15 = t12+theta;
t16 = t5.*t8.*t11;
t17 = t7.*t8.*t9;
t19 = t2.*t3.*t6;
t20 = t2.*t6.*t8;
t21 = t3.*t4.*t6;
t22 = t4.*t6.*t8;
t23 = t3.*t5.*t11;
t24 = t3.*t7.*t9;
t26 = t2.*t3.*t5.*t7;
t27 = t2.*t5.*t7.*t8;
t28 = t3.*t4.*t5.*t7;
t29 = t2.*t5.*t6.*t11;
t30 = t2.*t6.*t7.*t9;
t31 = t2.*t3.*t9.*t11;
t32 = t4.*t5.*t7.*t8;
t33 = t3.*t5.*t7.*t10;
t34 = t4.*t5.*t6.*t11;
t35 = t4.*t6.*t7.*t9;
t36 = t2.*t8.*t9.*t11;
t37 = t3.*t4.*t9.*t11;
t38 = t5.*t7.*t8.*t10;
t41 = t4.*t8.*t9.*t11;
t42 = t3.*t9.*t10.*t11;
t44 = t8.*t9.*t10.*t11;
t18 = cos(t15);
t25 = -t17;
t39 = -t20;
t40 = -t21;
t43 = -t23;
t45 = t13.*t23;
t46 = t13.*t24;
t47 = t13.*t16;
t48 = t13.*t17;
t49 = t14.*t23;
t50 = t14.*t24;
t51 = -t27;
t52 = -t28;
t53 = -t30;
t54 = t14.*t16;
t55 = t14.*t17;
t56 = -t34;
t57 = -t36;
t58 = -t37;
t59 = -t46;
t60 = -t54;
t61 = t14+t29+t53;
t62 = t13+t35+t56;
t63 = t16+t25+t33+t42;
t64 = t24+t38+t43+t44;
t65 = t40+t45+t51+t57+t59;
t66 = t39+t52+t55+t58+t60;
Hp = reshape([t65,-t22+t26+t31+t47+t13.*t25,0.0,-t19+t32+t41+t50+t14.*t43,t66,0.0,t63,t64,0.0,t8.*t61,-t3.*t61,t4.*t6-t5.*t11.*t13+t7.*t9.*t13,t8.*t62,-t3.*t62,t2.*t6+t5.*t11.*t14-t7.*t9.*t14,t6.*t8.*t18,-t3.*t6.*t18,-t10.*t18,-t2.*t64,t2.*t63,-t2.*t6.*t18,t4.*t64,-t4.*t63,t4.*t6.*t18,t10.*t16+t10.*t25+t3.*t5.*t7+t3.*t9.*t11,t10.*t24+t10.*t43+t5.*t7.*t8+t8.*t9.*t11,-t6.*sin(t15),t66,t19-t32-t41+t49-t50,t62,t22-t26-t31-t47+t48,t65,-t14-t29+t30,0.0,0.0,0.0],[9,4]);
