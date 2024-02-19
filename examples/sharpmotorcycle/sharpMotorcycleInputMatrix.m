function out1 = sharpMotorcycleInputMatrix(in1,in2)
%sharpMotorcycleInputMatrix
%    OUT1 = sharpMotorcycleInputMatrix(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    19-Feb-2024 01:10:21

%    states = [x y psi varphi delta theta_r psi_f varphi_f theta_f u_1 u_2 u_3 u_4 u_5 Y_r Y_f s d xi]
%    params = [C_delta C_f1 C_f2 C_r1 C_r2 C_rxz I_fx I_fz I_rx I_rz Z_f a a_n b e f g h i_fy i_ry kappa m_f m_r r_f r_r sigma varepsilon]
%
C_rxz = in2(6,:);
I_fx = in2(7,:);
I_fz = in2(8,:);
I_rx = in2(9,:);
I_rz = in2(10,:);
a = in2(12,:);
e = in2(15,:);
f = in2(16,:);
h = in2(18,:);
m_f = in2(22,:);
m_r = in2(23,:);
varepsilon = in2(27,:);
t2 = cos(varepsilon);
t3 = sin(varepsilon);
t4 = h.*m_r;
t5 = m_f+m_r;
t6 = e.^2;
t7 = h.^2;
t8 = m_f.^2;
t9 = t2.^2;
t10 = t3.^2;
t11 = I_fz.*t2;
t12 = I_fz.*t3;
t13 = a.*t2;
t14 = e.*t2;
t15 = f.*t2;
t16 = a.*t3;
t17 = e.*t3;
t18 = f.*t3;
t19 = m_f.*t6;
t20 = h.*t4;
t21 = I_fx.*t2.*t3;
t22 = t3.*t11;
t23 = -t15;
t24 = I_fx.*t9;
t25 = I_fz.*t9;
t26 = I_fx.*t10;
t27 = I_fz.*t10;
t28 = I_fz+t19;
t29 = -t21;
t30 = t13+t14+t18;
t31 = t16+t17+t23;
t32 = t30.^2;
t33 = e.*m_f.*t30;
t34 = m_f.*t31;
t35 = m_f.*t32;
t36 = t31.^2;
t39 = t11+t33;
t37 = e.*t34;
t38 = t31.*t34;
t40 = t4+t34;
t43 = t39.^2;
t45 = t30.*t34;
t46 = I_rz+t25+t26+t35;
t41 = t40.^2;
t42 = t12+t37;
t47 = I_rx+t20+t24+t27+t38;
t50 = C_rxz+t22+t29+t45;
t44 = t42.^2;
t51 = t41.*t43;
t52 = t50.^2;
t58 = t28.*t41.*t46;
t59 = t8.*t28.*t32.*t47;
t60 = t5.*t43.*t47;
t61 = t6.*t8.*t46.*t47;
t62 = e.*t8.*t30.*t39.*t47.*2.0;
t63 = e.*m_f.*t40.*t42.*t46.*2.0;
t66 = t5.*t28.*t46.*t47;
t67 = m_f.*t30.*t39.*t40.*t42.*2.0;
t69 = e.*t8.*t30.*t42.*t50.*2.0;
t70 = m_f.*t28.*t30.*t40.*t50.*2.0;
t72 = e.*m_f.*t39.*t40.*t50.*2.0;
t73 = t5.*t39.*t42.*t50.*2.0;
t48 = t8.*t32.*t44;
t53 = -t51;
t54 = t6.*t8.*t52;
t56 = t5.*t28.*t52;
t57 = t5.*t44.*t46;
t64 = -t62;
t65 = -t63;
t68 = -t66;
t71 = -t70;
t74 = -t73;
t49 = -t48;
t55 = -t54;
t75 = t49+t53+t55+t56+t57+t58+t59+t60+t61+t64+t65+t67+t68+t69+t71+t72+t74;
t76 = 1.0./t75;
out1 = [0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;-t76.*(e.*m_f.*t52+t40.*t42.*t46-t39.*t40.*t50-e.*m_f.*t46.*t47+m_f.*t30.*t39.*t47-m_f.*t30.*t42.*t50);-t76.*(t39.*t41-t5.*t39.*t47+t5.*t42.*t50-e.*m_f.*t40.*t50+e.*t8.*t30.*t47-m_f.*t30.*t40.*t42);-t76.*(t8.*t32.*t42-t5.*t42.*t46+t5.*t39.*t50+e.*m_f.*t40.*t46-e.*t8.*t30.*t50-m_f.*t30.*t39.*t40);t76.*(t5.*t52+t41.*t46+t8.*t32.*t47-t5.*t46.*t47-m_f.*t30.*t40.*t50.*2.0);0.0;0.0;0.0;0.0;0.0];
