function B = sharpMotorcycleInputMatrix(in1,in2)
%sharpMotorcycleInputMatrix
%    B = sharpMotorcycleInputMatrix(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    20-Sep-2024 19:35:53

%   states = [varphi delta omega_psi omega_varphi omega_delta v_y Y_r Y_f]
%   inputs = [tau_delta]
%   params = [C_delta C_f1 C_f2 C_r1 C_r2 C_rxz I_fx I_fz I_rx I_rz V Z_f a a_n b e g h i_fy i_ry j k m_f m_r r_f r_r sigma varepsilon]
%
C_rxz = in2(6,:);
I_fx = in2(7,:);
I_fz = in2(8,:);
I_rx = in2(9,:);
I_rz = in2(10,:);
e = in2(16,:);
h = in2(18,:);
j = in2(21,:);
k = in2(22,:);
m_f = in2(23,:);
m_r = in2(24,:);
varepsilon = in2(28,:);
t2 = cos(varepsilon);
t3 = sin(varepsilon);
t4 = h.*m_r;
t5 = j.*m_f;
t6 = m_f+m_r;
t7 = e.^2;
t8 = h.^2;
t9 = j.^2;
t10 = k.^2;
t11 = m_f.^2;
t12 = varepsilon.*2.0;
t18 = e.*k.*m_f;
t13 = sin(t12);
t14 = t3.^2;
t15 = I_fz.*t2;
t16 = I_fz.*t3;
t17 = e.*t5;
t19 = k.*t5;
t20 = m_f.*t7;
t21 = h.*t4;
t22 = j.*t5;
t23 = m_f.*t10;
t27 = t4+t5;
t24 = I_fx.*t14;
t25 = I_fz.*t14;
t26 = I_fz+t20;
t30 = t27.^2;
t31 = t15+t18;
t32 = t16+t17;
t33 = (I_fx.*t13)./2.0;
t34 = (I_fz.*t13)./2.0;
t28 = -t24;
t29 = -t25;
t35 = -t33;
t36 = t31.^2;
t37 = t32.^2;
t46 = k.*m_f.*t27.*t31.*t32.*2.0;
t38 = t10.*t11.*t37;
t40 = t30.*t36;
t41 = I_fz+I_rz+t23+t24+t29;
t43 = C_rxz+t19+t34+t35;
t44 = I_fx+I_rx+t21+t22+t25+t28;
t39 = -t38;
t42 = -t40;
t45 = t43.^2;
t49 = t6.*t37.*t41;
t51 = t26.*t30.*t41;
t52 = t10.*t11.*t26.*t44;
t53 = e.*k.*t11.*t32.*t43.*2.0;
t54 = t6.*t36.*t44;
t55 = e.*m_f.*t27.*t32.*t41.*2.0;
t56 = k.*m_f.*t26.*t27.*t43.*2.0;
t57 = e.*k.*t11.*t31.*t44.*2.0;
t61 = e.*m_f.*t27.*t31.*t43.*2.0;
t62 = t6.*t31.*t32.*t43.*2.0;
t64 = t7.*t11.*t41.*t44;
t65 = t6.*t26.*t41.*t44;
t47 = t7.*t11.*t45;
t50 = t6.*t26.*t45;
t58 = -t55;
t59 = -t56;
t60 = -t57;
t63 = -t62;
t66 = -t65;
t48 = -t47;
t67 = t39+t42+t46+t48+t49+t50+t51+t52+t53+t54+t58+t59+t60+t61+t63+t64+t66;
t68 = 1.0./t67;
B = [0.0;0.0;-t68.*(t30.*t31-t6.*t31.*t44+t6.*t32.*t43+e.*k.*t11.*t44-e.*m_f.*t27.*t43-k.*m_f.*t27.*t32);-t68.*(t10.*t11.*t32-t6.*t32.*t41+t6.*t31.*t43-e.*k.*t11.*t43+e.*m_f.*t27.*t41-k.*m_f.*t27.*t31);t68.*(t6.*t45+t30.*t41+t10.*t11.*t44-t6.*t41.*t44-k.*m_f.*t27.*t43.*2.0);-t68.*(e.*m_f.*t45+t27.*t32.*t41-t27.*t31.*t43-e.*m_f.*t41.*t44+k.*m_f.*t31.*t44-k.*m_f.*t32.*t43);0.0;0.0];
