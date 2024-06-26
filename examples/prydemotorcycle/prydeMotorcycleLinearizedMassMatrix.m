function Mr = prydeMotorcycleLinearizedMassMatrix(V,in2)
%prydeMotorcycleLinearizedMassMatrix
%    Mr = prydeMotorcycleLinearizedMassMatrix(V,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    26-Jun-2024 19:15:16

C_rxz = in2(6,:);
I_fx = in2(7,:);
I_fz = in2(8,:);
I_rx = in2(9,:);
I_rz = in2(10,:);
a = in2(11,:);
b = in2(12,:);
e = in2(15,:);
f = in2(16,:);
h = in2(18,:);
m_f = in2(21,:);
m_r = in2(22,:);
varepsilon = in2(27,:);
t2 = cos(varepsilon);
t3 = sin(varepsilon);
t4 = b.*m_f;
t5 = b.*m_r;
t6 = e.*m_f;
t7 = h.*m_r;
t8 = a.^2;
t9 = b.^2;
t10 = e.^2;
t11 = f.^2;
t12 = varepsilon.*2.0;
t13 = cos(t12);
t14 = t2.^2;
t15 = sin(t12);
t16 = t3.^2;
t17 = I_fz.*t2;
t18 = I_fz.*t3;
t19 = e.*t4;
t20 = h.*t5;
t21 = a.*m_f.*t2;
t22 = t2.*t6;
t23 = f.*m_f.*t2;
t24 = a.*m_f.*t3;
t25 = t3.*t6;
t26 = f.*m_f.*t3;
t28 = f.*t2.*t4;
t30 = a.*t3.*t4;
t27 = a.*t22;
t29 = f.*t22;
t31 = a.*t25;
t32 = t3.*t19;
t33 = f.*t25;
t34 = -t23;
t35 = e.*t22;
t36 = e.*t25;
t37 = -t28;
t39 = a.*f.*m_f.*t13;
t40 = f.*t6.*t13;
t41 = a.*t6.*t15;
t42 = a.*f.*m_f.*t15;
t43 = f.*t6.*t15;
t44 = (I_fx.*t15)./2.0;
t45 = (I_fz.*t15)./2.0;
t49 = (m_f.*t8.*t15)./2.0;
t50 = (e.*t6.*t15)./2.0;
t51 = (m_f.*t11.*t15)./2.0;
t54 = t4+t5+t21+t22+t26;
t38 = -t29;
t46 = -t39;
t47 = -t40;
t48 = -t44;
t52 = -t51;
t53 = t7+t24+t25+t34;
t56 = t17+t19+t27+t33+t35;
t55 = t18+t31+t36+t38;
t57 = C_rxz+t20+t30+t32+t37+t41+t45+t46+t47+t48+t49+t50+t52;
Mr = reshape([1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,m_f+m_r,t54,t53,t6,0.0,0.0,t54,I_rz+t42+t43+I_fx.*t16+I_fz.*t14+b.*t4+b.*t5+t2.*t19.*2.0+a.*t2.*t4.*2.0+a.*t6.*t14.*2.0+e.*t6.*t14+f.*t3.*t4.*2.0+m_f.*t8.*t14+m_f.*t11.*t16,t57,t56,0.0,0.0,t53,t57,I_rx-t42-t43+I_fx.*t14+I_fz.*t16+h.*t7+a.*t6.*t16.*2.0+e.*t6.*t16+m_f.*t8.*t16+m_f.*t11.*t14,t55,0.0,0.0,t6,t56,t55,I_fz+e.*t6],[6,6]);
