function Msharp = extendedSharpMotorcycleLinearizedMassMatrix(V,in2)
%extendedSharpMotorcycleLinearizedMassMatrix
%    Msharp = extendedSharpMotorcycleLinearizedMassMatrix(V,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    14-Jun-2024 12:45:15

C_rxz = in2(6,:);
I_fx = in2(7,:);
I_fz = in2(8,:);
I_rx = in2(9,:);
I_rz = in2(10,:);
e = in2(15,:);
h = in2(17,:);
i_fy = in2(18,:);
i_ry = in2(19,:);
j = in2(20,:);
k = in2(21,:);
m_f = in2(22,:);
m_r = in2(23,:);
r_f = in2(24,:);
r_r = in2(25,:);
sigma = in2(26,:);
varepsilon = in2(27,:);
t2 = cos(varepsilon);
t3 = sin(varepsilon);
t4 = e.*m_f;
t5 = h.*m_r;
t6 = j.*m_f;
t7 = k.*m_f;
t8 = r_f.^2;
t9 = r_r.^2;
t10 = varepsilon.*2.0;
t11 = sin(t10);
t12 = t3.^2;
t13 = I_fz.*t2;
t14 = I_fz.*t3;
t15 = j.*t4;
t16 = k.*t4;
t17 = k.*t6;
t20 = t5+t6;
t18 = I_fx.*t12;
t19 = I_fz.*t12;
t21 = t13+t16;
t22 = t14+t15;
t23 = (I_fx.*t11)./2.0;
t24 = (I_fz.*t11)./2.0;
t25 = -t23;
t26 = C_rxz+t17+t24+t25;
Msharp = reshape([1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,m_f+m_r,t7,t20,t4,0.0,0.0,0.0,0.0,0.0,t7,I_fz+I_rz+t18-t19+k.*t7,t26,t21,0.0,0.0,0.0,0.0,0.0,t20,t26,I_fx+I_rx-t18+t19+h.*t5+j.*t6,t22,0.0,0.0,0.0,0.0,0.0,t4,t21,t22,I_fz+e.*t4,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,(i_fy.*t9+i_ry.*t8+m_f.*t8.*t9+m_r.*t8.*t9)./t8,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,sigma,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,sigma],[9,9]);