function Msharp = sharpMotorcycleLinearizedMassMatrix(V,in2)
%sharpMotorcycleLinearizedMassMatrix
%    Msharp = sharpMotorcycleLinearizedMassMatrix(V,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    24-Jun-2024 00:08:26

C_rxz = in2(6,:);
I_fx = in2(7,:);
I_fz = in2(8,:);
I_rx = in2(9,:);
I_rz = in2(10,:);
e = in2(15,:);
h = in2(17,:);
j = in2(20,:);
k = in2(21,:);
m_f = in2(22,:);
m_r = in2(23,:);
sigma = in2(26,:);
varepsilon = in2(27,:);
t2 = cos(varepsilon);
t3 = sin(varepsilon);
t4 = e.*m_f;
t5 = h.*m_r;
t6 = j.*m_f;
t7 = k.*m_f;
t8 = varepsilon.*2.0;
t9 = sin(t8);
t10 = t3.^2;
t11 = I_fz.*t2;
t12 = I_fz.*t3;
t13 = j.*t4;
t14 = k.*t4;
t15 = k.*t6;
t18 = t5+t6;
t16 = I_fx.*t10;
t17 = I_fz.*t10;
t19 = t11+t14;
t20 = t12+t13;
t21 = (I_fx.*t9)./2.0;
t22 = (I_fz.*t9)./2.0;
t23 = -t21;
t24 = C_rxz+t15+t22+t23;
Msharp = reshape([1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,m_f+m_r,t7,t18,t4,0.0,0.0,0.0,0.0,t7,I_fz+I_rz+t16-t17+k.*t7,t24,t19,0.0,0.0,0.0,0.0,t18,t24,I_fx+I_rx-t16+t17+h.*t5+j.*t6,t20,0.0,0.0,0.0,0.0,t4,t19,t20,I_fz+e.*t4,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,sigma,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,sigma],[8,8]);
