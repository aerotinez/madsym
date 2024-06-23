function Hsharp = sharpMotorcycleLinearizedForcingMatrix(V,in2)
%sharpMotorcycleLinearizedForcingMatrix
%    Hsharp = sharpMotorcycleLinearizedForcingMatrix(V,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    23-Jun-2024 20:38:09

C_delta = in2(1,:);
C_f1 = in2(2,:);
C_f2 = in2(3,:);
C_r1 = in2(4,:);
C_r2 = in2(5,:);
Z_f = in2(11,:);
a = in2(12,:);
a_n = in2(13,:);
b = in2(14,:);
e = in2(15,:);
g = in2(16,:);
h = in2(17,:);
i_fy = in2(18,:);
i_ry = in2(19,:);
j = in2(20,:);
k = in2(21,:);
m_f = in2(22,:);
m_r = in2(23,:);
r_f = in2(24,:);
r_r = in2(25,:);
varepsilon = in2(27,:);
t2 = cos(varepsilon);
t3 = sin(varepsilon);
t4 = Z_f.*a_n;
t5 = e.*g.*m_f;
t6 = -V;
t7 = -a_n;
t8 = 1.0./r_f;
t9 = 1.0./r_r;
t10 = -t4;
t11 = 1.0./t2;
t12 = a+t7;
t13 = V.*i_fy.*t8;
t14 = V.*i_ry.*t9;
t15 = t2.*t13;
t16 = t3.*t13;
t17 = t5+t10;
Hsharp = reshape([0.0,0.0,0.0,0.0,g.*(h.*m_r+j.*m_f),t17,C_r2.*V,C_f2.*V,0.0,0.0,0.0,0.0,t17,-t3.*(t4-t5),0.0,V.*(C_f1.*t2+C_f2.*t3),0.0,0.0,0.0,0.0,0.0,0.0,-C_r1,-C_f1,0.0,0.0,t6.*(m_f+m_r),k.*m_f.*t6,h.*m_r.*t6+j.*m_f.*t6+i_fy.*t6.*t8+i_ry.*t6.*t9,e.*m_f.*t6+i_fy.*t3.*t6.*t8,C_r1.*b,-C_f1.*t11.*t12,1.0,0.0,0.0,t13+t14,0.0,t15,0.0,0.0,0.0,1.0,0.0,t16,i_fy.*t2.*t6.*t8,-C_delta,0.0,C_f1.*a_n,0.0,0.0,1.0,-b,0.0,0.0,t6,0.0,0.0,0.0,1.0,t11.*t12,0.0,t7,0.0,t6],[8,8]);
