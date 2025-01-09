function H = sharpMotorcycleForcingMatrix(in1)
%sharpMotorcycleForcingMatrix
%    H = sharpMotorcycleForcingMatrix(IN1)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    28-Dec-2024 04:03:38

%   states = [varphi delta omega_psi v_y omega_varphi omega_delta Y_r Y_f]
%   inputs = [tau_delta]
%   params = [C_delta C_f1 C_f2 C_r1 C_r2 C_rxz I_fx I_fz I_rx I_rz V Z_f a a_n b e g h i_fy i_ry j k m_f m_r r_f r_r sigma varepsilon]
%
C_delta = in1(1,:);
C_f1 = in1(2,:);
C_f2 = in1(3,:);
C_r1 = in1(4,:);
C_r2 = in1(5,:);
V = in1(11,:);
Z_f = in1(12,:);
a = in1(13,:);
a_n = in1(14,:);
b = in1(15,:);
e = in1(16,:);
g = in1(17,:);
h = in1(18,:);
i_fy = in1(19,:);
i_ry = in1(20,:);
j = in1(21,:);
k = in1(22,:);
m_f = in1(23,:);
m_r = in1(24,:);
r_f = in1(25,:);
r_r = in1(26,:);
varepsilon = in1(28,:);
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
H = reshape([0.0,0.0,0.0,0.0,g.*(h.*m_r+j.*m_f),t17,C_r2.*V,C_f2.*V,0.0,0.0,0.0,0.0,t17,-t3.*(t4-t5),0.0,V.*(C_f1.*t2+C_f2.*t3),0.0,0.0,k.*m_f.*t6,t6.*(m_f+m_r),h.*m_r.*t6+j.*m_f.*t6+i_fy.*t6.*t8+i_ry.*t6.*t9,e.*m_f.*t6+i_fy.*t3.*t6.*t8,C_r1.*b,-C_f1.*t11.*t12,0.0,0.0,0.0,0.0,0.0,0.0,-C_r1,-C_f1,1.0,0.0,t13+t14,0.0,0.0,t15,0.0,0.0,0.0,1.0,t16,0.0,i_fy.*t2.*t6.*t8,-C_delta,0.0,C_f1.*a_n,0.0,0.0,-b,1.0,0.0,0.0,t6,0.0,0.0,0.0,t11.*t12,1.0,0.0,t7,0.0,t6],[8,8]);
end