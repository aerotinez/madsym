function out1 = normalForcesAtTrim(in1)
%normalForcesAtTrim
%    OUT1 = normalForcesAtTrim(IN1)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    03-Apr-2025 03:10:44

A = in1(1,:);
C_d = in1(2,:);
C_l = in1(4,:);
C_p = in1(5,:);
K_ycf = in1(14,:);
K_ycr = in1(17,:);
K_yvf = in1(19,:);
K_yvr = in1(20,:);
V = in1(25,:);
a = in1(26,:);
a_n = in1(27,:);
b = in1(28,:);
e = in1(29,:);
f = in1(30,:);
g = in1(31,:);
m_b = in1(35,:);
m_h = in1(36,:);
rho = in1(37,:);
rho_r = in1(39,:);
t_r = in1(41,:);
varepsilon = in1(42,:);
t2 = cos(varepsilon);
t3 = V.^2;
t4 = V.^3;
t5 = varepsilon.*2.0;
t9 = -a_n;
t6 = t2.^2;
t7 = sin(t5);
t8 = b.*t2;
t10 = K_ycf.*rho_r.*t2;
t11 = K_ycr.*rho_r.*t2;
t12 = K_ycf.*t2.*t_r;
t13 = K_ycr.*t2.*t_r;
t14 = K_yvf.*V.*rho_r.*t2;
t15 = K_yvr.*V.*rho_r.*t2;
t16 = K_yvf.*V.*t2.*t_r;
t17 = K_yvr.*V.*t2.*t_r;
t22 = A.*C_l.*a.*rho.*t3;
t23 = A.*C_l.*a_n.*rho.*t3;
t24 = A.*C_p.*a.*rho.*t3.*2.0;
t25 = A.*C_p.*a_n.*rho.*t3.*2.0;
t33 = A.*C_d.*rho.*rho_r.*t2.*t3.*2.0;
t34 = A.*C_d.*rho.*t2.*t3.*t_r.*2.0;
t18 = -t10;
t19 = -t12;
t20 = -t14;
t21 = -t16;
t26 = a.*g.*m_h.*t6.*4.0;
t27 = e.*g.*m_h.*t6.*4.0;
t28 = f.*g.*m_h.*t7.*2.0;
t29 = A.*C_l.*rho.*t3.*t8;
t30 = -t24;
t32 = A.*C_p.*rho.*t3.*t8.*2.0;
t31 = -t28;
t35 = -t32;
t36 = a+t8+t9+t11+t13+t15+t17+t18+t19+t20+t21;
t37 = 1.0./t36;
mt1 = [t37.*(t22+t25+t26+t27+t29+t30+t31+t33+t34+t35-a.*g.*m_b.*4.0-a.*g.*m_h.*4.0+a_n.*g.*m_b.*4.0+a_n.*g.*m_h.*4.0+g.*m_b.*t10.*4.0+g.*m_b.*t12.*4.0+g.*m_b.*t14.*4.0+g.*m_b.*t16.*4.0+g.*m_h.*t10.*4.0+g.*m_h.*t12.*4.0+g.*m_h.*t14.*4.0+g.*m_h.*t16.*4.0+A.*C_l.*rho.*t3.*t9-A.*C_l.*rho.*t3.*t10.*2.0-A.*C_l.*rho.*t3.*t12.*2.0-A.*C_l.*K_yvf.*rho.*rho_r.*t2.*t4.*2.0-A.*C_l.*K_yvf.*rho.*t2.*t4.*t_r.*2.0).*(-1.0./4.0)];
mt2 = [(t37.*(-t22+t23+t25+t26+t27-t29+t30+t31+t33+t34+t35+g.*m_b.*t8.*4.0+g.*m_b.*t11.*4.0+g.*m_b.*t13.*4.0+g.*m_b.*t15.*4.0+g.*m_b.*t17.*4.0+g.*m_h.*t8.*4.0+g.*m_h.*t11.*4.0+g.*m_h.*t13.*4.0+g.*m_h.*t15.*4.0+g.*m_h.*t17.*4.0-A.*C_l.*rho.*t3.*t11.*2.0-A.*C_l.*rho.*t3.*t13.*2.0-A.*C_l.*K_yvr.*rho.*rho_r.*t2.*t4.*2.0-A.*C_l.*K_yvr.*rho.*t2.*t4.*t_r.*2.0))./4.0];
out1 = [mt1;mt2];
end
