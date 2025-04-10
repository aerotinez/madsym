function out1 = normalForcesAtTrim(in1)
%normalForcesAtTrim
%    OUT1 = normalForcesAtTrim(IN1)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    09-Apr-2025 18:51:40

A = in1(1,:);
C_d = in1(2,:);
C_l = in1(4,:);
C_p = in1(5,:);
V = in1(25,:);
a = in1(26,:);
a_n = in1(27,:);
b = in1(28,:);
e = in1(29,:);
f = in1(30,:);
g = in1(31,:);
m_b = in1(35,:);
m_f = in1(36,:);
m_h = in1(37,:);
m_r = in1(38,:);
rho = in1(39,:);
rho_r = in1(41,:);
t_r = in1(43,:);
varepsilon = in1(44,:);
t2 = cos(varepsilon);
t3 = V.^2;
t4 = varepsilon.*2.0;
t8 = -a_n;
t5 = t2.^2;
t6 = sin(t4);
t7 = b.*t2;
t9 = A.*C_l.*a.*rho.*t3;
t10 = A.*C_l.*a_n.*rho.*t3;
t11 = A.*C_p.*a.*rho.*t3.*2.0;
t12 = A.*C_p.*a_n.*rho.*t3.*2.0;
t21 = A.*C_d.*rho.*rho_r.*t2.*t3.*2.0;
t22 = A.*C_d.*rho.*t2.*t3.*t_r.*2.0;
t13 = a.*g.*m_h.*t5.*4.0;
t14 = e.*g.*m_h.*t5.*4.0;
t15 = f.*g.*m_h.*t6.*2.0;
t16 = a+t7+t8;
t17 = A.*C_l.*rho.*t3.*t7;
t18 = -t11;
t20 = A.*C_p.*rho.*t3.*t7.*2.0;
t19 = -t15;
t23 = -t20;
t24 = 1.0./t16;
out1 = [t24.*(t9+t12+t13+t14+t17+t18+t19+t21+t22+t23-a.*g.*m_b.*4.0-a.*g.*m_h.*4.0-a.*g.*m_r.*4.0+a_n.*g.*m_b.*4.0+a_n.*g.*m_h.*4.0+a_n.*g.*m_r.*4.0-g.*m_r.*t7.*4.0+A.*C_l.*rho.*t3.*t8).*(-1.0./4.0);(t24.*(-t9+t10+t12+t13+t14-t17+t18+t19+t21+t22+t23+a.*g.*m_f.*4.0-a_n.*g.*m_f.*4.0+g.*m_b.*t7.*4.0+g.*m_f.*t7.*4.0+g.*m_h.*t7.*4.0))./4.0];
end
