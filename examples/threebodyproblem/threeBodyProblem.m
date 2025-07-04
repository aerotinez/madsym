function out1 = threeBodyProblem(in1,in2)
%threeBodyProblem
%    OUT1 = threeBodyProblem(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    21-Jun-2025 22:26:37

G = in2(1,:);
m_1 = in2(2,:);
m_2 = in2(3,:);
m_3 = in2(4,:);
u_1 = in1(10,:);
u_2 = in1(13,:);
u_3 = in1(16,:);
v_1 = in1(11,:);
v_2 = in1(14,:);
v_3 = in1(17,:);
w_1 = in1(12,:);
w_2 = in1(15,:);
w_3 = in1(18,:);
x_1 = in1(1,:);
x_2 = in1(4,:);
x_3 = in1(7,:);
y_1 = in1(2,:);
y_2 = in1(5,:);
y_3 = in1(8,:);
z_1 = in1(3,:);
z_2 = in1(6,:);
z_3 = in1(9,:);
t2 = x_1.^2;
t3 = x_2.^2;
t4 = x_3.^2;
t5 = y_1.^2;
t6 = y_2.^2;
t7 = y_3.^2;
t8 = z_1.^2;
t9 = z_2.^2;
t10 = z_3.^2;
t11 = x_1.*x_2.*2.0;
t12 = x_1.*x_3.*2.0;
t13 = x_2.*x_3.*2.0;
t14 = y_1.*y_2.*2.0;
t15 = y_1.*y_3.*2.0;
t16 = y_2.*y_3.*2.0;
t17 = z_1.*z_2.*2.0;
t18 = z_1.*z_3.*2.0;
t19 = z_2.*z_3.*2.0;
t20 = -t11;
t21 = -t12;
t22 = -t13;
t23 = -t14;
t24 = -t15;
t25 = -t16;
t26 = -t17;
t27 = -t18;
t28 = -t19;
t29 = t2+t3+t5+t6+t8+t9+t20+t23+t26;
t30 = t2+t4+t5+t7+t8+t10+t21+t24+t27;
t31 = t3+t4+t6+t7+t9+t10+t22+t25+t28;
t32 = 1.0./t29.^(3.0./2.0);
t33 = 1.0./t30.^(3.0./2.0);
t34 = 1.0./t31.^(3.0./2.0);
mt1 = [u_1;v_1;w_1;u_2;v_2;w_2;u_3;v_3;w_3;-G.*m_2.*t32.*x_1+G.*m_2.*t32.*x_2-G.*m_3.*t33.*x_1+G.*m_3.*t33.*x_3;-G.*m_2.*t32.*y_1+G.*m_2.*t32.*y_2-G.*m_3.*t33.*y_1+G.*m_3.*t33.*y_3;-G.*m_2.*t32.*z_1+G.*m_2.*t32.*z_2-G.*m_3.*t33.*z_1+G.*m_3.*t33.*z_3;G.*m_1.*t32.*x_1-G.*m_1.*t32.*x_2-G.*m_3.*t34.*x_2+G.*m_3.*t34.*x_3;G.*m_1.*t32.*y_1-G.*m_1.*t32.*y_2-G.*m_3.*t34.*y_2+G.*m_3.*t34.*y_3;G.*m_1.*t32.*z_1-G.*m_1.*t32.*z_2-G.*m_3.*t34.*z_2+G.*m_3.*t34.*z_3;G.*m_1.*t33.*x_1-G.*m_1.*t33.*x_3+G.*m_2.*t34.*x_2-G.*m_2.*t34.*x_3];
mt2 = [G.*m_1.*t33.*y_1-G.*m_1.*t33.*y_3+G.*m_2.*t34.*y_2-G.*m_2.*t34.*y_3;G.*m_1.*t33.*z_1-G.*m_1.*t33.*z_3+G.*m_2.*t34.*z_2-G.*m_2.*t34.*z_3];
out1 = [mt1;mt2];
end
