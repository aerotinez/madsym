function out1 = motoEgoMotionMassMatrix(in1,in2)
%motoEgoMotionMassMatrix
%    OUT1 = motoEgoMotionMassMatrix(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    20-Jun-2025 17:36:45

f = in2(:,1);
gamma = in1(:,2);
mu = in2(:,2);
p_x = in1(:,4);
p_y = in1(:,5);
p_z = in1(:,6);
theta = in1(:,3);
t2 = cos(gamma);
t3 = sin(gamma);
t4 = mu+theta;
t7 = 1.0./p_z.^2;
t5 = cos(t4);
t6 = sin(t4);
t8 = 1.0./t2;
t9 = 1.0./t6;
out1 = reshape([f.*t7.*t8.*t9.*(p_x.*t3+p_z.*t2.*cos(mu).*sin(theta)+p_z.*t2.*cos(theta).*sin(mu)),f.*p_y.*t3.*t7.*t8.*t9,f.*p_x.*t5.*t7.*t9,f.*t7.*t9.*(p_y.*t5+p_z.*t6)],[2,2]);
end
