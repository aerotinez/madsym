function out1 = steerModel(in1,in2)
%steerModel
%    OUT1 = steerModel(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    21-Jun-2025 01:30:24

a = in2(:,1);
a_n = in2(:,2);
b = in2(:,3);
t_f = in2(:,6);
t_r = in2(:,7);
theta = in1(:,2);
varphi = in1(:,1);
varepsilon = in2(:,8);
t2 = cos(varphi);
t3 = cos(varepsilon);
t4 = sin(varepsilon);
out1 = -(t2.*(t3.*t_f-t3.*t_r+a.*t2.*theta-a_n.*t2.*theta-t2.*t3.*t_f+t2.*t3.*t_r+b.*t2.*t3.*theta))./(t3.*sin(varphi).*(a_n.*t2+t4.*t_f-t4.*t_r-t2.*t4.*t_f.*2.0+t2.*t4.*t_r));
end
