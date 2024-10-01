function A = dynamicBicycleTireDynamicsStateMatrix(in1,in2)
%dynamicBicycleTireDynamicsStateMatrix
%    A = dynamicBicycleTireDynamicsStateMatrix(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    01-Oct-2024 13:51:48

%   states = [omega_z v_x v_y F_yr F_yf]
%   inputs = [delta a_x]
%   params = [C_f C_r I_zz V l_f l_r m sigma_f sigma_r]
%
C_f = in2(1,:);
C_r = in2(2,:);
I_zz = in2(3,:);
V = in2(4,:);
l_f = in2(5,:);
l_r = in2(6,:);
m = in2(7,:);
sigma_f = in2(8,:);
sigma_r = in2(9,:);
t2 = 1.0./I_zz;
t3 = 1.0./m;
t4 = 1.0./sigma_f;
t5 = 1.0./sigma_r;
t6 = t3.*2.0;
A = reshape([0.0,0.0,-V,C_r.*l_r.*t5,-C_f.*l_f.*t4,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-C_r.*t5,-C_f.*t4,l_r.*t2.*-2.0,0.0,t6,-V.*t5,0.0,l_f.*t2.*2.0,0.0,t6,0.0,-V.*t4],[5,5]);
