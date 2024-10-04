function B = dynamicBicycleTireDynamicsInputMatrix(in1,in2)
%dynamicBicycleTireDynamicsInputMatrix
%    B = dynamicBicycleTireDynamicsInputMatrix(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    01-Oct-2024 13:51:48

%   states = [omega_z v_x v_y F_yr F_yf]
%   inputs = [delta a_x]
%   params = [C_f C_r I_zz V l_f l_r m sigma_f sigma_r]
%
C_f = in2(1,:);
V = in2(4,:);
sigma_f = in2(8,:);
B = reshape([0.0,0.0,0.0,0.0,(C_f.*V)./sigma_f,0.0,1.0,0.0,0.0,0.0],[5,2]);