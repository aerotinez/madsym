function out1 = dynamicBicycleInputMatrix(in1,in2)
%dynamicBicycleInputMatrix
%    OUT1 = dynamicBicycleInputMatrix(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    02-Jul-2024 00:15:15

%    states = [x y psi v_x v_y omega_z]
%    params = [C_f C_r I_zz l_f l_r m]
%
C_f = in2(1,:);
I_zz = in2(3,:);
l_f = in2(4,:);
m = in2(6,:);
out1 = reshape([0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,(C_f.*2.0)./m,(C_f.*l_f.*2.0)./I_zz],[6,2]);
