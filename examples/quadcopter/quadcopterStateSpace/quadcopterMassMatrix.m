function M = quadcopterMassMatrix(in1)
%quadcopterMassMatrix
%    M = quadcopterMassMatrix(IN1)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    22-Nov-2024 13:12:28

%   states = [psi theta phi p_x p_y p_z omega_x omega_y omega_z v_x v_y v_z]
%   inputs = [tau_x tau_y tau_z f_z]
%   params = [I_xx I_yy I_zz g m]
%
I_xx = in1(1,:);
I_yy = in1(2,:);
I_zz = in1(3,:);
m = in1(5,:);
M = reshape([1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,I_xx,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,I_yy,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,I_zz,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,m,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,m,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,m],[12,12]);
end
