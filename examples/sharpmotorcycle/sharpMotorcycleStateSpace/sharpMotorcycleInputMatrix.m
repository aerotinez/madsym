function G = sharpMotorcycleInputMatrix(in1)
%sharpMotorcycleInputMatrix
%    G = sharpMotorcycleInputMatrix(IN1)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    24-Feb-2025 06:53:06

%   states = [varphi delta omega_psi v_y omega_varphi omega_delta Y_r Y_f]
%   inputs = [tau_delta]
%   params = [C_delta C_f1 C_f2 C_r1 C_r2 C_rxz I_fx I_fz I_rx I_rz V Z_f a a_n b e g h i_fy i_ry j k m_f m_r r_f r_r sigma varepsilon]
%
G = [0.0;0.0;0.0;0.0;0.0;1.0;0.0;0.0];
end
