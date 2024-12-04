function P = sharpMotorcycleSteadyStatePermutationMatrix
%sharpMotorcycleSteadyStatePermutationMatrix
%    P = sharpMotorcycleSteadyStatePermutationMatrix

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    04-Dec-2024 09:56:36

%   states = [varphi delta omega_psi v_y omega_varphi omega_delta]
%   inputs = [tau_delta]
%   params = [C_delta C_f1 C_f2 C_r1 C_r2 C_rxz I_fx I_fz I_rx I_rz V Z_f a a_n b e g h i_fy i_ry j k m_f m_r r_f r_r varepsilon]
%
P = reshape([1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0],[6,6]);
end
