function P = prydeMotorcyclePermutationMatrix
%prydeMotorcyclePermutationMatrix
%    P = prydeMotorcyclePermutationMatrix

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    05-Dec-2024 18:54:37

%   states = [varphi delta omega_bz v_rx v_ry omega_bx omega_delta]
%   inputs = [tau_delta]
%   params = [C_delta I_bxx I_bxz I_bzz I_hxx I_hzz V Z_f Z_r a a_n b e f g h i_fy i_ry k_alpha_f k_alpha_r k_varphi_f k_varphi_r m_b m_h rho_f rho_r t_f t_r varepsilon]
%
P = reshape([1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0],[7,7]);
end
