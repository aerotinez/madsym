function S = prydeMotorcycleLateralSelectorMatrix
%prydeMotorcycleLateralSelectorMatrix
%    S = prydeMotorcycleLateralSelectorMatrix

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    09-Jan-2025 04:43:23

%   states = [varphi delta omega_bz v_ry omega_bx omega_delta]
%   inputs = [tau_r tau_delta]
%   params = [C_delta I_bxx I_bxz I_bzz I_hxx I_hzz Psi_alpha_f Psi_alpha_r Psi_varphi_f Psi_varphi_r V Y_alpha_f Y_alpha_r Y_varphi_f Y_varphi_r Z_f Z_r a a_f0 a_n a_r0 b e f f_w0 f_w2 g h i_fy i_ry k_kappa_f k_kappa_r m_b m_h rho_f rho_r t_f t_r varepsilon]
%
S = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0],[6,14]);
end
