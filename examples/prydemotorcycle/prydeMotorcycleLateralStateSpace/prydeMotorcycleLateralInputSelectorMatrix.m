function Su = prydeMotorcycleLateralInputSelectorMatrix
%prydeMotorcycleLateralInputSelectorMatrix
%    Su = prydeMotorcycleLateralInputSelectorMatrix

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    31-Mar-2025 16:44:30

%   states = [varphi delta omega_bz v_ry omega_bx omega_delta]
%   inputs = [tau_r tau_br tau_bf tau_delta]
%   params = [C_delta I_bxx I_bxz I_bzz I_hxx I_hzz K_ycf K_ycr K_yvf K_yvr V a a_n b e f f_z0f f_z0r g h i_fy i_ry m_b m_h p_Kx1f p_Kx2f p_Ky1f p_Kx3f p_Ky2f p_Ky3f p_Ky6f p_Ky7f p_Kx1r p_Kx2r p_Ky1r p_Kx3r p_Ky2r p_Ky3r p_Ky6r p_Ky7r q_Dz1f q_Dz2f q_Dz8f q_Dz9f q_Dz1r q_Dz2r q_Dz8r q_Dz9r rho_f rho_r t_f t_r varepsilon]
%
Su = [0.0,0.0,0.0,1.0];
end
