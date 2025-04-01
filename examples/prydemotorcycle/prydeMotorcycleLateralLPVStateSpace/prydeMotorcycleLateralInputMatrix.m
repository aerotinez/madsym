function G = prydeMotorcycleLateralInputMatrix(in1)
%prydeMotorcycleLateralInputMatrix
%    G = prydeMotorcycleLateralInputMatrix(IN1)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    01-Apr-2025 13:01:05

%   states = [varphi delta omega_bz v_ry omega_bx omega_delta]
%   inputs = [tau_r tau_delta]
%   params = [C_delta I_bxx I_bxz I_bzz I_hxx I_hzz K_xkf K_xkr K_yaf K_ygf K_yar K_ygr K_zaf K_zgf K_zar K_zgr V a a_n b e f g h i_fy i_ry m_b m_h rho_f rho_r t_f t_r varepsilon]
%
G = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,0.0,0.0,0.0,0.0],[24,2]);
end
