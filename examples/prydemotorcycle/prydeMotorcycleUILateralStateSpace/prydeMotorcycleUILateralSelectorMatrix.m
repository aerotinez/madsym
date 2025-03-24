function S = prydeMotorcycleUILateralSelectorMatrix
%prydeMotorcycleUILateralSelectorMatrix
%    S = prydeMotorcycleUILateralSelectorMatrix

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    24-Mar-2025 21:03:15

%   states = [varphi delta omega_bz v_ry omega_bx omega_delta]
%   inputs = [tau_r tau_delta df_yr df_yf dm_zr dm_zf tau_x f_by]
%   params = [C_delta I_bxx I_bxz I_bzz I_hxx I_hzz K_xkf K_xkr K_yaf K_ygf K_yar K_ygr K_zaf K_zgf K_zar K_zgr V a a_n b e f g h i_fy i_ry m_b m_h rho_f rho_r t_f t_r varepsilon]
%
S = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0],[6,14]);
end
