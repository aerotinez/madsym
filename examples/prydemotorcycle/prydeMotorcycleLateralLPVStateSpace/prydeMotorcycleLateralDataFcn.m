function [A,B,C,D,E,dx0,x0,u0,y0,Delays] = prydeMotorcycleLateralDataFcn(~,p)
   %    states = [varphi delta omega_bz v_ry omega_bx omega_delta]
   %    inputs = [tau_r tau_br tau_bf tau_delta]
   %    params = [A C_d C_delta C_l C_p I_bxx I_bxz I_bzz I_hxx I_hzz K_xkf K_xkr K_yaf K_ycf K_ygf K_yar K_ycr K_ygr K_yvf K_yvr K_zaf K_zgf K_zar K_zgr V a a_n b e f g h i_fy i_ry m_b m_h rho rho_f rho_r t_f t_r varepsilon]
   Sx = prydeMotorcycleLateralStateSelectorMatrix();
   P = prydeMotorcycleLateralPermutationMatrix();
   M = prydeMotorcycleLateralMassMatrix(p);
   H = prydeMotorcycleLateralForcingMatrix(p);
   G = prydeMotorcycleLateralInputMatrix(p);
   Su = prydeMotorcycleLateralInputSelectorMatrix();
   A = Sx*P*(M\H)*Sx.';
   B = Sx*P*(M\G)*Su.';
   C = eye(size(A));
   D = zeros(size(B));
   E = [];
   dx0 = [];
   x0 = zeros(size(A,1),1);
   u0 = zeros(size(B,2),1);
   y0 = zeros(size(A,1),1);
   Delays = [];
