function H = prydeMotorcycleLongitudinalLPVSSForcingMatrix(in1)
%prydeMotorcycleLongitudinalLPVSSForcingMatrix
%    H = prydeMotorcycleLongitudinalLPVSSForcingMatrix(IN1)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    08-May-2025 03:56:19

%   states = [v_rx omega_r omega_f]
%   inputs = [tau_r tau_br tau_bf]
%   params = [A C_d C_delta C_l C_p I_bxx I_bxz I_bzz I_hxx I_hzz K_xkf K_xkr K_yaf K_ycf K_ygf K_yar K_ycr K_ygr K_yvf K_yvr K_zaf K_zgf K_zar K_zgr V a a_n b e f g h i_fy i_ry m_b m_f m_h m_r rho rho_f rho_r t_f t_r varepsilon]
%
A = in1(1,:);
C_d = in1(2,:);
C_l = in1(4,:);
C_p = in1(5,:);
K_xkf = in1(11,:);
K_xkr = in1(12,:);
K_ycf = in1(14,:);
K_ycr = in1(17,:);
K_yvf = in1(19,:);
K_yvr = in1(20,:);
V = in1(25,:);
a = in1(26,:);
a_n = in1(27,:);
b = in1(28,:);
e = in1(29,:);
f = in1(30,:);
g = in1(31,:);
m_b = in1(35,:);
m_f = in1(36,:);
m_h = in1(37,:);
m_r = in1(38,:);
rho = in1(39,:);
rho_f = in1(40,:);
rho_r = in1(41,:);
t_f = in1(42,:);
t_r = in1(43,:);
varepsilon = in1(44,:);
t2 = cos(varepsilon);
t3 = rho_f+t_f;
t4 = rho_r+t_r;
t5 = V.^2;
t6 = V.^3;
t7 = varepsilon.*2.0;
t8 = K_xkf.*a.*4.0;
t9 = K_xkr.*a.*4.0;
t10 = K_xkf.*a_n.*4.0;
t11 = K_xkr.*a_n.*4.0;
t15 = 1.0./V;
t16 = -a_n;
t18 = K_ycr.*a.*g.*m_b.*4.0;
t19 = K_ycf.*a.*g.*m_f.*4.0;
t20 = K_ycr.*a.*g.*m_h.*4.0;
t21 = K_ycr.*a.*g.*m_r.*4.0;
t22 = K_ycr.*a_n.*g.*m_b.*4.0;
t23 = K_ycf.*a_n.*g.*m_f.*4.0;
t24 = K_ycr.*a_n.*g.*m_h.*4.0;
t25 = K_ycr.*a_n.*g.*m_r.*4.0;
t29 = K_yvr.*V.*a.*g.*m_b.*8.0;
t30 = K_yvf.*V.*a.*g.*m_f.*8.0;
t31 = K_yvr.*V.*a.*g.*m_h.*8.0;
t32 = K_yvr.*V.*a.*g.*m_r.*8.0;
t33 = K_yvr.*V.*a_n.*g.*m_b.*8.0;
t34 = K_yvf.*V.*a_n.*g.*m_f.*8.0;
t35 = K_yvr.*V.*a_n.*g.*m_h.*8.0;
t36 = K_yvr.*V.*a_n.*g.*m_r.*8.0;
t12 = t2.^2;
t13 = sin(t7);
t14 = b.*t2;
t17 = -t10;
t28 = -t23;
t41 = -t34;
t42 = A.*C_l.*K_ycf.*a.*rho.*t5;
t43 = A.*C_l.*K_ycr.*a.*rho.*t5;
t44 = A.*C_l.*K_ycf.*a_n.*rho.*t5;
t45 = A.*C_l.*K_ycr.*a_n.*rho.*t5;
t57 = A.*C_l.*K_yvf.*a.*rho.*t6.*2.0;
t58 = A.*C_l.*K_yvr.*a.*rho.*t6.*2.0;
t59 = A.*C_p.*K_ycf.*a.*rho.*t5.*2.0;
t60 = A.*C_p.*K_ycr.*a.*rho.*t5.*2.0;
t61 = A.*C_p.*K_yvf.*a.*rho.*t6.*4.0;
t62 = A.*C_p.*K_yvr.*a.*rho.*t6.*4.0;
t63 = A.*C_l.*K_yvf.*a_n.*rho.*t6.*2.0;
t64 = A.*C_l.*K_yvr.*a_n.*rho.*t6.*2.0;
t65 = A.*C_p.*K_ycf.*a_n.*rho.*t5.*2.0;
t66 = A.*C_p.*K_ycr.*a_n.*rho.*t5.*2.0;
t67 = A.*C_p.*K_yvf.*a_n.*rho.*t6.*4.0;
t68 = A.*C_p.*K_yvr.*a_n.*rho.*t6.*4.0;
t89 = A.*C_d.*K_ycf.*rho.*rho_r.*t2.*t5.*2.0;
t90 = A.*C_d.*K_ycr.*rho.*rho_r.*t2.*t5.*2.0;
t91 = A.*C_d.*K_yvf.*rho.*rho_r.*t2.*t6.*4.0;
t92 = A.*C_d.*K_yvr.*rho.*rho_r.*t2.*t6.*4.0;
t93 = A.*C_d.*K_ycf.*rho.*t2.*t5.*t_r.*2.0;
t94 = A.*C_d.*K_ycr.*rho.*t2.*t5.*t_r.*2.0;
t95 = A.*C_d.*K_yvf.*rho.*t2.*t6.*t_r.*4.0;
t96 = A.*C_d.*K_yvr.*rho.*t2.*t6.*t_r.*4.0;
t26 = K_xkf.*t14.*4.0;
t27 = K_xkr.*t14.*4.0;
t37 = K_ycf.*g.*m_b.*t14.*4.0;
t38 = K_ycf.*g.*m_f.*t14.*4.0;
t39 = K_ycf.*g.*m_h.*t14.*4.0;
t40 = K_ycr.*g.*m_r.*t14.*4.0;
t46 = K_yvf.*V.*g.*m_b.*t14.*8.0;
t47 = K_yvf.*V.*g.*m_f.*t14.*8.0;
t48 = K_yvf.*V.*g.*m_h.*t14.*8.0;
t49 = K_yvr.*V.*g.*m_r.*t14.*8.0;
t50 = K_ycf.*a.*g.*m_h.*t12.*4.0;
t51 = t12.*t20;
t52 = a+t14+t16;
t53 = K_ycf.*e.*g.*m_h.*t12.*4.0;
t54 = K_ycr.*e.*g.*m_h.*t12.*4.0;
t55 = K_ycf.*f.*g.*m_h.*t13.*2.0;
t56 = K_ycr.*f.*g.*m_h.*t13.*2.0;
t70 = K_yvf.*V.*f.*g.*m_h.*t13.*4.0;
t71 = K_yvr.*V.*f.*g.*m_h.*t13.*4.0;
t72 = A.*C_l.*K_ycf.*rho.*t5.*t14;
t73 = A.*C_l.*K_ycr.*rho.*t5.*t14;
t74 = -t42;
t75 = -t57;
t76 = -t59;
t77 = -t61;
t78 = K_yvf.*V.*a.*g.*m_h.*t12.*8.0;
t79 = t12.*t31;
t80 = K_yvf.*V.*e.*g.*m_h.*t12.*8.0;
t81 = K_yvr.*V.*e.*g.*m_h.*t12.*8.0;
t83 = A.*C_l.*K_yvf.*rho.*t6.*t14.*2.0;
t84 = A.*C_l.*K_yvr.*rho.*t6.*t14.*2.0;
t85 = A.*C_p.*K_ycf.*rho.*t5.*t14.*2.0;
t86 = A.*C_p.*K_ycr.*rho.*t5.*t14.*2.0;
t87 = A.*C_p.*K_yvf.*rho.*t6.*t14.*4.0;
t88 = A.*C_p.*K_yvr.*rho.*t6.*t14.*4.0;
t69 = -t55;
t82 = -t70;
t97 = 1.0./t52;
t98 = -t72;
t99 = -t83;
t100 = -t85;
t101 = -t87;
mt1 = [(t15.*t97.*(t8+t9-t11+t17+t18+t19+t20+t21-t22-t24-t25+t26+t27+t28+t29+t30+t31+t32-t33-t35-t36+t37+t38+t39+t40+t41-t43+t44+t45+t46+t47+t48+t49+t50+t53-t54+t56-t58+t60+t62+t63+t64+t65-t66+t67-t68+t69+t71-t73+t74+t75+t76+t77+t78+t80-t81+t82-t84+t86+t88+t89-t90+t91-t92+t93-t94+t95-t96+t98+t99+t100+t101+A.*C_d.*a.*rho.*t5.*6.0-A.*C_d.*a_n.*rho.*t5.*6.0+A.*C_d.*rho.*t5.*t14.*6.0-K_ycr.*a.*g.*m_h.*t12.*4.0-K_yvr.*V.*a.*g.*m_h.*t12.*8.0))./4.0];
mt2 = [(t4.*t15.*t97.*(-t9+t11-t18-t20-t21+t22+t24+t25-t27-t29-t31-t32+t33+t35+t36-t40+t43-t49+t51+t54-t56+t58-t60-t62-t64+t66+t68-t71+t73+t79+t81+t84-t86-t88+t90+t92+t94+t96-A.*C_d.*a.*rho.*t5.*2.0+A.*C_d.*a_n.*rho.*t5.*2.0-A.*C_d.*rho.*t5.*t14.*2.0+A.*C_l.*K_ycr.*rho.*t5.*t16))./4.0,t3.*t15.*t97.*(t8+t17+t19+t26+t28+t30+t37+t38+t39+t41+t44+t46+t47+t48+t50+t53+t63+t65+t67+t69+t74+t75+t76+t77+t78+t80+t82+t89+t91+t93+t95+t98+t99+t100+t101).*(-1.0./4.0),-K_xkr.*t4.*t15,K_xkr.*t4.^2.*t15,0.0,-K_xkf.*t3.*t15,0.0,K_xkf.*t3.^2.*t15];
H = reshape([mt1,mt2],3,3);
end
