function M = prydeMotorcycleLateralSSMassMatrix(in1)
%prydeMotorcycleLateralSSMassMatrix
%    M = prydeMotorcycleLateralSSMassMatrix(IN1)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    08-May-2025 03:55:51

%   states = [varphi delta omega_bz v_ry omega_bx omega_delta]
%   inputs = [tau_delta]
%   params = [A C_d C_delta C_l C_p I_bxx I_bxz I_bzz I_hxx I_hzz K_xkf K_xkr K_yaf K_ycf K_ygf K_yar K_ycr K_ygr K_yvf K_yvr K_zaf K_zgf K_zar K_zgr V a a_n b e f g h i_fy i_ry m_b m_f m_h m_r rho rho_f rho_r t_f t_r varepsilon]
%
I_bxx = in1(6,:);
I_bxz = in1(7,:);
I_bzz = in1(8,:);
I_hxx = in1(9,:);
I_hzz = in1(10,:);
a = in1(26,:);
a_n = in1(27,:);
b = in1(28,:);
e = in1(29,:);
f = in1(30,:);
h = in1(32,:);
m_b = in1(35,:);
m_f = in1(36,:);
m_h = in1(37,:);
m_r = in1(38,:);
rho_f = in1(40,:);
rho_r = in1(41,:);
t_f = in1(42,:);
t_r = in1(43,:);
varepsilon = in1(44,:);
t2 = cos(varepsilon);
t3 = sin(varepsilon);
t4 = a_n.*m_f;
t5 = b.*m_b;
t6 = b.*m_f;
t7 = b.*m_h;
t8 = e.*m_h;
t9 = h.*m_b;
t10 = m_b.*rho_r;
t11 = m_f.*rho_f;
t12 = m_f.*rho_r;
t13 = m_h.*rho_r;
t14 = m_f.*t_f;
t15 = m_b.*t_r;
t16 = m_f.*t_r;
t17 = m_h.*t_r;
t18 = a.^2;
t22 = f.^2;
t27 = varepsilon.*2.0;
t49 = -I_bxz;
t28 = cos(t27);
t29 = t2.^2;
t30 = sin(t27);
t31 = t3.^2;
t32 = a.*t4;
t33 = I_hzz.*t3;
t34 = h.*t5;
t35 = rho_f.*t4;
t36 = rho_r.*t4;
t37 = rho_r.*t5;
t38 = rho_f.*t6;
t39 = rho_r.*t6;
t40 = rho_r.*t7;
t41 = t4.*t_f;
t42 = t4.*t_r;
t43 = t6.*t_f;
t44 = t5.*t_r;
t45 = t6.*t_r;
t46 = t7.*t_r;
t47 = rho_r.*t8;
t48 = t8.*t_r;
t50 = a.*m_h.*t2;
t51 = t2.*t8;
t52 = f.*m_h.*t2;
t53 = a.*m_h.*t3;
t54 = t3.*t8;
t55 = f.*m_h.*t3;
t56 = t3.*t11;
t57 = t3.*t14;
t58 = -t5;
t59 = -t6;
t60 = -t7;
t61 = -t8;
t62 = 1.0./t2;
t64 = -t10;
t65 = -t12;
t66 = -t13;
t67 = -t15;
t68 = -t16;
t69 = -t17;
t70 = a_n.*t4;
t71 = b.*t2.*t4;
t72 = e.*t2.*t7;
t73 = f.*t2.*t7;
t75 = a.*t3.*t7;
t79 = e.*t3.*t7;
t86 = f.*t3.*t13;
t88 = f.*t3.*t17;
t63 = 1.0./t29;
t74 = f.*t51;
t78 = a.*t54;
t80 = t2.*t47;
t81 = t2.*t48;
t82 = a.*t56;
t83 = t3.*t35;
t84 = a.*t57;
t85 = t3.*t41;
t87 = -t32;
t89 = rho_r.*t56;
t91 = t56.*t_r;
t92 = t57.*t_r;
t93 = -t35;
t94 = -t37;
t95 = -t39;
t96 = -t40;
t97 = -t41;
t98 = -t44;
t99 = -t45;
t100 = -t46;
t101 = -t47;
t102 = -t48;
t103 = I_hzz.*t29;
t104 = -t50;
t105 = -t51;
t106 = -t56;
t107 = -t57;
t108 = e.*t54;
t109 = rho_f.*t56;
t110 = t57.*t_f;
t111 = t56.*t_f.*2.0;
t112 = a.*m_f.*t62;
t113 = t4.*t62;
t114 = -t71;
t115 = a.*t2.*t66;
t116 = a.*t2.*t69;
t122 = t3.*t65.*t_f;
t125 = a.*t8.*t29;
t126 = a.*f.*m_h.*t28;
t127 = f.*t8.*t28;
t128 = a.*t8.*t30;
t129 = a.*f.*m_h.*t30;
t130 = f.*t8.*t30;
t131 = e.*t8.*t29;
t132 = a.*t11.*t62;
t135 = t36.*t62;
t136 = a.*t14.*t62;
t139 = t42.*t62;
t140 = (I_hxx.*t30)./2.0;
t141 = (I_hzz.*t30)./2.0;
t144 = a.*t62.*t65;
t146 = a.*t62.*t68;
t149 = (t30.*t38)./2.0;
t150 = (t30.*t43)./2.0;
t151 = (m_h.*t18.*t30)./2.0;
t152 = (e.*t8.*t30)./2.0;
t153 = (m_h.*t22.*t30)./2.0;
t158 = t9+t11+t14+t52+t53+t54+t64+t65+t66+t67+t68+t69;
t117 = -t80;
t118 = -t81;
t119 = -t83;
t120 = -t85;
t121 = -t89;
t123 = -t91;
t124 = -t92;
t142 = -t112;
t143 = -t140;
t145 = t62.*t93;
t147 = t62.*t97;
t148 = t130./2.0;
t155 = -t153;
t156 = t4+t61+t106+t107;
t154 = -t148;
t157 = t55+t58+t59+t60+t104+t105+t113+t142;
t159 = t33+t36+t42+t74+t78+t93+t97+t101+t102+t108+t109+t110+t111+t121+t122+t123+t124;
t163 = t34+t38+t43+t49+t73+t75+t79+t86+t88+t94+t95+t96+t98+t99+t100+t115+t116+t117+t118+t126+t127+t128+t132+t135+t136+t139+t141+t143+t144+t145+t146+t147+t151+t152+t155;
t160 = t70+t72+t82+t84+t87+t103+t114+t119+t120+t125+t131+t149+t150+t154;
t161 = t62.*t160;
t162 = -t161;
mt1 = [1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,-I_bzz-t72.*2.0-t103-t125.*2.0+t129+t130-I_hxx.*t31+b.*t58+b.*t59+b.*t60+b.*t113.*2.0+t32.*t63.*2.0-t63.*t70-a.*t2.*t7.*2.0-a.*t6.*t62.*2.0+e.*t29.*t61+f.*t3.*t7.*2.0-m_f.*t18.*t63-m_h.*t18.*t29-m_h.*t22.*t31,t157,t163,t162,0.0,0.0,t157,-m_b-m_f-m_h-m_r,t158,t156,0.0,0.0,t163,t158];
mt2 = [-I_bxx-t129-I_hxx.*t29-I_hzz.*t31-h.*t9-rho_f.*t11+rho_r.*t9.*2.0+rho_r.*t11.*2.0+rho_r.*t64+rho_r.*t65+rho_r.*t66+t3.*t47.*2.0+t3.*t48.*2.0-t11.*t_f.*2.0+t12.*t_f.*2.0-t14.*t_f+t9.*t_r.*2.0-t10.*t_r.*2.0+t11.*t_r.*2.0-t12.*t_r.*2.0-t13.*t_r.*2.0+t14.*t_r.*2.0+t67.*t_r+t68.*t_r+t69.*t_r+a.*t3.*t13.*2.0+a.*t3.*t17.*2.0-a.*t8.*t31.*2.0+e.*t31.*t61+f.*t2.*t13.*2.0+f.*t2.*t17.*2.0+f.*t30.*t61-m_h.*t18.*t31-m_h.*t22.*t29,t159,0.0,0.0,t162,t156,t159,-I_hzz-t70+t83.*2.0+t85.*2.0+e.*t61-rho_f.*t11.*t31-t11.*t31.*t_f.*2.0-t14.*t31.*t_f];
M = reshape([mt1,mt2],6,6);
end
