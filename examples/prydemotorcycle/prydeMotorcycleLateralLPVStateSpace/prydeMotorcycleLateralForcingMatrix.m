function H = prydeMotorcycleLateralForcingMatrix(in1)
%prydeMotorcycleLateralForcingMatrix
%    H = prydeMotorcycleLateralForcingMatrix(IN1)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    31-Mar-2025 17:35:37

%   states = [varphi delta omega_bz v_ry omega_bx omega_delta]
%   inputs = [tau_r tau_delta]
%   params = [C_delta I_bxx I_bxz I_bzz I_hxx I_hzz K_xkf K_xkr K_yaf K_ycf K_ygf K_yar K_ycr K_ygr K_yvf K_yvr K_zaf K_zgf K_zar K_zgr V a a_n b e f g h i_fy i_ry m_b m_h rho_f rho_r t_f t_r varepsilon]
%
C_delta = in1(1,:);
K_xkf = in1(7,:);
K_xkr = in1(8,:);
K_yaf = in1(9,:);
K_ycf = in1(10,:);
K_ygf = in1(11,:);
K_yar = in1(12,:);
K_ycr = in1(13,:);
K_ygr = in1(14,:);
K_yvf = in1(15,:);
K_yvr = in1(16,:);
K_zaf = in1(17,:);
K_zgf = in1(18,:);
K_zar = in1(19,:);
K_zgr = in1(20,:);
V = in1(21,:);
a = in1(22,:);
a_n = in1(23,:);
b = in1(24,:);
e = in1(25,:);
f = in1(26,:);
g = in1(27,:);
h = in1(28,:);
i_fy = in1(29,:);
i_ry = in1(30,:);
m_b = in1(31,:);
m_h = in1(32,:);
rho_f = in1(33,:);
rho_r = in1(34,:);
t_f = in1(35,:);
t_r = in1(36,:);
varepsilon = in1(37,:);
t2 = cos(varepsilon);
t3 = sin(varepsilon);
t4 = K_xkf.*a;
t5 = K_xkr.*a;
t6 = K_yaf.*a;
t7 = K_xkf.*a_n;
t8 = K_xkr.*a_n;
t9 = K_yaf.*a_n;
t10 = K_yaf.*rho_r;
t11 = K_yar.*rho_r;
t12 = K_yaf.*t_f;
t13 = K_yaf.*t_r;
t14 = rho_f+t_f;
t15 = rho_r+t_r;
t16 = V.^2;
t17 = a.^2;
t18 = a_n.^2;
t19 = a_n.^3;
t20 = b.^2;
t21 = rho_r.^2;
t22 = t_f.^2;
t23 = t_r.^2;
t24 = varepsilon.*2.0;
t25 = K_yaf+K_yar;
t35 = 1.0./V;
t37 = -a_n;
t51 = K_yvr.*V.*a.*g.*m_b;
t52 = K_yvr.*V.*a.*g.*m_h;
t53 = K_yvr.*V.*a_n.*g.*m_b;
t54 = K_yvr.*V.*a_n.*g.*m_h;
t60 = a.*a_n.*e.*g.*m_h.*2.0;
t26 = t2.^2;
t27 = t2.^3;
t29 = t2.^5;
t30 = sin(t24);
t31 = t3.^2;
t32 = t3.^3;
t33 = K_zaf.*t2;
t34 = b.*t2;
t36 = t3.*t_f;
t38 = -t7;
t39 = -t9;
t40 = -t12;
t41 = 1.0./t2;
t42 = a_n.*t9;
t47 = rho_f.*rho_r.*t9;
t48 = rho_r.*t9.*t_f;
t49 = rho_f.*t9.*t_r;
t50 = t9.*t_f.*t_r;
t55 = 1.0./t14;
t56 = 1.0./t15;
t57 = i_fy.*t2.*t16;
t58 = e.*g.*m_h.*t17;
t59 = e.*g.*m_h.*t18;
t77 = t3.*t60;
t78 = a.*f.*g.*m_h.*t2.*t_f.*3.0;
t79 = a_n.*f.*g.*m_h.*t2.*t_f.*3.0;
t28 = t26.^2;
t44 = K_xkf.*t34;
t45 = K_xkr.*t34;
t46 = K_yaf.*t34;
t61 = -t32;
t62 = t33.*t37;
t63 = K_yvf.*V.*g.*m_b.*t34;
t64 = K_yvf.*V.*g.*m_h.*t34;
t65 = a.*a_n.*g.*m_b.*t34;
t66 = a.*a_n.*g.*m_h.*t34;
t67 = a.*g.*m_b.*t34.*t_f;
t68 = a.*g.*m_h.*t34.*t_f;
t69 = a_n.*g.*m_b.*t34.*t_f;
t70 = a_n.*g.*m_h.*t34.*t_f;
t71 = g.*m_b.*t18.*t34;
t72 = g.*m_h.*t18.*t34;
t73 = a.*e.*g.*m_h.*t34.*2.0;
t74 = a_n.*e.*g.*m_h.*t34.*2.0;
t75 = t3.*t58;
t76 = t3.*t59;
t80 = a+t34+t37;
t81 = K_yvf.*V.*a.*g.*m_h.*t26;
t82 = t26.*t52;
t83 = K_yvf.*V.*e.*g.*m_h.*t26;
t84 = K_yvr.*V.*e.*g.*m_h.*t26;
t85 = a.*a_n.*b.*g.*m_h.*t27;
t86 = a.*a_n.*e.*g.*m_h.*t26;
t87 = a_n.*b.*e.*g.*m_h.*t27;
t88 = a.*a_n.*g.*m_h.*t26.*t_f;
t89 = a.*b.*e.*g.*m_h.*t30;
t90 = a_n.*b.*e.*g.*m_h.*t30;
t91 = a.*e.*g.*m_h.*t26.*t_f;
t92 = a_n.*e.*g.*m_h.*t26.*t_f;
t93 = b.*e.*g.*m_h.*t27.*t_f;
t95 = K_ycf.*a.*g.*m_h.*rho_f.*t27.*t_f;
t96 = K_ycf.*a.*g.*m_h.*rho_r.*t27.*t_f;
t97 = K_ycf.*b.*g.*m_b.*rho_f.*t26.*t_f;
t98 = K_ycf.*b.*g.*m_b.*rho_r.*t26.*t_f;
t99 = K_ycf.*b.*g.*m_h.*rho_f.*t26.*t_f;
t100 = K_ycf.*b.*g.*m_h.*rho_r.*t26.*t_f;
t101 = K_ycf.*e.*g.*m_h.*rho_f.*t27.*t_f;
t102 = K_ycf.*e.*g.*m_h.*rho_r.*t27.*t_f;
t103 = K_ycf.*f.*g.*m_h.*rho_f.*t26.*t_f;
t105 = K_ycf.*f.*g.*m_h.*rho_r.*t26.*t_f;
t107 = K_ycf.*f.*g.*m_h.*t26.*t_f.*t_r;
t109 = a.*g.*m_h.*t18.*t26;
t110 = a_n.*g.*m_h.*t17.*t26;
t111 = a_n.*g.*m_b.*t20.*t26;
t112 = a_n.*g.*m_h.*t20.*t26;
t113 = t26.*t59;
t114 = e.*g.*m_h.*t20.*t26;
t115 = g.*m_h.*t17.*t26.*t_f;
t116 = g.*m_b.*t20.*t26.*t_f;
t117 = g.*m_h.*t20.*t26.*t_f;
t118 = a.*f.*g.*m_h.*t27.*t_f.*3.0;
t119 = a_n.*f.*g.*m_h.*t27.*t_f.*3.0;
t120 = b.*f.*g.*m_h.*t26.*t_f.*3.0;
t122 = K_ycf.*a.*b.*g.*m_b.*rho_f.*t26;
t123 = K_ycf.*a.*b.*g.*m_h.*rho_f.*t26;
t125 = K_ycf.*a.*e.*g.*m_h.*rho_f.*t27;
t127 = K_yvf.*V.*a.*b.*g.*m_b.*rho_f.*t26;
t130 = K_yvf.*V.*a.*e.*g.*m_h.*rho_f.*t27;
t132 = K_yvf.*V.*a.*g.*m_h.*rho_f.*t27.*t_f;
t133 = K_yvf.*V.*a.*g.*m_h.*rho_r.*t27.*t_f;
t134 = K_yvf.*V.*b.*g.*m_b.*rho_f.*t26.*t_f;
t135 = K_yvf.*V.*b.*g.*m_b.*rho_r.*t26.*t_f;
t136 = K_yvf.*V.*b.*g.*m_h.*rho_f.*t26.*t_f;
t137 = K_yvf.*V.*b.*g.*m_h.*rho_r.*t26.*t_f;
t138 = K_yvf.*V.*e.*g.*m_h.*rho_f.*t27.*t_f;
t139 = K_yvf.*V.*e.*g.*m_h.*rho_r.*t27.*t_f;
t140 = K_yvf.*V.*f.*g.*m_h.*rho_f.*t26.*t_f;
t142 = K_yvf.*V.*f.*g.*m_h.*rho_r.*t26.*t_f;
t144 = K_yvf.*V.*f.*g.*m_h.*t26.*t_f.*t_r;
t146 = a.*b.*g.*m_h.*t27.*t36;
t147 = b.*e.*g.*m_h.*t27.*t36;
t148 = K_ycf.*g.*m_h.*rho_f.*t17.*t27;
t149 = K_ycf.*g.*m_b.*rho_f.*t20.*t27;
t150 = K_ycf.*g.*m_h.*rho_f.*t20.*t27;
t151 = K_ycf.*a.*g.*m_h.*t22.*t27;
t152 = K_ycf.*b.*g.*m_b.*t22.*t26;
t153 = K_ycf.*b.*g.*m_h.*t22.*t26;
t154 = K_ycf.*e.*g.*m_h.*t22.*t27;
t155 = K_ycf.*f.*g.*m_h.*t22.*t26;
t157 = K_yvf.*V.*g.*m_h.*rho_f.*t17.*t27;
t158 = K_yvf.*V.*g.*m_b.*rho_f.*t20.*t27;
t159 = K_yvf.*V.*g.*m_h.*rho_f.*t20.*t27;
t160 = K_yvf.*V.*a.*g.*m_h.*t22.*t27;
t161 = K_yvf.*V.*b.*g.*m_b.*t22.*t26;
t162 = K_yvf.*V.*b.*g.*m_h.*t22.*t26;
t163 = K_yvf.*V.*e.*g.*m_h.*t22.*t27;
t164 = K_yvf.*V.*f.*g.*m_h.*t22.*t26;
t168 = (K_yvf.*V.*f.*g.*m_h.*t30)./2.0;
t169 = (K_yvr.*V.*f.*g.*m_h.*t30)./2.0;
t170 = a.*f.*g.*m_h.*t30.*t_f.*(3.0./2.0);
t171 = a_n.*f.*g.*m_h.*t30.*t_f.*(3.0./2.0);
t172 = K_ycf.*b.*f.*g.*m_h.*rho_f.*t3.*t27;
t173 = K_ycf.*a.*g.*m_h.*rho_f.*t27.*t36;
t174 = K_ycf.*a.*g.*m_h.*rho_r.*t27.*t36;
t175 = K_ycf.*a.*g.*m_h.*t27.*t36.*t_r;
t176 = K_ycf.*e.*g.*m_h.*rho_f.*t27.*t36;
t177 = K_ycf.*e.*g.*m_h.*rho_r.*t27.*t36;
t178 = K_ycf.*e.*g.*m_h.*t27.*t36.*t_r;
t179 = K_yvf.*V.*b.*f.*g.*m_h.*rho_f.*t3.*t27;
t180 = K_yvf.*V.*a.*g.*m_h.*rho_f.*t27.*t36;
t181 = K_yvf.*V.*a.*g.*m_h.*rho_r.*t27.*t36;
t182 = K_yvf.*V.*a.*g.*m_h.*t27.*t36.*t_r;
t183 = K_yvf.*V.*e.*g.*m_h.*rho_f.*t27.*t36;
t184 = K_yvf.*V.*e.*g.*m_h.*rho_r.*t27.*t36;
t185 = K_yvf.*V.*e.*g.*m_h.*t27.*t36.*t_r;
t190 = t10+t11+t13+t40;
t94 = t3+t61;
t104 = K_ycf.*f.*g.*m_h.*rho_f.*t28.*t_f;
t106 = K_ycf.*f.*g.*m_h.*rho_r.*t28.*t_f;
t108 = K_ycf.*f.*g.*m_h.*t28.*t_f.*t_r;
t121 = b.*f.*g.*m_h.*t28.*t_f.*3.0;
t124 = K_ycf.*a.*b.*g.*m_h.*rho_f.*t28;
t126 = K_ycf.*b.*e.*g.*m_h.*rho_f.*t28;
t128 = b.*rho_f.*t81;
t129 = K_yvf.*V.*a.*b.*g.*m_h.*rho_f.*t28;
t131 = K_yvf.*V.*b.*e.*g.*m_h.*rho_f.*t28;
t141 = K_yvf.*V.*f.*g.*m_h.*rho_f.*t28.*t_f;
t143 = K_yvf.*V.*f.*g.*m_h.*rho_r.*t28.*t_f;
t145 = K_yvf.*V.*f.*g.*m_h.*t28.*t_f.*t_r;
t156 = K_ycf.*f.*g.*m_h.*t22.*t28;
t165 = K_yvf.*V.*f.*g.*m_h.*t22.*t28;
t166 = 1.0./t80;
t186 = t3.*t151;
t187 = t3.*t154;
t188 = t3.*t160;
t189 = t3.*t163;
t167 = t166.^2;
t191 = e.*g.*m_h.*t20.*t94;
t192 = b.*f.*g.*m_h.*t94.*t_f.*3.0;
t193 = K_ycf.*f.*g.*m_h.*rho_f.*t94.*t_f;
t194 = K_ycf.*f.*g.*m_h.*rho_r.*t94.*t_f;
t195 = K_yvf.*V.*f.*g.*m_h.*rho_f.*t94.*t_f;
t196 = K_yvf.*V.*f.*g.*m_h.*rho_r.*t94.*t_f;
t197 = K_ycf.*f.*g.*m_h.*t22.*t94;
t198 = K_yvf.*V.*f.*g.*m_h.*t22.*t94;
et1 = K_ygf.*t17+K_ygf.*t18-K_ygf.*a.*a_n.*2.0+K_ygf.*a.*t34.*2.0+K_zgf.*a.*t2+K_zgr.*a.*t2-K_ygf.*a_n.*t34.*2.0+K_zgf.*b.*t26+K_zgr.*b.*t26+K_ygf.*t20.*t26+K_zgf.*t2.*t37+K_zgr.*t2.*t37+rho_r.*t2.*t51+rho_r.*t2.*t52-rho_r.*t27.*t52+K_ycr.*a.*g.*m_b.*rho_r.*t2+K_ycr.*a.*g.*m_h.*rho_r.*t2+K_ycf.*a.*g.*m_h.*rho_r.*t27-K_ycr.*a.*g.*m_h.*rho_r.*t27+K_ycf.*b.*g.*m_b.*rho_r.*t26+K_ycf.*b.*g.*m_h.*rho_r.*t26-K_ycf.*a.*g.*m_h.*t27.*t_f+K_ycf.*a.*g.*m_h.*t27.*t_r-K_ycf.*b.*g.*m_b.*t26.*t_f-K_ycf.*b.*g.*m_h.*t26.*t_f+K_ycf.*b.*g.*m_b.*t26.*t_r+K_ycf.*b.*g.*m_h.*t26.*t_r+K_ycf.*e.*g.*m_h.*rho_r.*t27-K_ycr.*e.*g.*m_h.*rho_r.*t27-K_ycf.*e.*g.*m_h.*t27.*t_f+K_ycf.*e.*g.*m_h.*t27.*t_r-K_ycf.*f.*g.*m_h.*t26.*t36;
et2 = K_ycr.*g.*m_b.*rho_r.*t2.*t37+K_ycr.*g.*m_h.*rho_r.*t2.*t37+K_yvf.*V.*a.*g.*m_h.*rho_r.*t27+K_yvf.*V.*b.*g.*m_b.*rho_r.*t26+K_yvf.*V.*b.*g.*m_h.*rho_r.*t26-K_yvf.*V.*a.*g.*m_h.*t27.*t_f+K_yvf.*V.*a.*g.*m_h.*t27.*t_r-K_yvf.*V.*b.*g.*m_b.*t26.*t_f-K_yvf.*V.*b.*g.*m_h.*t26.*t_f+K_yvf.*V.*b.*g.*m_b.*t26.*t_r+K_yvf.*V.*b.*g.*m_h.*t26.*t_r+K_yvf.*V.*e.*g.*m_h.*rho_r.*t27-K_yvr.*V.*e.*g.*m_h.*rho_r.*t27-K_yvf.*V.*e.*g.*m_h.*t27.*t_f+K_yvf.*V.*e.*g.*m_h.*t27.*t_r-K_yvf.*V.*f.*g.*m_h.*t26.*t36+K_yvr.*V.*g.*m_b.*rho_r.*t2.*t37+K_yvr.*V.*g.*m_h.*rho_r.*t2.*t37+K_ycf.*f.*g.*m_h.*rho_r.*t3.*t26-K_ycr.*f.*g.*m_h.*rho_r.*t3.*t26+K_ycf.*f.*g.*m_h.*t3.*t26.*t_r+K_yvf.*V.*f.*g.*m_h.*rho_r.*t3.*t26-K_yvr.*V.*f.*g.*m_h.*rho_r.*t3.*t26;
et3 = K_yvf.*V.*f.*g.*m_h.*t3.*t26.*t_r;
et4 = b.*t81-K_zaf.*a.*t26+(K_zgf.*a.*t30)./2.0+K_zaf.*a_n.*t26-(K_zgf.*a_n.*t30)./2.0-K_zaf.*b.*t27+K_zgf.*b.*t94+K_ygf.*t3.*t17+K_ygf.*t3.*t18-K_yaf.*t20.*t27+K_ygf.*t20.*t94-a.*t2.*t6+a_n.*t2.*t6.*2.0-b.*t6.*t26.*2.0+b.*t9.*t26.*2.0+t2.*t9.*t37-K_ygf.*a.*a_n.*t3.*2.0+K_ygf.*a.*b.*t30+K_ygf.*b.*t30.*t37+K_ycf.*g.*m_b.*t20.*t27+K_ycf.*g.*m_h.*t17.*t27+K_ycf.*g.*m_h.*t20.*t27+K_ycf.*a.*b.*g.*m_b.*t26+K_ycf.*a.*b.*g.*m_h.*t26+K_ycf.*a.*b.*g.*m_h.*t28+K_ycf.*a.*e.*g.*m_h.*t27+K_ycf.*b.*e.*g.*m_h.*t28+K_ycf.*a.*f.*g.*m_h.*t94+K_yvf.*V.*g.*m_b.*t20.*t27+K_yvf.*V.*g.*m_h.*t17.*t27+K_yvf.*V.*g.*m_h.*t20.*t27-K_ycf.*a.*g.*m_h.*t27.*t36-K_ycf.*b.*g.*m_b.*t94.*t_f;
et5 = -K_ycf.*b.*g.*m_h.*t94.*t_f-K_ycf.*e.*g.*m_h.*t27.*t36-K_ycf.*f.*g.*m_h.*t26.*t_f+K_ycf.*f.*g.*m_h.*t28.*t_f+K_yvf.*V.*a.*b.*g.*m_b.*t26+K_yvf.*V.*a.*b.*g.*m_h.*t28+K_yvf.*V.*a.*e.*g.*m_h.*t27+K_yvf.*V.*b.*e.*g.*m_h.*t28+K_yvf.*V.*a.*f.*g.*m_h.*t94-K_yvf.*V.*a.*g.*m_h.*t27.*t36-K_yvf.*V.*b.*g.*m_b.*t94.*t_f-K_yvf.*V.*b.*g.*m_h.*t94.*t_f-K_yvf.*V.*e.*g.*m_h.*t27.*t36-K_yvf.*V.*f.*g.*m_h.*t26.*t_f+K_yvf.*V.*f.*g.*m_h.*t28.*t_f+K_ycf.*b.*f.*g.*m_h.*t3.*t27+K_yvf.*V.*b.*f.*g.*m_h.*t3.*t27;
et6 = -t67-t68+t69+t70+t75+t76+t88+t89-t91+t92-t93+t95-t96+t97-t98+t99-t100+t101-t102-t115-t116-t117+t132-t133+t134-t135+t136-t137+t138-t139+t151+t152+t153+t154+t160+t161+t162+t163+t170-t171+t191+t192+t193-t194+t195-t196+t197+t198+K_ygf.*rho_r.*t17+K_ygf.*rho_r.*t18+K_ygr.*rho_r.*t17+K_ygr.*rho_r.*t18+K_ygf.*t17.*t_r+K_ygf.*t18.*t_r+K_ygr.*t17.*t_r+K_ygr.*t18.*t_r+K_ygf.*rho_r.*t20.*t26+K_ygr.*rho_r.*t20.*t26+K_ygf.*t20.*t26.*t_r+K_ygr.*t20.*t26.*t_r+g.*h.*m_b.*t17+g.*h.*m_b.*t18-g.*m_b.*t17.*t_r-g.*m_b.*t18.*t_r-g.*m_h.*t17.*t_r-g.*m_h.*t18.*t_r+a.^3.*g.*m_h.*t3;
et7 = K_ygf.*a.*a_n.*rho_r.*-2.0-K_ygr.*a.*a_n.*rho_r.*2.0-K_ygf.*a.*a_n.*t_r.*2.0-K_ygr.*a.*a_n.*t_r.*2.0+K_ygf.*a.*rho_r.*t34.*2.0+K_ygr.*a.*rho_r.*t34.*2.0-K_ygf.*a_n.*rho_r.*t34.*2.0-K_ygr.*a_n.*rho_r.*t34.*2.0+K_ygf.*a.*t34.*t_r.*2.0+K_ygr.*a.*t34.*t_r.*2.0-K_ygf.*a_n.*t34.*t_r.*2.0-K_ygr.*a_n.*t34.*t_r.*2.0-a.*a_n.*g.*h.*m_b.*2.0+a.*a_n.*g.*m_b.*t_r.*2.0+a.*a_n.*g.*m_h.*t_r.*2.0+a.*g.*h.*m_b.*t34.*2.0-a_n.*g.*h.*m_b.*t34.*2.0+a.*g.*m_h.*t3.*t18+a.*g.*m_h.*t20.*t94-a.*g.*m_b.*t34.*t_r-a_n.*g.*m_h.*t3.*t17.*2.0-a.*g.*m_h.*t34.*t_r+a_n.*g.*m_b.*t34.*t_r+a_n.*g.*m_h.*t34.*t_r+b.*g.*m_h.*t17.*t30+f.*g.*m_h.*t2.*t17+f.*g.*m_h.*t2.*t18+f.*g.*m_h.*t20.*t27+g.*h.*m_b.*t20.*t26;
et8 = g.*m_h.*t17.*t26.*t_r+K_ycf.*a.*g.*m_h.*t23.*t27+K_ycf.*b.*g.*m_b.*t23.*t26+K_ycf.*b.*g.*m_h.*t23.*t26+K_ycf.*e.*g.*m_h.*t23.*t27+K_ycf.*f.*g.*m_h.*t23.*t94-a.*a_n.*e.*g.*m_h.*t3.*2.0-a.*a_n.*f.*g.*m_h.*t2.*2.0+a.*b.*f.*g.*m_h.*t26.*2.0-a_n.*b.*f.*g.*m_h.*t26.*2.0+a.*b.*g.*m_h.*t30.*t37-a.*b.*g.*m_h.*t27.*t_f+a.*b.*g.*m_h.*t27.*t_r+a.*e.*g.*m_h.*t26.*t_r+b.*e.*g.*m_h.*t30.*t37+b.*e.*g.*m_h.*t27.*t_r-a.*f.*g.*m_h.*t30.*t_r.*(3.0./2.0)+a_n.*f.*g.*m_h.*t30.*t_r.*(3.0./2.0)-b.*f.*g.*m_h.*t94.*t_r.*3.0+a.*g.*m_h.*t26.*t37.*t_r+e.*g.*m_h.*t26.*t37.*t_r+K_yvf.*V.*a.*g.*m_h.*t23.*t27+K_yvf.*V.*b.*g.*m_b.*t23.*t26+K_yvf.*V.*b.*g.*m_h.*t23.*t26+K_yvf.*V.*e.*g.*m_h.*t23.*t27;
et9 = K_yvf.*V.*f.*g.*m_h.*t23.*t94-K_ycf.*a.*g.*m_h.*rho_f.*t27.*t_r+K_ycf.*a.*g.*m_h.*rho_r.*t27.*t_r-K_ycf.*b.*g.*m_b.*rho_f.*t26.*t_r+K_ycf.*b.*g.*m_b.*rho_r.*t26.*t_r-K_ycf.*b.*g.*m_h.*rho_f.*t26.*t_r+K_ycf.*b.*g.*m_h.*rho_r.*t26.*t_r-K_ycf.*a.*g.*m_h.*t27.*t_f.*t_r.*2.0-K_ycf.*b.*g.*m_b.*t26.*t_f.*t_r.*2.0-K_ycf.*b.*g.*m_h.*t26.*t_f.*t_r.*2.0-K_ycf.*e.*g.*m_h.*rho_f.*t27.*t_r+K_ycf.*e.*g.*m_h.*rho_r.*t27.*t_r-K_ycf.*f.*g.*m_h.*rho_f.*t94.*t_r+K_ycf.*f.*g.*m_h.*rho_r.*t94.*t_r-K_ycf.*e.*g.*m_h.*t27.*t_f.*t_r.*2.0-K_ycf.*f.*g.*m_h.*t94.*t_f.*t_r.*2.0-K_yvf.*V.*a.*g.*m_h.*rho_f.*t27.*t_r+K_yvf.*V.*a.*g.*m_h.*rho_r.*t27.*t_r-K_yvf.*V.*b.*g.*m_b.*rho_f.*t26.*t_r+K_yvf.*V.*b.*g.*m_b.*rho_r.*t26.*t_r-K_yvf.*V.*b.*g.*m_h.*rho_f.*t26.*t_r;
et10 = K_yvf.*V.*b.*g.*m_h.*rho_r.*t26.*t_r-K_yvf.*V.*a.*g.*m_h.*t27.*t_f.*t_r.*2.0-K_yvf.*V.*b.*g.*m_b.*t26.*t_f.*t_r.*2.0-K_yvf.*V.*b.*g.*m_h.*t26.*t_f.*t_r.*2.0-K_yvf.*V.*e.*g.*m_h.*rho_f.*t27.*t_r+K_yvf.*V.*e.*g.*m_h.*rho_r.*t27.*t_r-K_yvf.*V.*f.*g.*m_h.*rho_f.*t94.*t_r+K_yvf.*V.*f.*g.*m_h.*rho_r.*t94.*t_r-K_yvf.*V.*e.*g.*m_h.*t27.*t_f.*t_r.*2.0-K_yvf.*V.*f.*g.*m_h.*t94.*t_f.*t_r.*2.0;
et11 = t58+t59-t60+t65+t66-t71-t72+t73-t74+t78-t79+t85+t86+t87+t103-t104-t105+t106-t107+t108-t109+t110+t111+t112-t113+t114-t118+t119+t120-t121-t122-t123-t124-t125-t126-t127-t128-t129-t130-t131+t140-t141-t142+t143-t144+t145-t146-t147-t148-t149-t150+t155-t156-t157-t158-t159+t164-t165-t172+t173-t174-t175+t176-t177-t178-t179+t180-t181-t182+t183;
et12 = -t184-t185+t186+t187+t188+t189+b.*rho_r.*t81-b.*t81.*t_f+b.*t81.*t_r-t10.*t20.*t27-t13.*t20.*t27+K_ygf.*rho_r.*t3.*t17+K_ygf.*rho_r.*t3.*t18+K_ygf.*rho_r.*t20.*t94+K_ygf.*t3.*t17.*t_r+K_ygf.*t3.*t18.*t_r+K_ygf.*t20.*t94.*t_r-a.*rho_r.*t2.*t6+a_n.*rho_r.*t2.*t6.*2.0-b.*rho_r.*t6.*t26.*2.0+b.*rho_r.*t9.*t26.*2.0-a.*t2.*t6.*t_r+a_n.*t2.*t6.*t_r.*2.0-b.*t6.*t26.*t_r.*2.0+b.*t9.*t26.*t_r.*2.0+rho_r.*t2.*t9.*t37+t2.*t9.*t37.*t_r-K_ygf.*a.*a_n.*rho_r.*t3.*2.0+K_ygf.*a.*b.*rho_r.*t30-K_ygf.*a.*a_n.*t3.*t_r.*2.0+K_ygf.*a.*b.*t30.*t_r+K_ygf.*b.*rho_r.*t30.*t37+K_ygf.*b.*t30.*t37.*t_r+f.*g.*m_h.*t18.*t30.*(3.0./2.0)-g.*m_b.*t20.*t94.*t_f-g.*m_h.*t17.*t94.*t_f;
et13 = -g.*m_h.*t20.*t94.*t_f+K_ycf.*b.*g.*m_b.*t22.*t94+K_ycf.*b.*g.*m_h.*t22.*t94-a.*a_n.*f.*g.*m_h.*t30.*(3.0./2.0)-a_n.*b.*f.*g.*m_h.*t94.*3.0+K_ycf.*g.*m_b.*rho_r.*t20.*t27+K_ycf.*g.*m_h.*rho_r.*t17.*t27+K_ycf.*g.*m_h.*rho_r.*t20.*t27-K_ycf.*g.*m_b.*t20.*t27.*t_f-K_ycf.*g.*m_h.*t17.*t27.*t_f-K_ycf.*g.*m_h.*t20.*t27.*t_f+K_ycf.*g.*m_b.*t20.*t27.*t_r+K_ycf.*g.*m_h.*t17.*t27.*t_r+K_ycf.*g.*m_h.*t20.*t27.*t_r+a.*a_n.*g.*m_h.*t94.*t_f-(a.*b.*g.*m_b.*t30.*t_f)./2.0-(a.*b.*g.*m_h.*t30.*t_f)./2.0+(a_n.*b.*g.*m_b.*t30.*t_f)./2.0+(a_n.*b.*g.*m_h.*t30.*t_f)./2.0-a.*e.*g.*m_h.*t94.*t_f+a_n.*e.*g.*m_h.*t94.*t_f+K_yvf.*V.*b.*g.*m_b.*t22.*t94+K_yvf.*V.*b.*g.*m_h.*t22.*t94+K_ycf.*a.*b.*g.*m_b.*rho_r.*t26;
et14 = K_ycf.*a.*b.*g.*m_h.*rho_r.*t26+K_ycf.*a.*b.*g.*m_h.*rho_r.*t28-K_ycf.*a.*b.*g.*m_b.*t26.*t_f-K_ycf.*a.*b.*g.*m_h.*t26.*t_f-K_ycf.*a.*b.*g.*m_h.*t28.*t_f+K_ycf.*a.*b.*g.*m_b.*t26.*t_r+K_ycf.*a.*b.*g.*m_h.*t26.*t_r+K_ycf.*a.*b.*g.*m_h.*t28.*t_r+K_ycf.*a.*e.*g.*m_h.*rho_r.*t27+K_ycf.*b.*e.*g.*m_h.*rho_r.*t28-K_ycf.*a.*f.*g.*m_h.*rho_f.*t94+K_ycf.*a.*f.*g.*m_h.*rho_r.*t94-K_ycf.*a.*e.*g.*m_h.*t27.*t_f+K_ycf.*a.*e.*g.*m_h.*t27.*t_r-K_ycf.*b.*e.*g.*m_h.*t28.*t_f-K_ycf.*a.*f.*g.*m_h.*t94.*t_f+K_ycf.*b.*e.*g.*m_h.*t28.*t_r+K_ycf.*a.*f.*g.*m_h.*t94.*t_r-K_ycf.*b.*f.*g.*m_h.*t27.*t36+K_yvf.*V.*g.*m_b.*rho_r.*t20.*t27+K_yvf.*V.*g.*m_h.*rho_r.*t17.*t27+K_yvf.*V.*g.*m_h.*rho_r.*t20.*t27;
et15 = -K_yvf.*V.*g.*m_b.*t20.*t27.*t_f-K_yvf.*V.*g.*m_h.*t17.*t27.*t_f-K_yvf.*V.*g.*m_h.*t20.*t27.*t_f+K_yvf.*V.*g.*m_b.*t20.*t27.*t_r+K_yvf.*V.*g.*m_h.*t17.*t27.*t_r+K_yvf.*V.*g.*m_h.*t20.*t27.*t_r+K_ycf.*b.*g.*m_b.*rho_f.*t94.*t_f-K_ycf.*b.*g.*m_b.*rho_r.*t94.*t_f+K_ycf.*b.*g.*m_h.*rho_f.*t94.*t_f-K_ycf.*b.*g.*m_h.*rho_r.*t94.*t_f-K_ycf.*b.*g.*m_b.*t94.*t_f.*t_r-K_ycf.*b.*g.*m_h.*t94.*t_f.*t_r+K_yvf.*V.*a.*b.*g.*m_b.*rho_r.*t26+K_yvf.*V.*a.*b.*g.*m_h.*rho_r.*t28-K_yvf.*V.*a.*b.*g.*m_b.*t26.*t_f-K_yvf.*V.*a.*b.*g.*m_h.*t28.*t_f+K_yvf.*V.*a.*b.*g.*m_b.*t26.*t_r+K_yvf.*V.*a.*b.*g.*m_h.*t28.*t_r+K_yvf.*V.*a.*e.*g.*m_h.*rho_r.*t27+K_yvf.*V.*b.*e.*g.*m_h.*rho_r.*t28-K_yvf.*V.*a.*f.*g.*m_h.*rho_f.*t94;
et16 = K_yvf.*V.*a.*f.*g.*m_h.*rho_r.*t94-K_yvf.*V.*a.*e.*g.*m_h.*t27.*t_f+K_yvf.*V.*a.*e.*g.*m_h.*t27.*t_r-K_yvf.*V.*b.*e.*g.*m_h.*t28.*t_f-K_yvf.*V.*a.*f.*g.*m_h.*t94.*t_f+K_yvf.*V.*b.*e.*g.*m_h.*t28.*t_r+K_yvf.*V.*a.*f.*g.*m_h.*t94.*t_r-K_yvf.*V.*b.*f.*g.*m_h.*t27.*t36+K_yvf.*V.*b.*g.*m_b.*rho_f.*t94.*t_f-K_yvf.*V.*b.*g.*m_b.*rho_r.*t94.*t_f+K_yvf.*V.*b.*g.*m_h.*rho_f.*t94.*t_f-K_yvf.*V.*b.*g.*m_h.*rho_r.*t94.*t_f-K_yvf.*V.*b.*g.*m_b.*t94.*t_f.*t_r-K_yvf.*V.*b.*g.*m_h.*t94.*t_f.*t_r+K_ycf.*b.*f.*g.*m_h.*rho_r.*t3.*t27+K_ycf.*b.*f.*g.*m_h.*t3.*t27.*t_r+K_yvf.*V.*b.*f.*g.*m_h.*rho_r.*t3.*t27+K_yvf.*V.*b.*f.*g.*m_h.*t3.*t27.*t_r;
et17 = rho_r.*t57+t47.*t_r.*2.0+t48.*t_r.*2.0+t57.*t_r-rho_f.*t6.*t21-rho_f.*t6.*t23+rho_f.*t9.*t21+rho_f.*t9.*t23+t23.*t34.*t40-t6.*t21.*t_f-t6.*t23.*t_f+t9.*t21.*t_f+t9.*t23.*t_f+i_ry.*rho_f.*t2.*t16+i_ry.*t2.*t16.*t_f-rho_f.*rho_r.*t10.*t34-rho_f.*rho_r.*t6.*t_r.*2.0-rho_r.*t10.*t34.*t_f-rho_f.*t10.*t34.*t_r.*2.0-rho_f.*t13.*t34.*t_r-rho_r.*t6.*t_f.*t_r.*2.0-t10.*t34.*t_f.*t_r.*2.0-m_b.*rho_f.*t2.*t16.*t21-m_b.*rho_f.*t2.*t16.*t23-m_h.*rho_f.*t2.*t16.*t21-m_h.*rho_f.*t2.*t16.*t23-m_b.*t2.*t16.*t21.*t_f-m_b.*t2.*t16.*t23.*t_f-m_h.*t2.*t16.*t21.*t_f-m_h.*t2.*t16.*t23.*t_f+(e.*m_h.*rho_r.*t16.*t30.*t_f)./2.0;
et18 = (e.*m_h.*rho_f.*t16.*t30.*t_r)./2.0+f.*m_h.*rho_r.*t16.*t26.*t_f+f.*m_h.*rho_f.*t16.*t26.*t_r+h.*m_b.*rho_f.*rho_r.*t2.*t16+(e.*m_h.*t16.*t30.*t_f.*t_r)./2.0+f.*m_h.*t16.*t26.*t_f.*t_r+h.*m_b.*rho_r.*t2.*t16.*t_f+h.*m_b.*rho_f.*t2.*t16.*t_r+h.*m_b.*t2.*t16.*t_f.*t_r-m_b.*rho_f.*rho_r.*t2.*t16.*t_r.*2.0-m_h.*rho_f.*rho_r.*t2.*t16.*t_r.*2.0-m_b.*rho_r.*t2.*t16.*t_f.*t_r.*2.0-m_h.*rho_r.*t2.*t16.*t_f.*t_r.*2.0+(a.*m_h.*rho_f.*rho_r.*t16.*t30)./2.0+(a.*m_h.*rho_r.*t16.*t30.*t_f)./2.0+(a.*m_h.*rho_f.*t16.*t30.*t_r)./2.0+(a.*m_h.*t16.*t30.*t_f.*t_r)./2.0+(e.*m_h.*rho_f.*rho_r.*t16.*t30)./2.0+f.*m_h.*rho_f.*rho_r.*t16.*t26;
et19 = -t58-t59+t60+t71+t72-t73+t74-t78+t79-t103+t104+t105-t106+t107-t108+t109+t113-t114+t118-t119-t120+t121+t122+t123+t124+t125+t126+t127+t128+t129+t130+t131-t140+t141+t142-t143+t144-t145+t146+t147+t148+t149+t150-t155+t156+t157+t158+t159-t164+t165+t172-t173+t174+t175-t176+t177+t178+t179-t180+t181+t182-t183+t184+t185-t186-t187-t188-t189-K_ygf.*t19-t3.*t152-t3.*t153-t3.*t161-t3.*t162+K_ygf.*a.*t18.*2.0+K_ygf.*t18.*t34.*2.0+K_ygf.*t17.*t37+K_zgf.*t2.*t17+K_zgf.*t2.*t18+K_zgf.*t20.*t27+K_ygf.*t20.*t26.*t37+f.*rho_f.*t3.*t81;
et20 = K_ygf.*a.*a_n.*t34.*-2.0-K_zgf.*a.*a_n.*t2.*2.0+K_zgf.*a.*b.*t26.*2.0-K_zgf.*a_n.*b.*t26.*2.0+a.*g.*m_b.*t34.*t36+a.*g.*m_b.*t34.*t37+a.*g.*m_h.*t34.*t36+a.*g.*m_h.*t34.*t37+g.*m_b.*t20.*t26.*t36+g.*m_b.*t20.*t26.*t37+g.*m_b.*t34.*t36.*t37+g.*m_h.*t17.*t26.*t36+g.*m_h.*t17.*t26.*t37+g.*m_h.*t20.*t26.*t36+g.*m_h.*t20.*t26.*t37+g.*m_h.*t34.*t36.*t37+a.*b.*g.*m_h.*t27.*t37+a.*e.*g.*m_h.*t26.*t36+a.*e.*g.*m_h.*t26.*t37+b.*e.*g.*m_h.*t27.*t37+a.*g.*m_h.*t26.*t36.*t37+e.*g.*m_h.*t26.*t36.*t37-f.*g.*m_h.*t2.*t3.*t18.*3.0+K_ycf.*a.*a_n.*g.*m_h.*t27.*t_f+K_ycf.*a_n.*b.*g.*m_b.*t26.*t_f+K_ycf.*a_n.*b.*g.*m_h.*t26.*t_f+K_ycf.*a_n.*e.*g.*m_h.*t27.*t_f+K_ycf.*a_n.*f.*g.*m_h.*t26.*t36+K_ycf.*a.*g.*m_h.*rho_r.*t27.*t37;
et21 = -K_ycf.*b.*g.*m_b.*rho_f.*t26.*t36+K_ycf.*b.*g.*m_b.*rho_r.*t26.*t36+K_ycf.*b.*g.*m_b.*rho_r.*t26.*t37-K_ycf.*b.*g.*m_h.*rho_f.*t26.*t36+K_ycf.*b.*g.*m_h.*rho_r.*t26.*t36+K_ycf.*b.*g.*m_h.*rho_r.*t26.*t37+K_ycf.*a.*g.*m_h.*t27.*t37.*t_r+K_ycf.*b.*g.*m_b.*t26.*t36.*t_r+K_ycf.*b.*g.*m_b.*t26.*t37.*t_r+K_ycf.*b.*g.*m_h.*t26.*t36.*t_r+K_ycf.*b.*g.*m_h.*t26.*t37.*t_r+K_ycf.*e.*g.*m_h.*rho_r.*t27.*t37+K_ycf.*e.*g.*m_h.*t27.*t37.*t_r+a.*a_n.*f.*g.*m_h.*t2.*t3.*3.0+a_n.*b.*f.*g.*m_h.*t3.*t26.*3.0+K_yvf.*V.*a.*a_n.*g.*m_h.*t27.*t_f+K_yvf.*V.*a_n.*b.*g.*m_b.*t26.*t_f+K_yvf.*V.*a_n.*b.*g.*m_h.*t26.*t_f+K_yvf.*V.*a_n.*e.*g.*m_h.*t27.*t_f+K_yvf.*V.*a_n.*f.*g.*m_h.*t26.*t36+K_yvf.*V.*a.*g.*m_h.*rho_r.*t27.*t37-K_yvf.*V.*b.*g.*m_b.*rho_f.*t26.*t36+K_yvf.*V.*b.*g.*m_b.*rho_r.*t26.*t36;
et22 = K_yvf.*V.*b.*g.*m_b.*rho_r.*t26.*t37-K_yvf.*V.*b.*g.*m_h.*rho_f.*t26.*t36+K_yvf.*V.*b.*g.*m_h.*rho_r.*t26.*t36+K_yvf.*V.*b.*g.*m_h.*rho_r.*t26.*t37+K_yvf.*V.*a.*g.*m_h.*t27.*t37.*t_r+K_yvf.*V.*b.*g.*m_b.*t26.*t36.*t_r+K_yvf.*V.*b.*g.*m_b.*t26.*t37.*t_r+K_yvf.*V.*b.*g.*m_h.*t26.*t36.*t_r+K_yvf.*V.*b.*g.*m_h.*t26.*t37.*t_r+K_yvf.*V.*e.*g.*m_h.*rho_r.*t27.*t37+K_yvf.*V.*e.*g.*m_h.*t27.*t37.*t_r+K_ycf.*a.*f.*g.*m_h.*rho_f.*t3.*t26+K_ycf.*f.*g.*m_h.*rho_r.*t3.*t26.*t37+K_ycf.*f.*g.*m_h.*t3.*t26.*t37.*t_r+K_yvf.*V.*f.*g.*m_h.*rho_r.*t3.*t26.*t37+K_yvf.*V.*f.*g.*m_h.*t3.*t26.*t37.*t_r;
et23 = t67+t68-t75-t76+t77-t89+t90+t91+t93-t95+t96-t97+t98-t99+t100-t101+t102+t115+t116+t117-t132+t133-t134+t135-t136+t137-t138+t139-t151-t152-t153-t154-t160-t161-t162-t163-t170+t171-t191-t192-t193+t194-t195+t196-t197-t198+t3.*t124+t3.*t125+t3.*t126+t3.*t129+t3.*t130+t3.*t131+t3.*t148+t3.*t149+t3.*t150+t59.*t94+t3.*t156+t3.*t157+t3.*t158+t3.*t159+t3.*t165-K_ygf.*t3.*t19-K_zaf.*t17.*t26-K_zaf.*t18.*t26-K_zaf.*t20.*t28+(K_zgf.*t17.*t30)./2.0;
et24 = (K_zgf.*t18.*t30)./2.0-b.*t26.*t42.*2.0+f.*rho_f.*t81-t2.*t6.*t18.*2.0+t2.*t9.*t18+t9.*t20.*t27+K_ygf.*t3.*t17.*t37+K_ygf.*t20.*t37.*t94+K_zgf.*t3.*t20.*t27+a.*a_n.*t2.*t6+a_n.*b.*t6.*t26.*2.0+K_zaf.*a.*a_n.*t26.*2.0-K_zaf.*a.*b.*t27.*2.0+K_zgf.*a.*b.*t94.*2.0+K_zaf.*a_n.*b.*t27.*2.0-K_zgf.*a_n.*b.*t94.*2.0+K_ygf.*a.*t3.*t18.*2.0+K_zgf.*a.*t30.*t37+K_ygf.*b.*t18.*t30+K_ygf.*a.*b.*t30.*t37+a.*g.*m_h.*t18.*t94+(b.*g.*m_b.*t18.*t30)./2.0+(b.*g.*m_h.*t18.*t30)./2.0-f.*g.*m_h.*t2.*t18.*3.0+f.*g.*m_h.*t18.*t27.*3.0+g.*m_b.*t20.*t37.*t94+g.*m_h.*t17.*t37.*t94+g.*m_h.*t20.*t37.*t94-g.*m_b.*t20.*t28.*t_f+g.*m_b.*t34.*t37.*t_f-g.*m_h.*t17.*t28.*t_f-g.*m_h.*t20.*t28.*t_f;
et25 = g.*m_h.*t34.*t37.*t_f+K_ycf.*a.*g.*m_h.*t22.*t29+K_ycf.*b.*g.*m_b.*t22.*t28+K_ycf.*b.*g.*m_h.*t22.*t28-(a.*a_n.*b.*g.*m_b.*t30)./2.0-(a.*a_n.*b.*g.*m_h.*t30)./2.0+K_ycf.*e.*g.*m_h.*t22.*t29+a.*a_n.*f.*g.*m_h.*t2.*3.0-a.*a_n.*f.*g.*m_h.*t27.*3.0+a_n.*b.*f.*g.*m_h.*t26.*3.0-a_n.*b.*f.*g.*m_h.*t28.*3.0+a.*a_n.*g.*m_h.*t28.*t_f-a.*b.*g.*m_b.*t27.*t_f-a.*b.*g.*m_h.*t29.*t_f+a_n.*b.*g.*m_b.*t27.*t_f+a_n.*b.*g.*m_h.*t27.*t_f+a.*e.*g.*m_h.*t37.*t94-a.*e.*g.*m_h.*t28.*t_f+a_n.*e.*g.*m_h.*t28.*t_f+a.*f.*g.*m_h.*t27.*t36.*3.0-b.*e.*g.*m_h.*t29.*t_f-a_n.*f.*g.*m_h.*t27.*t36.*3.0+b.*f.*g.*m_h.*t28.*t36.*3.0+a.*g.*m_h.*t26.*t37.*t_f+e.*g.*m_h.*t26.*t37.*t_f;
et26 = K_yvf.*V.*a.*g.*m_h.*t22.*t29+K_yvf.*V.*b.*g.*m_b.*t22.*t28+K_yvf.*V.*b.*g.*m_h.*t22.*t28+K_yvf.*V.*e.*g.*m_h.*t22.*t29+K_ycf.*a.*b.*g.*m_b.*rho_f.*t94+K_ycf.*a.*b.*g.*m_h.*rho_f.*t94+K_ycf.*a.*a_n.*g.*m_h.*t27.*t36+K_ycf.*a_n.*b.*g.*m_b.*t94.*t_f+K_ycf.*a_n.*b.*g.*m_h.*t94.*t_f+K_ycf.*a.*f.*g.*m_h.*rho_f.*t26-K_ycf.*a.*f.*g.*m_h.*rho_f.*t28+K_ycf.*a_n.*f.*g.*m_h.*rho_r.*t28+K_ycf.*a_n.*e.*g.*m_h.*t27.*t36+K_ycf.*b.*f.*g.*m_h.*rho_f.*t27-K_ycf.*b.*f.*g.*m_h.*rho_f.*t29+K_ycf.*a_n.*f.*g.*m_h.*t26.*t_f+K_ycf.*a_n.*f.*g.*m_h.*t28.*t_r+K_ycf.*a.*g.*m_h.*rho_f.*t29.*t_f-K_ycf.*a.*g.*m_h.*rho_r.*t29.*t_f+K_ycf.*b.*g.*m_b.*rho_r.*t37.*t94+K_ycf.*b.*g.*m_h.*rho_r.*t37.*t94+K_ycf.*b.*g.*m_b.*rho_f.*t28.*t_f-K_ycf.*b.*g.*m_b.*rho_r.*t28.*t_f+K_ycf.*b.*g.*m_h.*rho_f.*t28.*t_f;
et27 = -K_ycf.*b.*g.*m_h.*rho_r.*t28.*t_f+K_ycf.*a.*g.*m_h.*t27.*t_f.*t_r-K_ycf.*a.*g.*m_h.*t29.*t_f.*t_r+K_ycf.*b.*g.*m_b.*t37.*t94.*t_r+K_ycf.*b.*g.*m_h.*t37.*t94.*t_r+K_ycf.*b.*g.*m_b.*t26.*t_f.*t_r-K_ycf.*b.*g.*m_b.*t28.*t_f.*t_r+K_ycf.*b.*g.*m_h.*t26.*t_f.*t_r-K_ycf.*b.*g.*m_h.*t28.*t_f.*t_r+K_ycf.*e.*g.*m_h.*rho_f.*t29.*t_f-K_ycf.*e.*g.*m_h.*rho_r.*t29.*t_f+K_ycf.*f.*g.*m_h.*rho_f.*t28.*t36+K_ycf.*f.*g.*m_h.*rho_r.*t26.*t37-K_ycf.*f.*g.*m_h.*rho_r.*t28.*t36+K_ycf.*e.*g.*m_h.*t27.*t_f.*t_r-K_ycf.*e.*g.*m_h.*t29.*t_f.*t_r+K_ycf.*f.*g.*m_h.*t28.*t37.*t_f+K_ycf.*f.*g.*m_h.*t26.*t37.*t_r-K_ycf.*f.*g.*m_h.*t28.*t36.*t_r+K_ycf.*f.*g.*m_h.*t94.*t_f.*t_r+a.*b.*g.*m_h.*t3.*t27.*t37+b.*e.*g.*m_h.*t3.*t27.*t37;
et28 = K_yvf.*V.*a.*b.*g.*m_b.*rho_f.*t94+K_yvf.*V.*a.*b.*g.*m_h.*rho_f.*t94+K_yvf.*V.*a.*a_n.*g.*m_h.*t27.*t36+K_yvf.*V.*a_n.*b.*g.*m_b.*t94.*t_f+K_yvf.*V.*a_n.*b.*g.*m_h.*t94.*t_f-K_yvf.*V.*a.*f.*g.*m_h.*rho_f.*t28+K_yvf.*V.*a_n.*f.*g.*m_h.*rho_r.*t28+K_yvf.*V.*a_n.*e.*g.*m_h.*t27.*t36+K_yvf.*V.*b.*f.*g.*m_h.*rho_f.*t27-K_yvf.*V.*b.*f.*g.*m_h.*rho_f.*t29+K_yvf.*V.*a_n.*f.*g.*m_h.*t26.*t_f+K_yvf.*V.*a_n.*f.*g.*m_h.*t28.*t_r+K_yvf.*V.*a.*g.*m_h.*rho_f.*t29.*t_f-K_yvf.*V.*a.*g.*m_h.*rho_r.*t29.*t_f+K_yvf.*V.*b.*g.*m_b.*rho_r.*t37.*t94+K_yvf.*V.*b.*g.*m_h.*rho_r.*t37.*t94+K_yvf.*V.*b.*g.*m_b.*rho_f.*t28.*t_f-K_yvf.*V.*b.*g.*m_b.*rho_r.*t28.*t_f+K_yvf.*V.*b.*g.*m_h.*rho_f.*t28.*t_f-K_yvf.*V.*b.*g.*m_h.*rho_r.*t28.*t_f+K_yvf.*V.*a.*g.*m_h.*t27.*t_f.*t_r;
et29 = -K_yvf.*V.*a.*g.*m_h.*t29.*t_f.*t_r+K_yvf.*V.*b.*g.*m_b.*t37.*t94.*t_r+K_yvf.*V.*b.*g.*m_h.*t37.*t94.*t_r+K_yvf.*V.*b.*g.*m_b.*t26.*t_f.*t_r-K_yvf.*V.*b.*g.*m_b.*t28.*t_f.*t_r+K_yvf.*V.*b.*g.*m_h.*t26.*t_f.*t_r-K_yvf.*V.*b.*g.*m_h.*t28.*t_f.*t_r+K_yvf.*V.*e.*g.*m_h.*rho_f.*t29.*t_f-K_yvf.*V.*e.*g.*m_h.*rho_r.*t29.*t_f+K_yvf.*V.*f.*g.*m_h.*rho_f.*t28.*t36+K_yvf.*V.*f.*g.*m_h.*rho_r.*t26.*t37-K_yvf.*V.*f.*g.*m_h.*rho_r.*t28.*t36+K_yvf.*V.*e.*g.*m_h.*t27.*t_f.*t_r-K_yvf.*V.*e.*g.*m_h.*t29.*t_f.*t_r+K_yvf.*V.*f.*g.*m_h.*t28.*t37.*t_f+K_yvf.*V.*f.*g.*m_h.*t26.*t37.*t_r-K_yvf.*V.*f.*g.*m_h.*t28.*t36.*t_r+K_yvf.*V.*f.*g.*m_h.*t94.*t_f.*t_r+K_ycf.*a.*g.*m_h.*rho_r.*t3.*t27.*t37+K_ycf.*a.*g.*m_h.*t3.*t27.*t37.*t_r;
et30 = K_ycf.*e.*g.*m_h.*rho_r.*t3.*t27.*t37+K_ycf.*e.*g.*m_h.*t3.*t27.*t37.*t_r+K_yvf.*V.*a.*g.*m_h.*rho_r.*t3.*t27.*t37+K_yvf.*V.*a.*g.*m_h.*t3.*t27.*t37.*t_r+K_yvf.*V.*e.*g.*m_h.*rho_r.*t3.*t27.*t37+K_yvf.*V.*e.*g.*m_h.*t3.*t27.*t37.*t_r;
mt1 = [0.0,0.0,V,0.0,0.0,0.0,0.0,0.0,0.0,0.0,V,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-t41.*t166.*(et1+et2+et3),0.0,-K_ygf-K_ygr,-t167.*(et6+et7+et8+et9+et10),0.0,-t167.*(et19+et20+et21+et22),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t41.*t166.*(et4+et5),0.0];
mt2 = [t166.*(-t2.*t6+t2.*t9+K_ygf.*a.*t3-K_yaf.*b.*t26+K_ygf.*t3.*t34+K_ygf.*t3.*t37+K_ycf.*a.*g.*m_h.*t27+K_ycf.*b.*g.*m_b.*t26+K_ycf.*b.*g.*m_h.*t26+K_ycf.*e.*g.*m_h.*t27+K_yvf.*V.*a.*g.*m_h.*t27+K_yvf.*V.*b.*g.*m_b.*t26+K_yvf.*V.*b.*g.*m_h.*t26+K_yvf.*V.*e.*g.*m_h.*t27+K_ycf.*f.*g.*m_h.*t3.*t26+K_yvf.*V.*f.*g.*m_h.*t3.*t26),t167.*(et11+et12+et13+et14+et15+et16),0.0,t167.*(et23+et24+et25+et26+et27+et28+et29+et30),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,t41.*t80,0.0];
mt3 = [(t35.*(t42+t62+a.*t6+a.*t33-a_n.*t6.*2.0+t6.*t34.*2.0-t9.*t34.*2.0+K_zaf.*b.*t26+K_yaf.*t20.*t26+a.*m_h.*t16.*t27+b.*m_b.*t16.*t26+b.*m_h.*t16.*t26+e.*m_h.*t16.*t27-f.*m_h.*t3.*t16.*t26))./t26,0.0,t35.*t41.*(t6+t39+t46+m_b.*t2.*t16+m_h.*t2.*t16),-t35.*t41.*t55.*t56.*(et17+et18),0.0,t35.*t41.*t55.*(rho_f.*t42+rho_f.*t62+t42.*t_f+t62.*t_f+a.*rho_f.*t33+a.*t33.*t_f+(i_fy.*t16.*t30)./2.0+rho_f.*t6.*t37+rho_f.*t34.*t39+t6.*t37.*t_f+t34.*t39.*t_f+K_zaf.*b.*rho_f.*t26+K_zaf.*b.*t26.*t_f+e.*m_h.*rho_f.*t2.*t16+e.*m_h.*t2.*t16.*t_f),0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0];
mt4 = [t35.*t166.*(t4+t5-t8+t38+t44+t45+t51+t52+t63+t64+t81-t82+t83-t84+t168-t169+K_yvr.*V.*g.*m_b.*t37+K_yvr.*V.*g.*m_h.*t37),0.0,0.0,t15.*t35.*t166.*(-t5+t8-t45-t51-t52+t53+t54+t82+t84+t169),0.0,-t14.*t35.*t166.*(t4+t38+t44+t63+t64+t81+t83+t168),0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,t35.*t41.*(t6+t33+t39+t46+K_zar.*t2),0.0,t25.*t35,t15.*t25.*t35,0.0,-t35.*(t9-t33),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,-t14+t15,1.0];
mt5 = [K_zaf.*rho_f.*t35+K_zaf.*rho_r.*t35+K_zar.*rho_r.*t35+V.*i_fy.*t55+V.*i_ry.*t56+K_zaf.*t35.*t_r+b.*t10.*t35+b.*t13.*t35-K_zaf.*rho_f.*t26.*t35-K_zaf.*rho_f.*t31.*t35-K_zaf.*t26.*t35.*t_f-K_zaf.*t31.*t35.*t_f+b.*t26.*t35.*t40+b.*t31.*t35.*t40+rho_r.*t6.*t35.*t41+rho_r.*t35.*t39.*t41-t6.*t35.*t41.*t_f+t9.*t35.*t41.*t_f+t6.*t35.*t41.*t_r+t35.*t39.*t41.*t_r+K_yaf.*b.*rho_f.*t35-K_yaf.*b.*rho_f.*t26.*t35-K_yaf.*b.*rho_f.*t31.*t35,0.0,t35.*t190,t15.*t35.*t190,0.0,t35.*t55.*(t57+t9.*t22-t22.*t33+rho_f.*rho_r.*t33+rho_f.*rho_r.*t39+rho_f.*t9.*t_f-rho_f.*t33.*t_f+rho_r.*t33.*t_f+rho_r.*t39.*t_f+rho_f.*t33.*t_r+rho_f.*t39.*t_r+t33.*t_f.*t_r+t39.*t_f.*t_r),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0];
mt6 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,-K_xkr.*t15.*t35,0.0,0.0,K_xkr.*t35.*1.0./t56.^2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,t2,0.0,t36+t37+rho_f.*t3,-t3,K_zaf.*t35.*t36+K_zaf.*t35.*t37+b.*t35.*t39+t35.*t41.*t42-V.*i_fy.*t3.*t55+b.*t3.*t12.*t35+t6.*t35.*t36.*t41+t6.*t35.*t37.*t41+t35.*t36.*t39.*t41,0.0,-K_yaf.*t35.*(a_n-t36),-t35.*t55.*(t47+t48+t49+t50+t57-rho_f.*t10.*t36-t3.*t10.*t22+t36.*t40.*t_r+rho_f.*t3.*t40.*t_r),0.0,t35.*(t42+t62+t33.*t36+t36.*t39+C_delta.*V),0.0,0.0,0.0,0.0,-V.*t2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,-K_xkf.*t14.*t35,0.0,0.0,0.0,0.0,K_xkf.*t35.*1.0./t55.^2,0.0,0.0,0.0,0.0,0.0];
H = reshape([mt1,mt2,mt3,mt4,mt5,mt6],24,14);
end
