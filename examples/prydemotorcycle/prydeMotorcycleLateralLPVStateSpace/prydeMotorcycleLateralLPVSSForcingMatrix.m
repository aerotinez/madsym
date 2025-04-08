function H = prydeMotorcycleLateralLPVSSForcingMatrix(in1)
%prydeMotorcycleLateralLPVSSForcingMatrix
%    H = prydeMotorcycleLateralLPVSSForcingMatrix(IN1)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    07-Apr-2025 02:26:37

%   states = [varphi delta omega_bz v_ry omega_bx omega_delta]
%   inputs = [tau_delta]
%   params = [A C_d C_delta C_l C_p I_bxx I_bxz I_bzz I_hxx I_hzz K_xkf K_xkr K_yaf K_ycf K_ygf K_yar K_ycr K_ygr K_yvf K_yvr K_zaf K_zgf K_zar K_zgr V a a_n b e f g h i_fy i_ry m_b m_f m_h m_r rho rho_f rho_r t_f t_r varepsilon]
%
A = in1(1,:);
C_d = in1(2,:);
C_delta = in1(3,:);
C_l = in1(4,:);
C_p = in1(5,:);
K_xkf = in1(11,:);
K_xkr = in1(12,:);
K_yaf = in1(13,:);
K_ycf = in1(14,:);
K_ygf = in1(15,:);
K_yar = in1(16,:);
K_ycr = in1(17,:);
K_ygr = in1(18,:);
K_yvf = in1(19,:);
K_yvr = in1(20,:);
K_zaf = in1(21,:);
K_zgf = in1(22,:);
K_zar = in1(23,:);
K_zgr = in1(24,:);
V = in1(25,:);
a = in1(26,:);
a_n = in1(27,:);
b = in1(28,:);
e = in1(29,:);
f = in1(30,:);
g = in1(31,:);
h = in1(32,:);
i_fy = in1(33,:);
i_ry = in1(34,:);
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
t3 = sin(varepsilon);
t4 = K_yaf.*a;
t5 = K_yaf.*a_n;
t6 = K_yaf.*rho_r;
t7 = K_yar.*rho_r;
t8 = K_yaf.*t_f;
t9 = K_yaf.*t_r;
t10 = rho_f+t_f;
t11 = rho_r+t_r;
t12 = V.^2;
t13 = V.^3;
t15 = V.^5;
t16 = a.*4.0;
t17 = a.^2;
t18 = a_n.*4.0;
t19 = a_n.^2;
t21 = b.^2;
t22 = rho_f.^2;
t23 = rho_r.^2;
t24 = t_f.^2;
t25 = t_r.^2;
t26 = varepsilon.*2.0;
t27 = K_yaf+K_yar;
t35 = 1.0./K_xkf;
t36 = 1.0./K_xkr;
t37 = 1.0./V;
t38 = -a_n;
t53 = a_n.*g.*m_f.*rho_f.*-4.0;
t56 = a_n.*e.*g.*m_h.*-4.0;
t14 = t12.^2;
t28 = t2.^2;
t29 = t2.^3;
t31 = sin(t26);
t32 = t3.^3;
t33 = K_zaf.*t2;
t34 = b.*t2;
t39 = -t18;
t40 = -t8;
t41 = 1.0./t2;
t42 = a_n.*t5;
t46 = g.*m_f.*rho_f.*t16;
t47 = g.*m_f.*rho_f.*t18;
t49 = 1.0./t10;
t50 = 1.0./t11;
t51 = e.*g.*m_h.*t16;
t52 = e.*g.*m_h.*t18;
t59 = K_xkf.*rho_f.*rho_r.*t4.*t18;
t60 = K_xkf.*rho_r.*t4.*t18.*t_f;
t61 = K_xkf.*rho_f.*t4.*t18.*t_r;
t62 = K_xkf.*t4.*t18.*t_f.*t_r;
t67 = a_n.*g.*m_h.*t3.*t16;
t72 = f.*g.*m_h.*t2.*t_f.*4.0;
t75 = K_xkf.*a_n.*rho_f.*t4.*t16;
t76 = K_xkf.*a_n.*t4.*t16.*t_f;
t77 = K_xkf.*rho_f.*rho_r.*t5.*t18;
t78 = K_xkf.*rho_r.*t5.*t18.*t_f;
t79 = K_xkf.*rho_f.*t5.*t18.*t_r;
t80 = K_xkf.*t5.*t18.*t_f.*t_r;
t82 = a_n.*f.*g.*m_h.*t2.*-4.0;
t83 = A.*C_l.*a.*rho.*t12.*t_f;
t84 = A.*C_l.*a_n.*rho.*t12.*t_f;
t87 = t3.*t53;
t89 = K_xkf.*i_fy.*t2.*t12.*t16;
t108 = K_xkf.*rho_f.*t4.*t19.*8.0;
t109 = K_xkf.*t4.*t19.*t_f.*8.0;
t110 = A.*C_l.*rho.*t12.*t19;
t111 = A.*C_p.*a.*a_n.*rho.*t12.*2.0;
t112 = A.*C_p.*a.*rho.*t12.*t_f.*2.0;
t113 = A.*C_p.*a_n.*rho.*t12.*t_f.*2.0;
t115 = K_xkf.*a_n.*i_fy.*t2.*t12.*-4.0;
t122 = K_xkf.*K_xkr.*a_n.*rho_f.*rho_r.*t4.*t_r.*1.6e+1;
t123 = K_xkf.*K_xkr.*a_n.*rho_r.*t4.*t_f.*t_r.*1.6e+1;
t130 = A.*C_p.*rho.*t12.*t19.*2.0;
t131 = A.*C_l.*a.*rho.*t12.*t38;
t133 = A.*C_l.*rho.*t12.*t38.*t_f;
t135 = K_xkf.*K_xkr.*rho_f.*t4.*t16.*t23;
t136 = K_xkf.*K_xkr.*rho_f.*t5.*t18.*t23;
t137 = K_xkf.*K_xkr.*a_n.*rho_f.*t4.*t23.*8.0;
t138 = K_xkf.*K_xkr.*t4.*t16.*t23.*t_f;
t139 = K_xkf.*K_xkr.*rho_f.*t4.*t16.*t25;
t140 = K_xkf.*K_xkr.*t5.*t18.*t23.*t_f;
t141 = K_xkf.*K_xkr.*rho_f.*t5.*t18.*t25;
t142 = K_xkf.*K_xkr.*a_n.*t4.*t23.*t_f.*8.0;
t143 = K_xkf.*K_xkr.*a_n.*rho_f.*t4.*t25.*8.0;
t144 = K_xkf.*K_xkr.*t4.*t16.*t25.*t_f;
t145 = K_xkf.*K_xkr.*t5.*t18.*t25.*t_f;
t146 = K_xkf.*K_xkr.*a_n.*t4.*t25.*t_f.*8.0;
t147 = K_xkf.*K_xkr.*a.*rho_f.*rho_r.*t4.*t_r.*8.0;
t149 = K_xkf.*K_xkr.*a.*rho_r.*t4.*t_f.*t_r.*8.0;
t159 = K_xkf.*K_xkr.*i_ry.*rho_f.*t2.*t12.*t16;
t163 = K_xkf.*K_xkr.*i_ry.*t2.*t12.*t16.*t_f;
t168 = K_ycf.*g.*i_fy.*m_f.*t2.*t12.*t16;
t169 = K_yvf.*g.*i_fy.*m_f.*t2.*t13.*t16;
t173 = A.*C_l.*K_yvf.*a.*i_fy.*rho.*t2.*t15;
t175 = A.*C_l.*K_yvf.*a_n.*i_fy.*rho.*t2.*t15;
t187 = K_xkf.*K_xkr.*a_n.*i_ry.*rho_f.*t2.*t12.*-4.0;
t189 = K_xkf.*K_xkr.*a_n.*i_ry.*t2.*t12.*t_f.*-4.0;
t193 = A.*C_p.*a_n.*rho.*t3.*t12.*t_f.*-2.0;
t194 = K_ycf.*a_n.*g.*i_fy.*m_f.*t2.*t12.*-4.0;
t195 = K_yvf.*a_n.*g.*i_fy.*m_f.*t2.*t13.*-4.0;
t197 = A.*C_p.*K_yvf.*a.*i_fy.*rho.*t2.*t15.*2.0;
t199 = A.*C_p.*K_yvf.*a_n.*i_fy.*rho.*t2.*t15.*2.0;
t242 = K_xkf.*K_ycr.*g.*i_ry.*m_b.*rho_f.*t2.*t12.*t16;
t243 = K_xkf.*K_yvr.*g.*i_ry.*m_b.*rho_f.*t2.*t13.*t16;
t244 = K_xkf.*K_ycr.*g.*i_ry.*m_h.*rho_f.*t2.*t12.*t16;
t245 = K_xkf.*K_yvr.*g.*i_ry.*m_h.*rho_f.*t2.*t13.*t16;
t246 = K_xkf.*K_ycr.*g.*i_ry.*m_r.*rho_f.*t2.*t12.*t16;
t247 = K_xkf.*K_yvr.*g.*i_ry.*m_r.*rho_f.*t2.*t13.*t16;
t256 = K_xkf.*K_ycr.*g.*i_ry.*m_b.*t2.*t12.*t16.*t_f;
t257 = K_xkf.*K_yvr.*g.*i_ry.*m_b.*t2.*t13.*t16.*t_f;
t260 = K_xkf.*K_ycr.*g.*i_ry.*m_h.*t2.*t12.*t16.*t_f;
t261 = K_xkf.*K_yvr.*g.*i_ry.*m_h.*t2.*t13.*t16.*t_f;
t262 = K_xkf.*K_ycr.*g.*i_ry.*m_r.*t2.*t12.*t16.*t_f;
t263 = K_xkf.*K_yvr.*g.*i_ry.*m_r.*t2.*t13.*t16.*t_f;
t275 = A.*C_l.*K_xkf.*K_yvr.*a.*i_ry.*rho.*rho_f.*t2.*t15;
t279 = A.*C_l.*K_xkf.*K_yvr.*a_n.*i_ry.*rho.*rho_f.*t2.*t15;
t282 = A.*C_l.*K_xkf.*K_yvr.*a.*i_ry.*rho.*t2.*t15.*t_f;
t286 = A.*C_l.*K_xkf.*K_yvr.*a_n.*i_ry.*rho.*t2.*t15.*t_f;
t306 = K_xkf.*K_ycr.*a_n.*g.*i_ry.*m_b.*rho_f.*t2.*t12.*-4.0;
t307 = K_xkf.*K_yvr.*a_n.*g.*i_ry.*m_b.*rho_f.*t2.*t13.*-4.0;
t308 = K_xkf.*K_ycr.*a_n.*g.*i_ry.*m_h.*rho_f.*t2.*t12.*-4.0;
t309 = K_xkf.*K_yvr.*a_n.*g.*i_ry.*m_h.*rho_f.*t2.*t13.*-4.0;
t310 = K_xkf.*K_ycr.*a_n.*g.*i_ry.*m_r.*rho_f.*t2.*t12.*-4.0;
t311 = K_xkf.*K_yvr.*a_n.*g.*i_ry.*m_r.*rho_f.*t2.*t13.*-4.0;
t312 = K_xkf.*K_ycr.*a_n.*g.*i_ry.*m_b.*t2.*t12.*t_f.*-4.0;
t313 = K_xkf.*K_yvr.*a_n.*g.*i_ry.*m_b.*t2.*t13.*t_f.*-4.0;
t316 = K_xkf.*K_ycr.*a_n.*g.*i_ry.*m_h.*t2.*t12.*t_f.*-4.0;
t317 = K_xkf.*K_yvr.*a_n.*g.*i_ry.*m_h.*t2.*t13.*t_f.*-4.0;
t318 = K_xkf.*K_ycr.*a_n.*g.*i_ry.*m_r.*t2.*t12.*t_f.*-4.0;
t319 = K_xkf.*K_yvr.*a_n.*g.*i_ry.*m_r.*t2.*t13.*t_f.*-4.0;
t323 = A.*C_p.*K_xkf.*K_yvr.*a.*i_ry.*rho.*rho_f.*t2.*t15.*2.0;
t327 = A.*C_p.*K_xkf.*K_yvr.*a_n.*i_ry.*rho.*rho_f.*t2.*t15.*2.0;
t330 = A.*C_p.*K_xkf.*K_yvr.*a.*i_ry.*rho.*t2.*t15.*t_f.*2.0;
t334 = A.*C_p.*K_xkf.*K_yvr.*a_n.*i_ry.*rho.*t2.*t15.*t_f.*2.0;
t365 = A.*C_p.*K_xkr.*K_yvf.*a.*i_fy.*rho.*rho_r.*t2.*t15.*-2.0;
t375 = A.*C_p.*K_xkr.*K_yvf.*a.*i_fy.*rho.*t2.*t15.*t_r.*-2.0;
t30 = t28.^2;
t43 = t34.*4.0;
t44 = a_n.*t33;
t45 = K_yaf.*t34;
t48 = -t33;
t54 = -t32;
t55 = t33.*t38;
t57 = K_xkf.*rho_f.*t18.*t42;
t58 = K_xkf.*t18.*t42.*t_f;
t63 = g.*m_b.*t18.*t34;
t64 = g.*m_h.*t18.*t34;
t69 = t3.*t51;
t73 = t3.*t46;
t85 = g.*m_b.*t34.*t_f.*-4.0;
t86 = g.*m_h.*t34.*t_f.*-4.0;
t92 = a+t34+t38;
t93 = a_n.*g.*m_h.*t16.*t28;
t94 = t28.*t52;
t96 = b.*e.*g.*m_h.*t31.*2.0;
t97 = a_n.*f.*g.*m_h.*t31.*2.0;
t98 = e.*g.*m_h.*t28.*t_f.*4.0;
t99 = f.*g.*m_h.*t29.*t_f.*4.0;
t100 = b.*g.*m_f.*rho_f.*t31.*2.0;
t101 = b.*g.*m_b.*t31.*t_f.*2.0;
t102 = b.*g.*m_h.*t31.*t_f.*2.0;
t103 = f.*g.*m_h.*t31.*t_f.*2.0;
t116 = a.*g.*m_h.*t28.*t_f.*-4.0;
t124 = K_xkf.*a_n.*rho_f.*t4.*t34.*8.0;
t125 = K_xkf.*a_n.*t4.*t34.*t_f.*8.0;
t127 = A.*C_l.*rho.*t12.*t34.*t_f;
t128 = t3.*t83;
t132 = -t111;
t134 = -t113;
t148 = K_xkf.*K_xkr.*rho_f.*rho_r.*t42.*t_r.*8.0;
t150 = K_xkf.*K_xkr.*rho_r.*t42.*t_f.*t_r.*8.0;
t151 = K_xkf.*b.*i_fy.*t12.*t28.*4.0;
t152 = K_xkf.*rho_f.*t34.*t42.*8.0;
t153 = K_xkf.*t34.*t42.*t_f.*8.0;
t154 = K_xkf.*a.*i_fy.*t12.*t31.*2.0;
t155 = K_xkf.*a_n.*i_fy.*t12.*t31.*2.0;
t156 = A.*C_p.*a_n.*rho.*t12.*t34.*2.0;
t157 = A.*C_p.*rho.*t12.*t34.*t_f.*2.0;
t158 = K_xkr.*rho_r.*t89;
t162 = K_xkr.*t89.*t_r;
t166 = t3.*t112;
t172 = A.*C_l.*K_ycf.*a.*i_fy.*rho.*t2.*t14;
t174 = A.*C_l.*K_ycf.*a_n.*i_fy.*rho.*t2.*t14;
t176 = A.*C_p.*b.*rho.*t12.*t31.*t_f;
t177 = K_xkf.*K_xkr.*rho_f.*rho_r.*t4.*t34.*t_r.*1.6e+1;
t178 = K_xkf.*K_xkr.*rho_f.*rho_r.*t5.*t34.*t_r.*1.6e+1;
t179 = K_xkf.*K_xkr.*rho_r.*t4.*t34.*t_f.*t_r.*1.6e+1;
t180 = K_xkf.*K_xkr.*rho_r.*t5.*t34.*t_f.*t_r.*1.6e+1;
t184 = A.*C_l.*rho.*t12.*t34.*t38;
t186 = K_xkr.*rho_r.*t115;
t188 = K_xkr.*t115.*t_r;
t190 = K_xkf.*rho_f.*t5.*t21.*t28.*4.0;
t191 = K_xkf.*t5.*t21.*t28.*t_f.*4.0;
t192 = t3.*t133;
t196 = A.*C_p.*K_ycf.*a.*i_fy.*rho.*t2.*t14.*2.0;
t198 = A.*C_p.*K_ycf.*a_n.*i_fy.*rho.*t2.*t14.*2.0;
t200 = K_xkf.*K_xkr.*rho_f.*t4.*t23.*t34.*8.0;
t202 = K_xkf.*K_xkr.*b.*i_ry.*rho_f.*t12.*t28.*4.0;
t203 = K_xkf.*K_xkr.*rho_f.*t5.*t23.*t34.*8.0;
t204 = K_xkf.*K_xkr.*t4.*t23.*t34.*t_f.*8.0;
t205 = K_xkf.*K_xkr.*rho_f.*t4.*t25.*t34.*8.0;
t207 = K_xkf.*K_xkr.*b.*i_ry.*t12.*t28.*t_f.*4.0;
t208 = K_xkf.*K_xkr.*t5.*t23.*t34.*t_f.*8.0;
t209 = K_xkf.*K_xkr.*rho_f.*t5.*t25.*t34.*8.0;
t210 = K_xkf.*K_xkr.*t4.*t25.*t34.*t_f.*8.0;
t211 = K_xkf.*K_xkr.*t5.*t25.*t34.*t_f.*8.0;
t212 = K_ycf.*g.*i_fy.*m_h.*t12.*t16.*t29;
t213 = K_yvf.*g.*i_fy.*m_h.*t13.*t16.*t29;
t214 = K_ycf.*b.*g.*i_fy.*m_b.*t12.*t28.*4.0;
t215 = K_yvf.*b.*g.*i_fy.*m_b.*t13.*t28.*4.0;
t216 = K_ycf.*b.*g.*i_fy.*m_f.*t12.*t28.*4.0;
t217 = K_yvf.*b.*g.*i_fy.*m_f.*t13.*t28.*4.0;
t218 = K_ycf.*b.*g.*i_fy.*m_h.*t12.*t28.*4.0;
t219 = K_yvf.*b.*g.*i_fy.*m_h.*t13.*t28.*4.0;
t220 = K_ycf.*e.*g.*i_fy.*m_h.*t12.*t29.*4.0;
t221 = K_yvf.*e.*g.*i_fy.*m_h.*t13.*t29.*4.0;
t222 = K_ycf.*f.*g.*i_fy.*m_h.*t12.*t28.*4.0;
t224 = K_yvf.*f.*g.*i_fy.*m_h.*t13.*t28.*4.0;
t226 = K_ycf.*a.*g.*i_fy.*m_f.*t12.*t31.*2.0;
t227 = K_yvf.*a.*g.*i_fy.*m_f.*t13.*t31.*2.0;
t228 = K_ycf.*a_n.*g.*i_fy.*m_f.*t12.*t31.*2.0;
t229 = K_yvf.*a_n.*g.*i_fy.*m_f.*t13.*t31.*2.0;
t230 = A.*C_l.*K_ycf.*b.*i_fy.*rho.*t14.*t28;
t231 = A.*C_l.*K_yvf.*b.*i_fy.*rho.*t15.*t28;
t232 = A.*C_p.*K_ycf.*a.*i_fy.*rho.*t14.*t31;
t233 = A.*C_p.*K_yvf.*a.*i_fy.*rho.*t15.*t31;
t234 = A.*C_p.*K_ycf.*a_n.*i_fy.*rho.*t14.*t31;
t235 = A.*C_p.*K_yvf.*a_n.*i_fy.*rho.*t15.*t31;
t236 = A.*C_d.*K_xkf.*a.*i_ry.*rho.*rho_f.*t2.*t14.*2.0;
t237 = A.*C_d.*K_xkf.*a_n.*i_ry.*rho.*rho_f.*t2.*t14.*2.0;
t238 = A.*C_d.*K_xkf.*a.*i_ry.*rho.*t2.*t14.*t_f.*2.0;
t239 = A.*C_d.*K_xkf.*a_n.*i_ry.*rho.*t2.*t14.*t_f.*2.0;
t240 = K_xkr.*rho_r.*t168;
t241 = K_xkr.*rho_r.*t169;
t258 = K_xkr.*t168.*t_r;
t259 = K_xkr.*t169.*t_r;
t273 = K_xkr.*rho_r.*t173;
t274 = A.*C_l.*K_xkf.*K_ycr.*a.*i_ry.*rho.*rho_f.*t2.*t14;
t277 = K_xkr.*rho_r.*t175;
t278 = A.*C_l.*K_xkf.*K_ycr.*a_n.*i_ry.*rho.*rho_f.*t2.*t14;
t280 = A.*C_l.*K_xkf.*K_ycr.*a.*i_ry.*rho.*t2.*t14.*t_f;
t283 = K_xkr.*t173.*t_r;
t284 = A.*C_l.*K_xkf.*K_ycr.*a_n.*i_ry.*rho.*t2.*t14.*t_f;
t287 = K_xkr.*t175.*t_r;
t289 = -t173;
t291 = -t197;
t296 = A.*C_p.*K_ycf.*b.*i_fy.*rho.*t14.*t28.*2.0;
t297 = A.*C_p.*K_yvf.*b.*i_fy.*rho.*t15.*t28.*2.0;
t298 = A.*C_d.*K_ycf.*i_fy.*rho.*rho_r.*t14.*t28.*2.0;
t299 = A.*C_d.*K_yvf.*i_fy.*rho.*rho_r.*t15.*t28.*2.0;
t301 = A.*C_d.*K_ycf.*i_fy.*rho.*t14.*t28.*t_r.*2.0;
t302 = A.*C_d.*K_yvf.*i_fy.*rho.*t15.*t28.*t_r.*2.0;
t304 = K_xkr.*rho_r.*t194;
t305 = K_xkr.*rho_r.*t195;
t314 = K_xkr.*t194.*t_r;
t315 = K_xkr.*t195.*t_r;
t322 = A.*C_p.*K_xkf.*K_ycr.*a.*i_ry.*rho.*rho_f.*t2.*t14.*2.0;
t325 = K_xkr.*rho_r.*t199;
t326 = A.*C_p.*K_xkf.*K_ycr.*a_n.*i_ry.*rho.*rho_f.*t2.*t14.*2.0;
t328 = A.*C_p.*K_xkf.*K_ycr.*a.*i_ry.*rho.*t2.*t14.*t_f.*2.0;
t332 = A.*C_p.*K_xkf.*K_ycr.*a_n.*i_ry.*rho.*t2.*t14.*t_f.*2.0;
t335 = K_xkr.*t199.*t_r;
t336 = t6+t7+t9+t40;
t339 = A.*C_l.*K_xkf.*K_ycr.*b.*i_ry.*rho.*rho_f.*t14.*t28;
t340 = A.*C_l.*K_xkf.*K_yvr.*b.*i_ry.*rho.*rho_f.*t15.*t28;
t341 = A.*C_l.*K_xkf.*K_ycr.*b.*i_ry.*rho.*t14.*t28.*t_f;
t343 = A.*C_l.*K_xkf.*K_yvr.*b.*i_ry.*rho.*t15.*t28.*t_f;
t349 = (A.*C_l.*b.*rho.*t12.*t31.*t_f)./2.0;
t356 = K_xkf.*K_xkr.*rho_f.*rho_r.*t6.*t21.*t28.*4.0;
t357 = K_xkf.*K_xkr.*rho_r.*t6.*t21.*t28.*t_f.*4.0;
t358 = K_xkf.*K_xkr.*rho_f.*t9.*t21.*t28.*t_r.*4.0;
t359 = K_xkf.*K_xkr.*t8.*t21.*t25.*t28.*4.0;
t360 = K_xkf.*K_xkr.*rho_f.*t6.*t21.*t28.*t_r.*8.0;
t361 = K_xkf.*K_xkr.*t6.*t21.*t28.*t_f.*t_r.*8.0;
t364 = A.*C_p.*K_xkr.*K_ycf.*a.*i_fy.*rho.*rho_r.*t2.*t14.*-2.0;
t367 = -t275;
t369 = -t327;
t372 = -t282;
t374 = A.*C_p.*K_xkr.*K_ycf.*a.*i_fy.*rho.*t2.*t14.*t_r.*-2.0;
t377 = -t334;
t378 = A.*C_d.*K_xkf.*b.*i_ry.*rho.*rho_f.*t14.*t28.*2.0;
t379 = A.*C_d.*K_xkf.*b.*i_ry.*rho.*t14.*t28.*t_f.*2.0;
t391 = K_xkf.*K_ycr.*b.*g.*i_ry.*m_r.*rho_f.*t12.*t28.*4.0;
t392 = K_xkf.*K_yvr.*b.*g.*i_ry.*m_r.*rho_f.*t13.*t28.*4.0;
t403 = K_xkf.*K_ycr.*b.*g.*i_ry.*m_r.*t12.*t28.*t_f.*4.0;
t404 = K_xkf.*K_yvr.*b.*g.*i_ry.*m_r.*t13.*t28.*t_f.*4.0;
t407 = K_xkf.*K_ycr.*e.*g.*i_ry.*m_h.*rho_f.*t12.*t29.*4.0;
t408 = K_xkf.*K_yvr.*e.*g.*i_ry.*m_h.*rho_f.*t13.*t29.*4.0;
t410 = K_xkf.*K_ycr.*e.*g.*i_ry.*m_h.*t12.*t29.*t_f.*4.0;
t412 = K_xkf.*K_yvr.*e.*g.*i_ry.*m_h.*t13.*t29.*t_f.*4.0;
t417 = A.*C_p.*K_xkf.*K_ycr.*b.*i_ry.*rho.*rho_f.*t14.*t28.*2.0;
t418 = A.*C_p.*K_xkf.*K_yvr.*b.*i_ry.*rho.*rho_f.*t15.*t28.*2.0;
t419 = A.*C_p.*K_xkf.*K_ycr.*b.*i_ry.*rho.*t14.*t28.*t_f.*2.0;
t421 = A.*C_p.*K_xkf.*K_yvr.*b.*i_ry.*rho.*t15.*t28.*t_f.*2.0;
t423 = A.*C_d.*K_xkf.*K_ycr.*i_ry.*rho.*rho_f.*rho_r.*t14.*t28.*2.0;
t424 = A.*C_d.*K_xkf.*K_yvr.*i_ry.*rho.*rho_f.*rho_r.*t15.*t28.*2.0;
t425 = A.*C_d.*K_xkf.*K_ycr.*i_ry.*rho.*rho_r.*t14.*t28.*t_f.*2.0;
t426 = A.*C_d.*K_xkr.*K_ycf.*i_fy.*rho.*rho_r.*t14.*t28.*t_r.*4.0;
t427 = A.*C_d.*K_xkf.*K_yvr.*i_ry.*rho.*rho_r.*t15.*t28.*t_f.*2.0;
t428 = A.*C_d.*K_xkr.*K_yvf.*i_fy.*rho.*rho_r.*t15.*t28.*t_r.*4.0;
t429 = A.*C_d.*K_xkf.*K_ycr.*i_ry.*rho.*rho_f.*t14.*t28.*t_r.*2.0;
t430 = A.*C_d.*K_xkf.*K_yvr.*i_ry.*rho.*rho_f.*t15.*t28.*t_r.*2.0;
t431 = A.*C_d.*K_xkf.*K_ycr.*i_ry.*rho.*t14.*t28.*t_f.*t_r.*2.0;
t432 = A.*C_d.*K_xkf.*K_yvr.*i_ry.*rho.*t15.*t28.*t_f.*t_r.*2.0;
t433 = (A.*C_l.*K_ycf.*a.*i_fy.*rho.*t14.*t31)./2.0;
t434 = (A.*C_l.*K_yvf.*a.*i_fy.*rho.*t15.*t31)./2.0;
t435 = (A.*C_l.*K_ycf.*a_n.*i_fy.*rho.*t14.*t31)./2.0;
t436 = (A.*C_l.*K_yvf.*a_n.*i_fy.*rho.*t15.*t31)./2.0;
t437 = K_xkf.*K_ycr.*a.*g.*i_ry.*m_h.*rho_f.*t12.*t29.*-4.0;
t438 = K_xkf.*K_yvr.*a.*g.*i_ry.*m_h.*rho_f.*t13.*t29.*-4.0;
t439 = K_xkf.*K_ycr.*a.*g.*i_ry.*m_h.*t12.*t29.*t_f.*-4.0;
t440 = K_xkf.*K_yvr.*a.*g.*i_ry.*m_h.*t13.*t29.*t_f.*-4.0;
t447 = A.*C_d.*K_xkr.*K_ycf.*i_fy.*rho.*t14.*t23.*t28.*2.0;
t448 = A.*C_d.*K_xkr.*K_yvf.*i_fy.*rho.*t15.*t23.*t28.*2.0;
t449 = A.*C_d.*K_xkr.*K_ycf.*i_fy.*rho.*t14.*t25.*t28.*2.0;
t450 = A.*C_d.*K_xkr.*K_yvf.*i_fy.*rho.*t15.*t25.*t28.*2.0;
t453 = A.*C_p.*K_xkr.*K_ycf.*b.*i_fy.*rho.*rho_r.*t14.*t28.*-2.0;
t454 = A.*C_p.*K_xkr.*K_yvf.*b.*i_fy.*rho.*rho_r.*t15.*t28.*-2.0;
t461 = A.*C_p.*K_xkr.*K_ycf.*b.*i_fy.*rho.*t14.*t28.*t_r.*-2.0;
t462 = A.*C_p.*K_xkr.*K_yvf.*b.*i_fy.*rho.*t15.*t28.*t_r.*-2.0;
t65 = e.*g.*m_h.*t43;
t68 = g.*m_f.*rho_f.*t43;
t88 = K_xkf.*rho_f.*t18.*t44;
t91 = K_xkf.*t18.*t44.*t_f;
t104 = K_xkf.*rho_f.*rho_r.*t5.*t43;
t105 = K_xkf.*rho_r.*t5.*t43.*t_f;
t106 = K_xkf.*rho_f.*t5.*t43.*t_r;
t107 = K_xkf.*t5.*t43.*t_f.*t_r;
t114 = t3+t54;
t117 = -t97;
t118 = -t98;
t119 = -t99;
t120 = -t101;
t121 = -t102;
t181 = -t155;
t182 = t16+t39+t43;
t183 = 1.0./t92;
t185 = -t156;
t201 = K_xkr.*rho_r.*t151;
t206 = K_xkr.*t151.*t_r;
t223 = K_ycf.*f.*g.*i_fy.*m_h.*t12.*t30.*4.0;
t225 = K_yvf.*f.*g.*i_fy.*m_h.*t13.*t30.*4.0;
t272 = K_xkr.*rho_r.*t172;
t276 = K_xkr.*rho_r.*t174;
t281 = K_xkr.*t172.*t_r;
t285 = K_xkr.*t174.*t_r;
t288 = -t172;
t290 = -t196;
t292 = -t222;
t293 = -t224;
t294 = -t228;
t295 = -t229;
t300 = -t237;
t303 = -t239;
t324 = K_xkr.*rho_r.*t198;
t333 = K_xkr.*t198.*t_r;
t337 = K_xkr.*rho_r.*t230;
t338 = K_xkr.*rho_r.*t231;
t342 = K_xkr.*t230.*t_r;
t344 = K_xkr.*t231.*t_r;
t345 = t3.*t212;
t346 = t3.*t213;
t347 = t3.*t220;
t348 = t3.*t221;
t350 = -t230;
t351 = -t231;
t352 = -t296;
t353 = -t297;
t354 = -t232;
t355 = -t233;
t363 = -t273;
t366 = -t274;
t368 = -t326;
t370 = -t280;
t373 = -t283;
t376 = -t332;
t381 = K_xkr.*rho_r.*t212;
t382 = K_xkr.*rho_r.*t213;
t385 = K_xkr.*rho_r.*t214;
t386 = K_xkr.*rho_r.*t215;
t387 = K_xkr.*rho_r.*t216;
t388 = K_xkr.*rho_r.*t217;
t389 = K_xkr.*rho_r.*t218;
t390 = K_xkr.*rho_r.*t219;
t394 = K_xkr.*t212.*t_r;
t396 = K_xkr.*t213.*t_r;
t397 = K_xkr.*t214.*t_r;
t398 = K_xkr.*t215.*t_r;
t399 = K_xkr.*t216.*t_r;
t400 = K_xkr.*t217.*t_r;
t401 = K_xkr.*t218.*t_r;
t402 = K_xkr.*t219.*t_r;
t405 = K_xkr.*rho_r.*t220;
t406 = K_xkr.*rho_r.*t221;
t411 = K_xkr.*t220.*t_r;
t413 = K_xkr.*t221.*t_r;
t441 = -t407;
t442 = -t408;
t444 = -t410;
t445 = -t412;
t455 = -t339;
t456 = -t340;
t457 = -t341;
t459 = -t343;
t464 = -t423;
t465 = -t424;
t466 = -t425;
t467 = -t427;
t468 = -t429;
t469 = -t430;
t470 = -t431;
t471 = -t432;
t472 = -t433;
t473 = -t434;
t362 = -t272;
t371 = -t281;
t380 = 1.0./t182;
t414 = e.*g.*m_h.*t114.*t_f.*4.0;
t443 = a.*g.*m_h.*t114.*t_f.*-4.0;
t451 = -t337;
t452 = -t338;
t458 = -t342;
t460 = -t344;
t463 = K_xkf.*b.*i_fy.*t12.*t114.*4.0;
t474 = A.*C_l.*K_ycf.*b.*i_fy.*rho.*t14.*t114;
t475 = A.*C_l.*K_yvf.*b.*i_fy.*rho.*t15.*t114;
t476 = K_ycf.*b.*g.*i_fy.*m_b.*t12.*t114.*4.0;
t477 = K_yvf.*b.*g.*i_fy.*m_b.*t13.*t114.*4.0;
t478 = K_ycf.*b.*g.*i_fy.*m_f.*t12.*t114.*4.0;
t479 = K_yvf.*b.*g.*i_fy.*m_f.*t13.*t114.*4.0;
t480 = K_ycf.*b.*g.*i_fy.*m_h.*t12.*t114.*4.0;
t481 = K_yvf.*b.*g.*i_fy.*m_h.*t13.*t114.*4.0;
t482 = A.*C_p.*K_ycf.*b.*i_fy.*rho.*t14.*t114.*2.0;
t483 = A.*C_p.*K_yvf.*b.*i_fy.*rho.*t15.*t114.*2.0;
t484 = A.*C_d.*K_ycf.*i_fy.*rho.*rho_r.*t14.*t114.*2.0;
t485 = A.*C_d.*K_yvf.*i_fy.*rho.*rho_r.*t15.*t114.*2.0;
t486 = A.*C_d.*K_ycf.*i_fy.*rho.*t14.*t114.*t_r.*2.0;
t487 = A.*C_d.*K_yvf.*i_fy.*rho.*t15.*t114.*t_r.*2.0;
t446 = -t414;
t488 = -t474;
t489 = -t475;
t490 = -t482;
t491 = -t483;
et1 = -t122-t123+t135+t136-t137+t138+t139+t140+t141-t142-t143+t144+t145-t146+t147+t148+t149+t150+t158+t159+t162+t163+t177-t178+t179-t180+t186+t187+t188+t189+t200+t201+t202-t203+t204+t205+t206+t207-t208-t209+t210-t211+t236+t238+t240+t241+t242+t243+t244+t245+t246+t247+t256+t257+t258+t259+t260+t261+t262+t263+t276+t277+t278+t279+t284+t285+t286+t287+t300+t303+t304+t305+t306+t307+t308+t309+t310+t311+t312+t313+t314+t315+t316+t317+t318+t319+t322+t323+t324+t325+t328+t330+t333+t335+t356+t357+t358+t359+t360+t361+t362+t363+t364+t365+t366+t367+t368+t369+t370+t371+t372+t373+t374+t375+t376+t377+t378+t379+t381+t382+t385+t386+t387+t388+t389+t390+t391+t392+t394+t396+t397+t398+t399+t400+t401+t402+t403+t404+t405+t406+t411+t413+t417+t418+t419+t421+t426+t428+t437+t438+t439+t440+t441+t442+t444+t445+t447+t448+t449+t450+t451+t452+t453+t454;
et2 = t455+t456+t457+t458+t459+t460+t461+t462+t464+t465+t466+t467+t468+t469+t470+t471-K_xkf.*K_xkr.*rho_f.*t23.*t44.*4.0-K_xkf.*K_xkr.*rho_f.*t25.*t44.*4.0-K_xkf.*K_xkr.*rho_r.*t24.*t42.*4.0-K_xkf.*K_xkr.*t23.*t44.*t_f.*4.0-K_xkf.*K_xkr.*t25.*t44.*t_f.*4.0-K_xkf.*K_xkr.*t24.*t42.*t_r.*4.0-K_xkf.*K_xkr.*a.*rho_r.*t4.*t24.*4.0-K_xkf.*K_xkr.*a.*rho_r.*t24.*t33.*4.0+K_xkf.*K_xkr.*a_n.*rho_r.*t4.*t24.*8.0-K_xkf.*K_xkr.*a.*t4.*t24.*t_r.*4.0-K_xkf.*K_xkr.*a.*t24.*t33.*t_r.*4.0+K_xkf.*K_xkr.*a_n.*t4.*t24.*t_r.*8.0-K_xkf.*K_xkr.*rho_f.*rho_r.*t42.*t_f.*4.0-K_xkf.*K_xkr.*rho_f.*rho_r.*t44.*t_r.*8.0+K_xkf.*K_xkr.*rho_f.*t16.*t23.*t33+K_xkf.*K_xkr.*rho_f.*t16.*t25.*t33-K_xkf.*K_xkr.*rho_r.*t4.*t24.*t34.*8.0+K_xkf.*K_xkr.*rho_r.*t5.*t24.*t34.*8.0+K_xkf.*K_xkr.*rho_r.*t18.*t24.*t33-K_xkf.*K_xkr.*rho_f.*t42.*t_f.*t_r.*4.0-K_xkf.*K_xkr.*rho_r.*t44.*t_f.*t_r.*8.0;
et3 = K_xkf.*K_xkr.*t6.*t21.*t24.*t28.*-4.0+K_xkf.*K_xkr.*t16.*t23.*t33.*t_f+K_xkf.*K_xkr.*t16.*t25.*t33.*t_f-K_xkf.*K_xkr.*t4.*t24.*t34.*t_r.*8.0+K_xkf.*K_xkr.*t5.*t24.*t34.*t_r.*8.0+K_xkf.*K_xkr.*t18.*t24.*t33.*t_r+K_xkf.*K_xkr.*K_zar.*rho_f.*t2.*t16.*t23+K_xkf.*K_xkr.*K_zar.*t2.*t16.*t23.*t_f-K_xkf.*K_xkr.*a.*rho_f.*rho_r.*t4.*t_f.*4.0-K_xkf.*K_xkr.*a.*rho_f.*rho_r.*t33.*t_f.*4.0+K_xkf.*K_xkr.*a.*rho_f.*rho_r.*t33.*t_r.*8.0+K_xkf.*K_xkr.*a_n.*rho_f.*rho_r.*t4.*t_f.*8.0-K_xkf.*K_xkr.*a.*rho_f.*t4.*t_f.*t_r.*4.0-K_xkf.*K_xkr.*a.*rho_f.*t33.*t_f.*t_r.*4.0+K_xkf.*K_xkr.*a.*rho_r.*t33.*t_f.*t_r.*8.0+K_xkf.*K_xkr.*a_n.*rho_f.*t4.*t_f.*t_r.*8.0-K_xkf.*K_xkr.*rho_f.*rho_r.*t4.*t34.*t_f.*8.0+K_xkf.*K_xkr.*rho_f.*rho_r.*t5.*t34.*t_f.*8.0+K_xkf.*K_xkr.*rho_f.*rho_r.*t18.*t33.*t_f-K_xkf.*K_xkr.*rho_f.*t6.*t21.*t28.*t_f.*4.0-K_xkf.*K_xkr.*rho_f.*t8.*t21.*t28.*t_r.*4.0;
et4 = K_xkf.*K_xkr.*rho_f.*t4.*t34.*t_f.*t_r.*-8.0+K_xkf.*K_xkr.*rho_f.*t5.*t34.*t_f.*t_r.*8.0+K_xkf.*K_xkr.*rho_f.*t18.*t33.*t_f.*t_r-K_xkf.*K_xkr.*t8.*t21.*t28.*t_f.*t_r.*4.0-K_xkf.*K_xkr.*K_zar.*a_n.*rho_f.*t2.*t23.*4.0+K_xkf.*K_xkr.*K_zaf.*b.*rho_f.*t23.*t28.*4.0+K_xkf.*K_xkr.*K_zaf.*b.*rho_f.*t25.*t28.*4.0+K_xkf.*K_xkr.*K_zar.*b.*rho_f.*t23.*t28.*4.0-K_xkf.*K_xkr.*K_zaf.*b.*rho_r.*t24.*t28.*4.0-K_xkf.*K_xkr.*K_zar.*a_n.*t2.*t23.*t_f.*4.0+K_xkf.*K_xkr.*K_zaf.*b.*t23.*t28.*t_f.*4.0+K_xkf.*K_xkr.*K_zaf.*b.*t25.*t28.*t_f.*4.0+K_xkf.*K_xkr.*K_zar.*b.*t23.*t28.*t_f.*4.0-K_xkf.*K_xkr.*K_zaf.*b.*t24.*t28.*t_r.*4.0-K_xkf.*K_xkr.*K_zar.*a_n.*rho_f.*rho_r.*t2.*t_r.*4.0-K_xkf.*K_xkr.*K_zaf.*b.*rho_f.*rho_r.*t28.*t_f.*4.0+K_xkf.*K_xkr.*K_zaf.*b.*rho_f.*rho_r.*t28.*t_r.*8.0+K_xkf.*K_xkr.*K_zar.*b.*rho_f.*rho_r.*t28.*t_r.*4.0-K_xkf.*K_xkr.*K_zar.*a_n.*rho_r.*t2.*t_f.*t_r.*4.0-K_xkf.*K_xkr.*K_zaf.*b.*rho_f.*t28.*t_f.*t_r.*4.0;
et5 = K_xkf.*K_xkr.*K_zaf.*b.*rho_r.*t28.*t_f.*t_r.*8.0+K_xkf.*K_xkr.*K_zar.*b.*rho_r.*t28.*t_f.*t_r.*4.0+K_xkf.*K_xkr.*K_zar.*rho_f.*rho_r.*t2.*t16.*t_r+K_xkf.*K_xkr.*K_zar.*rho_r.*t2.*t16.*t_f.*t_r-K_xkr.*K_ycf.*f.*g.*i_fy.*m_h.*rho_r.*t3.*t12.*t28.*4.0-K_xkr.*K_yvf.*f.*g.*i_fy.*m_h.*rho_r.*t3.*t13.*t28.*4.0+K_xkf.*K_ycr.*f.*g.*i_ry.*m_h.*rho_f.*t3.*t12.*t28.*4.0+K_xkf.*K_yvr.*f.*g.*i_ry.*m_h.*rho_f.*t3.*t13.*t28.*4.0+K_xkf.*K_ycr.*f.*g.*i_ry.*m_h.*t3.*t12.*t28.*t_f.*4.0-K_xkr.*K_ycf.*f.*g.*i_fy.*m_h.*t3.*t12.*t28.*t_r.*4.0+K_xkf.*K_yvr.*f.*g.*i_ry.*m_h.*t3.*t13.*t28.*t_f.*4.0-K_xkr.*K_yvf.*f.*g.*i_fy.*m_h.*t3.*t13.*t28.*t_r.*4.0;
et6 = t57+t58+t75+t76-t108-t109+t124+t125-t152-t153+t154+t181+t190+t191+t223+t225+t226+t227+t234+t235+t292+t293+t294+t295+t345+t346+t347+t348+t354+t355+t435+t436+t463+t472+t473+t476+t477+t478+t479+t480+t481+t484+t485+t486+t487+t488+t489+t490+t491-K_xkf.*rho_f.*t19.*t33.*4.0+K_xkf.*rho_f.*t16.*t44-K_xkf.*t3.*t24.*t42.*4.0-K_xkf.*t19.*t33.*t_f.*4.0+K_xkf.*t16.*t44.*t_f-K_xkf.*K_zaf.*a.*t24.*t31.*2.0+K_xkf.*K_zaf.*a_n.*t24.*t31.*2.0-K_xkf.*K_zaf.*b.*t24.*t114.*4.0-K_xkf.*a.*t3.*t4.*t24.*4.0+K_xkf.*a_n.*t3.*t4.*t24.*8.0-K_xkf.*b.*t4.*t24.*t31.*4.0+K_xkf.*b.*t5.*t24.*t31.*4.0-K_xkf.*rho_f.*t8.*t21.*t114.*4.0-K_xkf.*rho_f.*t3.*t42.*t_f.*4.0-K_xkf.*t8.*t21.*t114.*t_f.*4.0-K_xkf.*K_zaf.*a.*rho_f.*t31.*t_f.*2.0+K_xkf.*K_zaf.*a_n.*rho_f.*t31.*t_f.*2.0+K_xkf.*K_zaf.*b.*rho_f.*t18.*t28;
et7 = K_xkf.*K_zaf.*b.*rho_f.*t114.*t_f.*-4.0+K_xkf.*K_zaf.*b.*t18.*t28.*t_f-K_xkf.*a.*rho_f.*t3.*t4.*t_f.*4.0+K_xkf.*a_n.*rho_f.*t3.*t4.*t_f.*8.0-K_xkf.*b.*rho_f.*t4.*t31.*t_f.*4.0+K_xkf.*b.*rho_f.*t5.*t31.*t_f.*4.0;
et8 = t46+t53+t68+t69+t82+t83+t85+t86+t96+t103+t112+t116+t118+t127+t133+t134+t157+t3.*t56-K_ygf.*a_n.*rho_r.*4.0-K_ygr.*a_n.*rho_r.*4.0-K_ygf.*a_n.*t_r.*4.0-K_ygr.*a_n.*t_r.*4.0+K_ygf.*rho_r.*t16+K_ygr.*rho_r.*t16+K_ygf.*rho_r.*t43+K_ygr.*rho_r.*t43+K_ygf.*t16.*t_r+K_ygr.*t16.*t_r+K_ygf.*t43.*t_r+K_ygr.*t43.*t_r-a_n.*g.*m_r.*rho_r.*4.0-a.*g.*m_b.*t_r.*4.0-a.*g.*m_h.*t_r.*4.0+g.*h.*m_b.*t16+g.*h.*m_b.*t43+g.*m_r.*rho_r.*t16+g.*m_r.*rho_r.*t43+g.*m_b.*t18.*t_r+g.*m_h.*t18.*t_r-a_n.*g.*h.*m_b.*4.0-a.*a_n.*g.*m_h.*t3.*4.0+a.*b.*g.*m_h.*t31.*2.0+b.*f.*g.*m_h.*t28.*4.0+a.*g.*m_h.*t3.*t16+e.*g.*m_h.*t28.*t_r.*4.0+f.*g.*m_h.*t2.*t16-f.*g.*m_h.*t31.*t_r.*2.0+g.*m_h.*t16.*t28.*t_r-A.*C_l.*a.*rho.*rho_r.*t12.*2.0;
et9 = A.*C_l.*a_n.*rho.*rho_r.*t12.*2.0-A.*C_l.*a.*rho.*t12.*t_r-A.*C_p.*a.*rho.*t12.*t_r.*2.0+A.*C_l.*a_n.*rho.*t12.*t_r+A.*C_p.*a_n.*rho.*t12.*t_r.*2.0-A.*C_l.*rho.*rho_r.*t12.*t34.*2.0-A.*C_l.*rho.*t12.*t34.*t_r-A.*C_p.*rho.*t12.*t34.*t_r.*2.0;
et10 = t122+t123+t137+t142+t143+t146-t147-t148-t149-t150+t158+t159+t162+t163-t177+t178-t179+t180+t186+t187+t188+t189-t200+t201+t202+t203-t204-t205+t206+t207+t208+t209-t210+t211+t236+t238+t240+t241+t242+t243+t244+t245+t246+t247+t256+t257+t258+t259+t260+t261+t262+t263+t276+t277+t278+t279+t284+t285+t286+t287+t300+t303+t304+t305+t306+t307+t308+t309+t310+t311+t312+t313+t314+t315+t316+t317+t318+t319+t322+t323+t324+t325+t328+t330+t333+t335-t356-t357-t358-t359-t360-t361+t362+t363+t364+t365+t366+t367+t368+t369+t370+t371+t372+t373+t374+t375+t376+t377+t378+t379+t381+t382+t385+t386+t387+t388+t389+t390+t391+t392+t394+t396+t397+t398+t399+t400+t401+t402+t403+t404+t405+t406+t411+t413+t417+t418+t419+t421+t426+t428+t437+t438+t439+t440+t441+t442+t444+t445+t447+t448+t449+t450;
et11 = t451+t452+t453+t454+t455+t456+t457+t458+t459+t460+t461+t462+t464+t465+t466+t467+t468+t469+t470+t471-K_xkf.*K_xkr.*rho_f.*t23.*t42.*4.0-K_xkf.*K_xkr.*rho_f.*t25.*t42.*4.0-K_xkf.*K_xkr.*t23.*t42.*t_f.*4.0-K_xkf.*K_xkr.*t25.*t42.*t_f.*4.0-K_xkf.*K_xkr.*a.*rho_f.*t4.*t23.*4.0-K_xkf.*K_xkr.*a.*rho_f.*t4.*t25.*4.0-K_xkf.*K_xkr.*a.*t4.*t23.*t_f.*4.0-K_xkf.*K_xkr.*a.*t4.*t25.*t_f.*4.0-K_xkf.*K_xkr.*a.*m_b.*rho_f.*t2.*t12.*t23.*4.0-K_xkf.*K_xkr.*a.*m_b.*rho_f.*t2.*t12.*t25.*4.0-K_xkf.*K_xkr.*a.*m_f.*rho_f.*t2.*t12.*t23.*4.0-K_xkf.*K_xkr.*a.*m_f.*rho_f.*t2.*t12.*t25.*4.0-K_xkf.*K_xkr.*a.*m_h.*rho_f.*t2.*t12.*t23.*4.0-K_xkf.*K_xkr.*a.*m_h.*rho_f.*t2.*t12.*t25.*4.0-K_xkf.*K_xkr.*a_n.*m_f.*rho_r.*t2.*t12.*t22.*4.0-K_xkf.*K_xkr.*a_n.*m_f.*rho_r.*t2.*t12.*t24.*4.0-K_xkf.*K_xkr.*b.*m_b.*rho_f.*t12.*t23.*t28.*4.0-K_xkf.*K_xkr.*b.*m_b.*rho_f.*t12.*t25.*t28.*4.0;
et12 = K_xkf.*K_xkr.*b.*m_f.*rho_f.*t12.*t23.*t28.*-4.0-K_xkf.*K_xkr.*b.*m_f.*rho_f.*t12.*t25.*t28.*4.0+K_xkf.*K_xkr.*b.*m_f.*rho_r.*t12.*t22.*t28.*4.0+K_xkf.*K_xkr.*b.*m_f.*rho_r.*t12.*t24.*t28.*4.0-K_xkf.*K_xkr.*b.*m_h.*rho_f.*t12.*t23.*t28.*4.0-K_xkf.*K_xkr.*b.*m_h.*rho_f.*t12.*t25.*t28.*4.0-K_xkf.*K_xkr.*a.*m_b.*t2.*t12.*t23.*t_f.*4.0-K_xkf.*K_xkr.*a.*m_b.*t2.*t12.*t25.*t_f.*4.0-K_xkf.*K_xkr.*a.*m_f.*t2.*t12.*t23.*t_f.*4.0-K_xkf.*K_xkr.*a.*m_f.*t2.*t12.*t25.*t_f.*4.0-K_xkf.*K_xkr.*a.*m_h.*t2.*t12.*t23.*t_f.*4.0-K_xkf.*K_xkr.*a.*m_h.*t2.*t12.*t25.*t_f.*4.0-K_xkf.*K_xkr.*a_n.*m_f.*t2.*t12.*t22.*t_r.*4.0-K_xkf.*K_xkr.*a_n.*m_f.*t2.*t12.*t24.*t_r.*4.0-K_xkf.*K_xkr.*b.*m_b.*t12.*t23.*t28.*t_f.*4.0-K_xkf.*K_xkr.*b.*m_b.*t12.*t25.*t28.*t_f.*4.0-K_xkf.*K_xkr.*b.*m_f.*t12.*t23.*t28.*t_f.*4.0-K_xkf.*K_xkr.*b.*m_f.*t12.*t25.*t28.*t_f.*4.0;
et13 = K_xkf.*K_xkr.*b.*m_h.*t12.*t23.*t28.*t_f.*-4.0-K_xkf.*K_xkr.*b.*m_h.*t12.*t25.*t28.*t_f.*4.0+K_xkf.*K_xkr.*b.*m_f.*t12.*t22.*t28.*t_r.*4.0+K_xkf.*K_xkr.*b.*m_f.*t12.*t24.*t28.*t_r.*4.0+K_xkf.*K_xkr.*m_h.*rho_f.*rho_r.*t12.*t17.*t31.*2.0+K_xkf.*K_xkr.*m_b.*rho_f.*t2.*t12.*t18.*t23+K_xkf.*K_xkr.*m_b.*rho_f.*t2.*t12.*t18.*t25+K_xkf.*K_xkr.*m_f.*rho_f.*t2.*t12.*t18.*t23+K_xkf.*K_xkr.*m_f.*rho_f.*t2.*t12.*t18.*t25+K_xkf.*K_xkr.*m_f.*rho_r.*t2.*t12.*t16.*t22+K_xkf.*K_xkr.*m_f.*rho_r.*t2.*t12.*t16.*t24+K_xkf.*K_xkr.*m_h.*rho_f.*t2.*t12.*t18.*t23+K_xkf.*K_xkr.*m_h.*rho_f.*t2.*t12.*t18.*t25+K_xkf.*K_xkr.*m_h.*rho_r.*t12.*t17.*t31.*t_f.*2.0+K_xkf.*K_xkr.*m_h.*rho_f.*t12.*t17.*t31.*t_r.*2.0+K_xkf.*K_xkr.*m_b.*t2.*t12.*t18.*t23.*t_f+K_xkf.*K_xkr.*m_b.*t2.*t12.*t18.*t25.*t_f+K_xkf.*K_xkr.*m_f.*t2.*t12.*t18.*t23.*t_f+K_xkf.*K_xkr.*m_f.*t2.*t12.*t18.*t25.*t_f+K_xkf.*K_xkr.*m_h.*t2.*t12.*t18.*t23.*t_f+K_xkf.*K_xkr.*m_h.*t2.*t12.*t18.*t25.*t_f;
et14 = K_xkf.*K_xkr.*m_f.*t2.*t12.*t16.*t22.*t_r+K_xkf.*K_xkr.*m_f.*t2.*t12.*t16.*t24.*t_r+K_xkf.*K_xkr.*m_h.*t12.*t17.*t31.*t_f.*t_r.*2.0+K_xkf.*K_xkr.*a.*m_f.*rho_f.*rho_r.*t2.*t12.*t_f.*8.0-K_xkf.*K_xkr.*a.*m_b.*rho_f.*rho_r.*t2.*t12.*t_r.*8.0-K_xkf.*K_xkr.*a.*m_f.*rho_f.*rho_r.*t2.*t12.*t_r.*8.0-K_xkf.*K_xkr.*a.*m_h.*rho_f.*rho_r.*t2.*t12.*t_r.*8.0-K_xkf.*K_xkr.*a_n.*m_f.*rho_f.*rho_r.*t2.*t12.*t_f.*8.0+K_xkf.*K_xkr.*a_n.*m_b.*rho_f.*rho_r.*t2.*t12.*t_r.*8.0+K_xkf.*K_xkr.*a_n.*m_f.*rho_f.*rho_r.*t2.*t12.*t_r.*8.0+K_xkf.*K_xkr.*a_n.*m_h.*rho_f.*rho_r.*t2.*t12.*t_r.*8.0+K_xkf.*K_xkr.*b.*m_h.*rho_f.*rho_r.*t12.*t16.*t114+K_xkf.*K_xkr.*b.*m_f.*rho_f.*rho_r.*t12.*t28.*t_f.*8.0-K_xkf.*K_xkr.*b.*m_b.*rho_f.*rho_r.*t12.*t28.*t_r.*8.0-K_xkf.*K_xkr.*b.*m_f.*rho_f.*rho_r.*t12.*t28.*t_r.*8.0-K_xkf.*K_xkr.*b.*m_h.*rho_f.*rho_r.*t12.*t28.*t_r.*8.0;
et15 = K_xkf.*K_xkr.*a.*m_b.*rho_r.*t2.*t12.*t_f.*t_r.*-8.0+K_xkf.*K_xkr.*a.*m_f.*rho_f.*t2.*t12.*t_f.*t_r.*8.0-K_xkf.*K_xkr.*a.*m_f.*rho_r.*t2.*t12.*t_f.*t_r.*8.0-K_xkf.*K_xkr.*a.*m_h.*rho_r.*t2.*t12.*t_f.*t_r.*8.0+K_xkf.*K_xkr.*a_n.*m_b.*rho_r.*t2.*t12.*t_f.*t_r.*8.0-K_xkf.*K_xkr.*a_n.*m_f.*rho_f.*t2.*t12.*t_f.*t_r.*8.0+K_xkf.*K_xkr.*a_n.*m_f.*rho_r.*t2.*t12.*t_f.*t_r.*8.0+K_xkf.*K_xkr.*a_n.*m_h.*rho_r.*t2.*t12.*t_f.*t_r.*8.0+K_xkf.*K_xkr.*b.*m_h.*rho_r.*t12.*t16.*t114.*t_f+K_xkf.*K_xkr.*b.*m_h.*rho_f.*t12.*t16.*t114.*t_r-K_xkf.*K_xkr.*b.*m_b.*rho_r.*t12.*t28.*t_f.*t_r.*8.0+K_xkf.*K_xkr.*b.*m_f.*rho_f.*t12.*t28.*t_f.*t_r.*8.0-K_xkf.*K_xkr.*b.*m_f.*rho_r.*t12.*t28.*t_f.*t_r.*8.0-K_xkf.*K_xkr.*b.*m_h.*rho_r.*t12.*t28.*t_f.*t_r.*8.0+K_xkf.*K_xkr.*b.*m_h.*t12.*t16.*t114.*t_f.*t_r+K_xkf.*K_xkr.*f.*m_h.*rho_f.*rho_r.*t12.*t16.*t28;
et16 = K_xkf.*K_xkr.*f.*m_h.*rho_r.*t12.*t16.*t28.*t_f+K_xkf.*K_xkr.*f.*m_h.*rho_f.*t12.*t16.*t28.*t_r+K_xkf.*K_xkr.*h.*m_b.*rho_f.*rho_r.*t2.*t12.*t16+K_xkf.*K_xkr.*f.*m_h.*t12.*t16.*t28.*t_f.*t_r+K_xkf.*K_xkr.*h.*m_b.*rho_r.*t2.*t12.*t16.*t_f+K_xkf.*K_xkr.*h.*m_b.*rho_f.*t2.*t12.*t16.*t_r+K_xkf.*K_xkr.*h.*m_b.*t2.*t12.*t16.*t_f.*t_r-K_xkf.*K_xkr.*a.*a_n.*m_h.*rho_f.*rho_r.*t12.*t31.*2.0-K_xkr.*K_ycf.*f.*g.*i_fy.*m_h.*rho_r.*t12.*t114.*4.0-K_xkr.*K_yvf.*f.*g.*i_fy.*m_h.*rho_r.*t13.*t114.*4.0+K_xkf.*K_ycr.*f.*g.*i_ry.*m_h.*rho_f.*t12.*t114.*4.0+K_xkf.*K_yvr.*f.*g.*i_ry.*m_h.*rho_f.*t13.*t114.*4.0-K_xkf.*K_xkr.*a.*a_n.*m_h.*rho_r.*t12.*t31.*t_f.*2.0-K_xkf.*K_xkr.*a.*a_n.*m_h.*rho_f.*t12.*t31.*t_r.*2.0+K_xkf.*K_ycr.*f.*g.*i_ry.*m_h.*t12.*t114.*t_f.*4.0-K_xkr.*K_ycf.*f.*g.*i_fy.*m_h.*t12.*t114.*t_r.*4.0+K_xkf.*K_yvr.*f.*g.*i_ry.*m_h.*t13.*t114.*t_f.*4.0-K_xkr.*K_yvf.*f.*g.*i_fy.*m_h.*t13.*t114.*t_r.*4.0;
et17 = K_xkf.*K_xkr.*a.*e.*m_h.*rho_f.*rho_r.*t12.*t31.*2.0-K_xkf.*K_xkr.*a.*a_n.*m_h.*t12.*t31.*t_f.*t_r.*2.0-K_xkf.*K_xkr.*a_n.*e.*m_h.*rho_f.*rho_r.*t12.*t31.*2.0+K_xkf.*K_xkr.*b.*e.*m_h.*rho_f.*rho_r.*t12.*t114.*4.0-K_xkf.*K_xkr.*a_n.*f.*m_h.*rho_f.*rho_r.*t12.*t28.*4.0+K_xkf.*K_xkr.*a.*e.*m_h.*rho_r.*t12.*t31.*t_f.*2.0+K_xkf.*K_xkr.*a.*e.*m_h.*rho_f.*t12.*t31.*t_r.*2.0+K_xkf.*K_xkr.*b.*f.*m_h.*rho_f.*rho_r.*t12.*t29.*4.0-K_xkf.*K_xkr.*a_n.*e.*m_h.*rho_r.*t12.*t31.*t_f.*2.0-K_xkf.*K_xkr.*a_n.*e.*m_h.*rho_f.*t12.*t31.*t_r.*2.0+K_xkf.*K_xkr.*b.*e.*m_h.*rho_r.*t12.*t114.*t_f.*4.0+K_xkf.*K_xkr.*b.*e.*m_h.*rho_f.*t12.*t114.*t_r.*4.0-K_xkf.*K_xkr.*a_n.*f.*m_h.*rho_r.*t12.*t28.*t_f.*4.0-K_xkf.*K_xkr.*a_n.*f.*m_h.*rho_f.*t12.*t28.*t_r.*4.0-K_xkf.*K_xkr.*a_n.*h.*m_b.*rho_f.*rho_r.*t2.*t12.*4.0+K_xkf.*K_xkr.*a.*e.*m_h.*t12.*t31.*t_f.*t_r.*2.0+K_xkf.*K_xkr.*b.*f.*m_h.*rho_r.*t12.*t29.*t_f.*4.0;
H = ft_1({A,C_d,C_delta,C_l,C_p,K_xkf,K_xkr,K_yaf,K_ycf,K_ygf,K_ygr,K_yvf,K_zaf,K_zar,K_zgf,K_zgr,V,a,a_n,b,e,et1,et10,et11,et12,et13,et14,et15,et16,et17,et2,et3,et4,et5,et6,et7,et8,et9,f,g,h,i_fy,m_b,m_f,m_h,m_r,rho,rho_f,rho_r,t100,t103,t104,t105,t106,t107,t108,t109,t11,t110,t112,t113,t114,t115,t116,t117,t118,t119,t12,t120,t121,t124,t125,t127,t128,t13,t130,t131,t132,t133,t134,t151,t152,t153,t154,t157,t16,t166,t168,t169,t174,t175,t176,t18,t181,t183,t184,t185,t19,t190,t191,t192,t193,t194,t195,t198,t199,t2,t21,t212,t213,t214,t215,t216,t217,t218,t219,t22,t220,t221,t223,t225,t226,t227,t234,t235,t24,t27,t28,t288,t289,t29,t290,t291,t292,t293,t294,t295,t298,t299,t3,t30,t301,t302,t31,t32,t33,t336,t34,t345,t346,t347,t348,t349,t35,t350,t351,t352,t353,t354,t355,t36,t37,t38,t380,t4,t41,t42,t43,t435,t436,t44,t443,t446,t45,t46,t463,t47,t472,t473,t476,t477,t478,t479,t48,t480,t481,t484,t485,t486,t487,t488,t489,t49,t490,t491,t5,t50,t51,t53,t55,t56,t59,t6,t60,t61,t62,t63,t64,t65,t67,t68,t69,t72,t73,t77,t78,t79,t8,t80,t82,t83,t84,t85,t86,t87,t88,t89,t9,t91,t93,t94,t96,t_f,t_r});
end
function H = ft_1(ct)
[A,C_d,C_delta,C_l,C_p,K_xkf,K_xkr,K_yaf,K_ycf,K_ygf,K_ygr,K_yvf,K_zaf,K_zar,K_zgf,K_zgr,V,a,a_n,b,e,et1,et10,et11,et12,et13,et14,et15,et16,et17,et2,et3,et4,et5,et6,et7,et8,et9,f,g,h,i_fy,m_b,m_f,m_h,m_r,rho,rho_f,rho_r,t100,t103,t104,t105,t106,t107,t108,t109,t11,t110,t112,t113,t114,t115,t116,t117,t118,t119,t12,t120,t121,t124,t125,t127,t128,t13,t130,t131,t132,t133,t134,t151,t152,t153,t154,t157,t16,t166,t168,t169,t174,t175,t176,t18,t181,t183,t184,t185,t19,t190,t191,t192,t193,t194,t195,t198,t199,t2,t21,t212,t213,t214,t215,t216,t217,t218,t219,t22,t220,t221,t223,t225,t226,t227,t234,t235,t24,t27,t28,t288,t289,t29,t290,t291,t292,t293,t294,t295,t298,t299,t3,t30,t301,t302,t31,t32,t33,t336,t34,t345,t346,t347,t348,t349,t35,t350,t351,t352,t353,t354,t355,t36,t37,t38,t380,t4,t41,t42,t43,t435,t436,t44,t443,t446,t45,t46,t463,t47,t472,t473,t476,t477,t478,t479,t48,t480,t481,t484,t485,t486,t487,t488,t489,t49,t490,t491,t5,t50,t51,t53,t55,t56,t59,t6,t60,t61,t62,t63,t64,t65,t67,t68,t69,t72,t73,t77,t78,t79,t8,t80,t82,t83,t84,t85,t86,t87,t88,t89,t9,t91,t93,t94,t96,t_f,t_r] = ct{:};
et18 = K_xkf.*K_xkr.*b.*f.*m_h.*rho_f.*t12.*t29.*t_r.*4.0+K_xkf.*K_xkr.*b.*h.*m_b.*rho_f.*rho_r.*t12.*t28.*4.0-K_xkf.*K_xkr.*a_n.*e.*m_h.*t12.*t31.*t_f.*t_r.*2.0+K_xkf.*K_xkr.*b.*e.*m_h.*t12.*t114.*t_f.*t_r.*4.0-K_xkf.*K_xkr.*a_n.*f.*m_h.*t12.*t28.*t_f.*t_r.*4.0-K_xkf.*K_xkr.*a_n.*h.*m_b.*rho_r.*t2.*t12.*t_f.*4.0-K_xkf.*K_xkr.*a_n.*h.*m_b.*rho_f.*t2.*t12.*t_r.*4.0+K_xkf.*K_xkr.*b.*f.*m_h.*t12.*t29.*t_f.*t_r.*4.0+K_xkf.*K_xkr.*b.*h.*m_b.*rho_r.*t12.*t28.*t_f.*4.0+K_xkf.*K_xkr.*b.*h.*m_b.*rho_f.*t12.*t28.*t_r.*4.0-K_xkf.*K_xkr.*a_n.*h.*m_b.*t2.*t12.*t_f.*t_r.*4.0+K_xkf.*K_xkr.*b.*h.*m_b.*t12.*t28.*t_f.*t_r.*4.0;
et19 = t46+t53+t67+t68+t69+t82+t83+t85+t86+t96+t103+t112+t116+t118+t127+t133+t134+t157+t28.*t47+t32.*t56-t28.*t83+t28.*t84+t3.*t110+t3.*t130+t3.*t131+t28.*t113-K_zgf.*a.*t31.*2.0-K_zaf.*a_n.*t28.*4.0+K_zgf.*a_n.*t31.*2.0-K_zgf.*b.*t3.*4.0+K_zaf.*b.*t29.*4.0+K_zgf.*b.*t32.*4.0-K_ygf.*t3.*t19.*4.0+K_zaf.*t16.*t28-a_n.*t2.*t4.*4.0-b.*t5.*t28.*4.0+t2.*t5.*t18+K_ygf.*a_n.*b.*t31.*2.0+K_ygf.*a_n.*t3.*t16-a.*a_n.*g.*m_h.*t32.*4.0+a_n.*b.*g.*m_b.*t31.*2.0+a_n.*b.*g.*m_h.*t31.*2.0-a.*g.*m_f.*rho_f.*t28.*4.0-b.*g.*m_f.*rho_f.*t29.*4.0+b.*g.*m_b.*t29.*t_f.*4.0+b.*g.*m_h.*t29.*t_f.*4.0+e.*g.*m_h.*t30.*t_f.*4.0+f.*g.*m_h.*t18.*t29+g.*m_h.*t16.*t30.*t_f-f.*g.*m_h.*t3.*t29.*t_f.*4.0;
et20 = A.*C_p.*a.*a_n.*rho.*t3.*t12.*-2.0-(A.*C_l.*a_n.*b.*rho.*t12.*t31)./2.0-A.*C_p.*a.*rho.*t12.*t28.*t_f.*2.0+A.*C_p.*b.*rho.*t12.*t31.*t38-A.*C_l.*b.*rho.*t12.*t29.*t_f-A.*C_p.*b.*rho.*t12.*t29.*t_f.*2.0;
et21 = t88+t91+t108+t109-t124-t125+t152+t153+t154+t181-t190-t191+t223+t225+t226+t227+t234+t235+t292+t293+t294+t295+t345+t346+t347+t348+t354+t355+t435+t436+t463+t472+t473+t476+t477+t478+t479+t480+t481+t484+t485+t486+t487+t488+t489+t490+t491-K_xkf.*rho_f.*t5.*t19.*4.0-K_xkf.*t5.*t19.*t_f.*4.0-K_xkf.*a.*rho_f.*t44.*8.0-K_xkf.*a.*t44.*t_f.*8.0+K_xkf.*K_zaf.*rho_f.*t21.*t29.*4.0+K_xkf.*K_zaf.*t21.*t29.*t_f.*4.0-K_xkf.*a.*a_n.*rho_f.*t4.*4.0-K_xkf.*a.*a_n.*t4.*t_f.*4.0+K_xkf.*a.*rho_f.*t16.*t33+K_xkf.*a.*t16.*t33.*t_f+K_xkf.*K_zaf.*a.*b.*rho_f.*t28.*8.0-K_xkf.*K_zaf.*a_n.*b.*rho_f.*t28.*8.0+K_xkf.*K_zaf.*a.*b.*t28.*t_f.*8.0-K_xkf.*K_zaf.*a_n.*b.*t28.*t_f.*8.0+K_xkf.*a.*m_f.*t12.*t22.*t31.*2.0+K_xkf.*a.*m_f.*t12.*t24.*t31.*2.0-K_xkf.*a_n.*m_f.*t12.*t22.*t31.*2.0;
et22 = K_xkf.*a_n.*m_f.*t12.*t24.*t31.*-2.0+K_xkf.*b.*m_f.*t12.*t22.*t114.*4.0+K_xkf.*b.*m_f.*t12.*t24.*t114.*4.0-K_xkf.*a.*a_n.*m_f.*rho_f.*t2.*t12.*4.0-K_xkf.*a_n.*b.*m_f.*rho_f.*t12.*t28.*4.0-K_xkf.*a.*a_n.*m_f.*t2.*t12.*t_f.*4.0-K_xkf.*a_n.*b.*m_f.*t12.*t28.*t_f.*4.0-K_xkf.*a_n.*e.*m_h.*rho_f.*t2.*t12.*4.0+K_xkf.*b.*e.*m_h.*rho_f.*t12.*t28.*4.0-K_xkf.*a_n.*e.*m_h.*t2.*t12.*t_f.*4.0+K_xkf.*b.*e.*m_h.*t12.*t28.*t_f.*4.0+K_xkf.*a_n.*m_f.*rho_f.*t2.*t12.*t18-K_xkf.*a_n.*m_f.*rho_f.*t12.*t31.*t_f.*4.0+K_xkf.*b.*m_f.*rho_f.*t12.*t114.*t_f.*8.0+K_xkf.*a_n.*m_f.*t2.*t12.*t18.*t_f+K_xkf.*e.*m_h.*rho_f.*t2.*t12.*t16+K_xkf.*e.*m_h.*t2.*t12.*t16.*t_f+K_xkf.*m_f.*rho_f.*t12.*t16.*t31.*t_f;
et23 = t77+t78+t79+t80+t89+t115+t151+t168+t169+t174+t175+t194+t195+t198+t199+t212+t213+t214+t215+t216+t217+t218+t219+t220+t221+t288+t289+t290+t291+t298+t299+t301+t302+t350+t351+t352+t353-K_xkf.*t24.*t42.*4.0-K_xkf.*rho_f.*rho_r.*t44.*4.0-K_xkf.*rho_f.*t42.*t_f.*4.0-K_xkf.*rho_r.*t44.*t_f.*4.0-K_xkf.*rho_f.*t44.*t_r.*4.0+K_xkf.*t4.*t18.*t24+K_xkf.*t5.*t24.*t43+K_xkf.*t18.*t24.*t33-K_xkf.*t44.*t_f.*t_r.*4.0-K_xkf.*a.*t24.*t33.*4.0-K_xkf.*K_zaf.*b.*t24.*t28.*4.0-K_xkf.*a_n.*rho_f.*rho_r.*t4.*4.0-K_xkf.*a.*rho_f.*t33.*t_f.*4.0-K_xkf.*a_n.*rho_r.*t4.*t_f.*4.0-K_xkf.*a_n.*rho_f.*t4.*t_r.*4.0-K_xkf.*a_n.*t4.*t_f.*t_r.*4.0-K_xkf.*rho_f.*rho_r.*t5.*t34.*4.0+K_xkf.*rho_f.*rho_r.*t16.*t33+K_xkf.*rho_f.*t4.*t18.*t_f+K_xkf.*rho_f.*t5.*t43.*t_f+K_xkf.*rho_f.*t18.*t33.*t_f-K_xkf.*rho_r.*t5.*t34.*t_f.*4.0+K_xkf.*rho_r.*t16.*t33.*t_f;
et24 = K_xkf.*rho_f.*t5.*t34.*t_r.*-4.0+K_xkf.*rho_f.*t16.*t33.*t_r-K_xkf.*t5.*t34.*t_f.*t_r.*4.0+K_xkf.*t16.*t33.*t_f.*t_r+K_xkf.*K_zaf.*b.*rho_f.*rho_r.*t28.*4.0-K_xkf.*K_zaf.*b.*rho_f.*t28.*t_f.*4.0+K_xkf.*K_zaf.*b.*rho_r.*t28.*t_f.*4.0+K_xkf.*K_zaf.*b.*rho_f.*t28.*t_r.*4.0+K_xkf.*K_zaf.*b.*t28.*t_f.*t_r.*4.0-K_ycf.*f.*g.*i_fy.*m_h.*t3.*t12.*t28.*4.0-K_yvf.*f.*g.*i_fy.*m_h.*t3.*t13.*t28.*4.0;
mt1 = [0.0,0.0,t41.*(K_ygf.*a_n.*-4.0+K_ygf.*t16+K_ygf.*t43+K_zgf.*t2.*4.0+K_zgr.*t2.*4.0-A.*C_l.*a.*rho.*t12-A.*C_p.*a.*rho.*t12.*2.0+A.*C_l.*a_n.*rho.*t12+A.*C_p.*a_n.*rho.*t12.*2.0-A.*C_l.*rho.*t12.*t34-A.*C_p.*rho.*t12.*t34.*2.0+A.*C_d.*rho.*t2.*t12.*t_r.*2.0).*(-1.0./4.0),-K_ygf-K_ygr+(A.*C_l.*rho.*t12)./2.0,t183.*(et8+et9).*(-1.0./4.0),t380.*(t51+t56+t63+t64+t65+t72+t73+t87+t93+t94+t100+t110+t117+t119+t120+t121+t128+t130+t131+t132+t166+t176+t184+t185+t192+t193+t349+t443+t446-K_ygf.*t19.*4.0-K_zgf.*a.*t2.*4.0+K_ygf.*a_n.*t16-K_zgf.*b.*t28.*4.0+K_ygf.*t18.*t34+K_zgf.*t2.*t18),0.0,0.0,-t4+t5-t45+t48+K_zgf.*t3+K_ygf.*b.*t3+K_ygf.*t3.*t38.*t41+K_ygf.*a.*t3.*t41];
mt2 = [-K_yaf.*t2+K_ygf.*t3,t380.*(t51+t56+t63+t64+t65+t72+t73+t87+t93+t94+t100+t110+t117+t119+t120+t121+t128+t130+t131+t132+t166+t176+t184+t185+t192+t193+t349+t443+t446-b.*t6.*t28.*4.0-b.*t9.*t28.*4.0-rho_r.*t2.*t4.*4.0+rho_r.*t2.*t5.*4.0-t2.*t4.*t_r.*4.0+t2.*t5.*t_r.*4.0+K_ygf.*rho_r.*t3.*t16+K_ygf.*t3.*t16.*t_r-K_ygf.*a_n.*rho_r.*t3.*4.0+K_ygf.*b.*rho_r.*t31.*2.0-K_ygf.*a_n.*t3.*t_r.*4.0+K_ygf.*b.*t31.*t_r.*2.0),-t380.*(et19+et20),0.0,0.0,(t37.*(t42+t55+a.*t4+a.*t33-a_n.*t4.*2.0+t4.*t34.*2.0-t5.*t34.*2.0+K_zaf.*b.*t28+K_yaf.*t21.*t28+a.*m_f.*t2.*t12+a.*m_h.*t12.*t29+b.*m_b.*t12.*t28+b.*m_f.*t12.*t28+b.*m_h.*t12.*t28+e.*m_h.*t12.*t29+m_f.*t2.*t12.*t38-f.*m_h.*t3.*t12.*t28))./t28];
mt3 = [V.*m_b+V.*m_f+V.*m_h+V.*m_r+K_yaf.*b.*t37+t4.*t37.*t41-t5.*t37.*t41,t35.*t36.*t37.*t41.*t49.*t50.*t183.*(et10+et11+et12+et13+et14+et15+et16+et17+et18).*(-1.0./4.0),(t35.*t37.*t41.*t49.*t183.*(et21+et22))./4.0,0.0,0.0,t37.*t41.*(t4-t5+t33+t45+K_zar.*t2),t27.*t37,t11.*t27.*t37,-t37.*(t5+t48),1.0,0.0,(t35.*t36.*t37.*t41.*t49.*t50.*t183.*(et1+et2+et3+et4+et5))./4.0,t37.*t336,t11.*t37.*t336,(t35.*t37.*t49.*t183.*(et23+et24))./4.0,0.0,1.0,t35.*t37.*t41.*t49.*t183.*(et6+et7).*(-1.0./4.0),-K_yaf.*t37.*(a_n-t3.*t_f)];
mt4 = [t35.*t37.*t49.*t183.*(t59+t60+t61+t62+t89+t104+t105+t106+t107+t115+t151+t168+t169+t174+t175+t194+t195+t198+t199+t212+t213+t214+t215+t216+t217+t218+t219+t220+t221+t288+t289+t290+t291+t298+t299+t301+t302+t350+t351+t352+t353-K_xkf.*rho_f.*rho_r.*t42.*4.0-K_xkf.*rho_r.*t42.*t_f.*4.0-K_xkf.*rho_f.*t42.*t_r.*4.0-K_xkf.*t42.*t_f.*t_r.*4.0-K_xkf.*b.*t6.*t24.*t31.*2.0-K_xkf.*rho_r.*t3.*t4.*t24.*4.0+K_xkf.*rho_r.*t3.*t5.*t24.*4.0-K_xkf.*t3.*t4.*t24.*t_r.*4.0+K_xkf.*t3.*t5.*t24.*t_r.*4.0-K_xkf.*b.*rho_f.*t6.*t31.*t_f.*2.0-K_xkf.*b.*rho_f.*t8.*t31.*t_r.*2.0-K_xkf.*b.*t8.*t31.*t_f.*t_r.*2.0-K_xkf.*rho_f.*rho_r.*t3.*t4.*t_f.*4.0+K_xkf.*rho_f.*rho_r.*t3.*t5.*t_f.*4.0-K_xkf.*rho_f.*t3.*t4.*t_f.*t_r.*4.0+K_xkf.*rho_f.*t3.*t5.*t_f.*t_r.*4.0-K_ycf.*f.*g.*i_fy.*m_h.*t12.*t114.*4.0-K_yvf.*f.*g.*i_fy.*m_h.*t13.*t114.*4.0).*(-1.0./4.0)];
mt5 = [t37.*(t42+t55+C_delta.*V-t3.*t5.*t_f+t3.*t33.*t_f)];
H = reshape([mt1,mt2,mt3,mt4,mt5],6,6);
end
