function out1 = sharpMotorcycleStateMatrix(in1,in2)
%sharpMotorcycleStateMatrix
%    OUT1 = sharpMotorcycleStateMatrix(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    19-Feb-2024 01:10:18

%    states = [x y psi varphi delta theta_r psi_f varphi_f theta_f u_1 u_2 u_3 u_4 u_5 Y_r Y_f s d xi]
%    params = [C_delta C_f1 C_f2 C_r1 C_r2 C_rxz I_fx I_fz I_rx I_rz Z_f a a_n b e f g h i_fy i_ry kappa m_f m_r r_f r_r sigma varepsilon]
%
C_delta = in2(1,:);
C_f1 = in2(2,:);
C_f2 = in2(3,:);
C_r1 = in2(4,:);
C_r2 = in2(5,:);
C_rxz = in2(6,:);
I_fx = in2(7,:);
I_fz = in2(8,:);
I_rx = in2(9,:);
I_rz = in2(10,:);
Z_f = in2(11,:);
a = in2(12,:);
a_n = in2(13,:);
b = in2(14,:);
d = in1(18,:);
e = in2(15,:);
f = in2(16,:);
g = in2(17,:);
h = in2(18,:);
i_fy = in2(19,:);
i_ry = in2(20,:);
kappa = in2(21,:);
m_f = in2(22,:);
m_r = in2(23,:);
r_f = in2(24,:);
r_r = in2(25,:);
sigma = in2(26,:);
u_1 = in1(10,:);
varepsilon = in2(27,:);
xi = in1(19,:);
t2 = cos(varepsilon);
t3 = cos(xi);
t4 = sin(varepsilon);
t5 = sin(xi);
t6 = a+e;
t7 = Z_f.*r_f;
t8 = d.*kappa;
t9 = g.*m_f;
t10 = h.*m_r;
t11 = m_f+m_r;
t12 = m_r.*u_1;
t13 = e.^2;
t14 = h.^2;
t15 = m_f.^2;
t32 = -r_f;
t33 = 1.0./r_f;
t34 = 1.0./r_r;
t35 = 1.0./sigma;
t16 = t2.^2;
t17 = t4.^2;
t18 = I_fz.*t2;
t19 = I_fz.*t4;
t20 = e.*t9;
t21 = a.*t2;
t22 = a_n.*t2;
t23 = g.*t10;
t24 = e.*t2;
t25 = f.*t2;
t26 = t10.*u_1;
t27 = a.*t4;
t28 = a_n.*t4;
t29 = e.*t4;
t30 = f.*t4;
t31 = r_f.*t4;
t36 = t4.*t7;
t37 = -t7;
t38 = -t9;
t39 = 1.0./t2;
t40 = m_f.*t13;
t41 = h.*t10;
t46 = I_fx.*t2.*t4;
t50 = t8-1.0;
t54 = t35.*u_1;
t62 = i_fy.*t33.*u_1;
t63 = i_ry.*t34.*u_1;
t73 = t2.*t6.*t9;
t42 = m_f.*t16;
t43 = r_f.*t16;
t44 = m_f.*t17;
t45 = r_f.*t17;
t47 = t4.*t18;
t48 = t4.*t20;
t49 = t9.*t30;
t51 = -t20;
t52 = -t22;
t53 = -t25;
t55 = -t27;
t56 = -t28;
t57 = -t31;
t58 = I_fx.*t16;
t59 = I_fz.*t16;
t60 = I_fx.*t17;
t61 = I_fz.*t17;
t64 = -t36;
t65 = I_fz+t40;
t67 = t9.*t16;
t68 = t7.*t17;
t71 = t9.*t17;
t75 = -t46;
t77 = -t54;
t78 = t16.*t20;
t81 = t17.*t37;
t82 = t21+t30;
t83 = 1.0./t50;
t85 = t4.*t62;
t89 = a.*t17.*t39;
t90 = a_n.*t17.*t39;
t105 = -Z_f.*(t28+t17.*t32);
t66 = e.*t42;
t69 = e.*t44;
t70 = t42.*u_1;
t72 = t44.*u_1;
t74 = t2.*t48;
t76 = a_n+t57;
t84 = t83.^2;
t91 = t25+t55;
t93 = t45+t56;
t94 = -t90;
t95 = t24+t82;
t96 = t42+t44;
t97 = t43+t45;
t98 = t4.*t9.*t82;
t103 = t27+t29+t53;
t128 = t38+t67+t71;
t151 = t48+t81+t105;
t79 = t66.*u_1;
t80 = t69.*u_1;
t86 = Z_f.*t76;
t99 = Z_f.*t97;
t100 = e.*t96;
t101 = t66+t69;
t102 = t96.*u_1;
t104 = t95.^2;
t106 = e.*m_f.*t95;
t107 = t9.*t95;
t108 = t4.*t9.*t91;
t110 = m_f.*t103;
t112 = t32+t97;
t114 = t103.^2;
t115 = m_f.*t2.*t95;
t118 = t38.*t95;
t120 = t9.*t103;
t121 = t42.*t95;
t122 = t44.*t95;
t129 = t70.*t95;
t130 = t72.*t95;
t132 = t42.*t103;
t133 = t44.*t103;
t136 = t70.*t103;
t137 = t72.*t103;
t145 = t95.*t96;
t156 = t21+t52+t89+t94;
t87 = -t79;
t88 = -t80;
t92 = -t86;
t109 = t101.*u_1;
t111 = -t102;
t113 = m_f.*t104;
t116 = t115.*u_1;
t117 = t4.*t107;
t119 = e.*t110;
t123 = t103.*t110;
t124 = -t115;
t125 = t12+t102;
t126 = t4.*t118;
t127 = t4.*t120;
t131 = t18+t106;
t134 = t10+t110;
t135 = t36+t51+t86;
t141 = -t136;
t142 = -t137;
t144 = t15.*t65.*t104;
t147 = -t145;
t149 = t95.*t110;
t150 = t102.*t103;
t155 = t49+t73+t118;
t168 = t132+t133;
t171 = t23+t37+t99+t120;
t138 = t134.^2;
t139 = t19+t119;
t140 = t131.^2;
t153 = I_rz+t59+t60+t113;
t154 = t70+t72+t111;
t157 = t100+t124;
t159 = I_rx+t41+t58+t61+t123;
t161 = e.*t15.*t95.*t131.*2.0;
t167 = m_f.*t65.*t95.*t134;
t170 = t74+t98+t126;
t175 = e.*m_f.*t131.*t134;
t179 = t168.*u_1;
t181 = C_rxz+t47+t75+t149;
t183 = t85+t87+t88+t109;
t191 = t26+t62+t63+t150;
t202 = m_f.*t95.*t131.*t134;
t226 = t64+t78+t92+t108+t127;
t232 = t121+t122+t147;
t143 = t139.^2;
t146 = t11.*t140;
t152 = t65.*t138;
t158 = t157.*u_1;
t160 = t13.*t15.*t153;
t163 = -t161;
t164 = e.*t15.*t95.*t139;
t165 = t15.*t104.*t139;
t166 = t11.*t65.*t153;
t176 = t13.*t15.*t159;
t177 = t131.*t138;
t178 = t134.*t140;
t182 = t138.*t140;
t184 = -t175;
t185 = t181.^2;
t186 = e.*m_f.*t134.*t139.*2.0;
t187 = t11.*t131.*t139;
t190 = t11.*t65.*t159;
t196 = t13.*t15.*t181;
t198 = t138.*t153;
t199 = e.*m_f.*t134.*t153;
t200 = e.*t15.*t95.*t159;
t205 = t15.*t104.*t159;
t206 = e.*m_f.*t139.*t153;
t207 = t11.*t65.*t181;
t209 = t11.*t139.*t153;
t211 = m_f.*t95.*t134.*t139;
t213 = m_f.*t95.*t131.*t139;
t214 = t65.*t134.*t153;
t216 = m_f.*t65.*t95.*t159;
t219 = e.*m_f.*t131.*t159;
t222 = t140.*t159;
t224 = t11.*t131.*t159;
t225 = t144.*t159;
t227 = e.*t15.*t95.*t181;
t231 = t131.*t134.*t139;
t233 = m_f.*t65.*t95.*t181;
t235 = t232.*u_1;
t236 = e.*m_f.*t131.*t181;
t237 = e.*m_f.*t134.*t181;
t238 = t11.*t131.*t181;
t241 = e.*m_f.*t139.*t181;
t242 = e.*m_f.*t153.*t159;
t244 = t11.*t153.*t159;
t246 = t11.*t139.*t181;
t247 = m_f.*t95.*t131.*t159;
t250 = t65.*t134.*t181;
t253 = t134.*t139.*t153;
t254 = t65.*t153.*t159;
t256 = t159.*t161;
t262 = t139.*t202.*2.0;
t264 = m_f.*t95.*t134.*t181.*2.0;
t265 = m_f.*t95.*t139.*t181;
t268 = t131.*t134.*t181;
t269 = t167.*t181.*2.0;
t271 = t131.*t139.*t181.*2.0;
t272 = t175.*t181.*2.0;
t276 = t62+t63+t141+t142+t179;
t148 = t11.*t143;
t162 = m_f.*t95.*t143;
t169 = -t164;
t172 = -t165;
t173 = t15.*t104.*t143;
t174 = -t166;
t188 = -t182;
t189 = -t186;
t192 = -t190;
t193 = e.*m_f.*t185;
t194 = t11.*t185;
t197 = t13.*t15.*t185;
t201 = t65.*t185;
t203 = t143.*t153;
t208 = -t199;
t212 = -t207;
t217 = -t211;
t218 = -t213;
t220 = -t214;
t221 = t152.*t153;
t223 = -t216;
t228 = t85+t116+t158;
t229 = -t224;
t230 = t146.*t159;
t234 = -t231;
t239 = -t235;
t240 = -t236;
t243 = -t237;
t245 = -t238;
t248 = -t241;
t249 = -t244;
t251 = -t247;
t252 = t159.*t160;
t255 = -t253;
t257 = -t254;
t258 = t153.*t186;
t259 = t131.*t200.*-2.0;
t260 = t139.*t199.*-2.0;
t261 = t159.*t166;
t266 = -t264;
t267 = t164.*t181.*2.0;
t270 = -t269;
t273 = -t271;
t274 = t181.*t187.*2.0;
t180 = -t173;
t195 = -t193;
t204 = -t197;
t210 = t65.*t194;
t215 = t148.*t153;
t263 = t159.*t174;
t275 = -t274;
t277 = t129+t130+t239;
t278 = t144+t146+t160+t163+t174;
t279 = t148+t152+t176+t189+t192;
t280 = t167+t169+t184+t187+t196+t212;
t281 = t194+t198+t205+t249+t266;
t282 = t201+t203+t222+t257+t273;
t283 = t172+t202+t208+t209+t227+t245;
t284 = t178+t206+t218+t220+t233+t240;
t285 = t177+t200+t217+t229+t243+t246;
t286 = t162+t219+t223+t234+t248+t250;
t287 = t195+t242+t251+t255+t265+t268;
t288 = t180+t188+t204+t210+t215+t221+t225+t230+t252+t259+t260+t262+t263+t267+t270+t272+t275;
t289 = 1.0./t288;
t290 = t282.*t289;
t291 = -t2.*t62.*t289.*(t165+t199-t202-t209-t227+t238);
t292 = t284.*t289;
t294 = t286.*t289;
t296 = -t289.*(t193-t242+t247+t253-t265-t268);
t293 = -t292;
t295 = -t294;
mt1 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,u_1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-t128.*t290+t155.*t294+t171.*t293+t135.*t289.*(t193-t242+t247+t253-t265-t268),t128.*t294+t171.*t289.*(t164-t167+t175-t187-t196+t207)+t135.*t285.*t289-t155.*t279.*t289,t128.*t292-t155.*t289.*(t164-t167+t175-t187-t196+t207)+t135.*t289.*(t165+t199-t202-t209-t227+t238)+t171.*t278.*t289];
mt2 = [-t171.*t289.*(t165+t199-t202-t209-t227+t238)+t128.*t289.*(t193-t242+t247+t253-t265-t268)-t135.*t281.*t289+t155.*t285.*t289,C_r2.*t54,C_f2.*t54,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t170.*t294+t226.*t293+t289.*(-t48+t68+Z_f.*(t28+t17.*t32)).*(t193-t242+t247+t253-t265-t268),t226.*t289.*(t164-t167+t175-t187-t196+t207)-t170.*t279.*t289+t285.*t289.*(-t48+t68+Z_f.*(t28+t17.*t32))];
mt3 = [-t170.*t289.*(t164-t167+t175-t187-t196+t207)+t289.*(-t48+t68+Z_f.*(t28+t17.*t32)).*(t165+t199-t202-t209-t227+t238)+t226.*t278.*t289,-t226.*t289.*(t165+t199-t202-t209-t227+t238)+t170.*t285.*t289-t281.*t289.*(-t48+t68+Z_f.*(t28+t17.*t32)),0.0,t35.*(C_f1.*t2.*u_1+C_f2.*t4.*u_1),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,-t34,-t33,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-t3.*t83,t5,kappa.*t3.*t83,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,t154.*t294,-t154.*t279.*t289];
mt4 = [-t154.*t289.*(t164-t167+t175-t187-t196+t207),t154.*t285.*t289,-C_r1.*t35,-C_f1.*t35,t5.*t83,t3,-kappa.*t5.*t83,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,-t125.*t290+t191.*t292+t277.*t294+t228.*t289.*(t193-t242+t247+t253-t265-t268),t125.*t294-t191.*t289.*(t164-t167+t175-t187-t196+t207)+t228.*t285.*t289-t277.*t279.*t289,t125.*t292-t277.*t289.*(t164-t167+t175-t187-t196+t207)+t228.*t289.*(t165+t199-t202-t209-t227+t238)-t191.*t278.*t289];
mt5 = [t191.*t289.*(t165+t199-t202-t209-t227+t238)+t125.*t289.*(t193-t242+t247+t253-t265-t268)-t228.*t281.*t289+t277.*t285.*t289,C_r1.*b.*t35,-C_f1.*t35.*t156,d.*t83,0.0,-t8.*t83+1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,t276.*t295+t2.*t62.*t296,t276.*t279.*t289-t2.*t62.*t285.*t289,t291+t276.*t289.*(t164-t167+t175-t187-t196+t207),-t276.*t285.*t289+t2.*t62.*t281.*t289,0.0,-C_f1.*t35.*t112,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,t294.*(t79+t80-t85-t109)+C_delta.*t289.*(t193-t242+t247+t253-t265-t268)+t2.*t62.*t292];
mt6 = [-t279.*t289.*(t79+t80-t85-t109)+C_delta.*t285.*t289-t2.*t62.*t289.*(t164-t167+t175-t187-t196+t207),C_delta.*t289.*(t165+t199-t202-t209-t227+t238)-t289.*(t79+t80-t85-t109).*(t164-t167+t175-t187-t196+t207)-t2.*t62.*t278.*t289,t285.*t289.*(t79+t80-t85-t109)-C_delta.*t281.*t289+t2.*t62.*t289.*(t165+t199-t202-t209-t227+t238),0.0,C_f1.*a_n.*t35,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t290+b.*t294,t295-b.*t279.*t289,t293-b.*t289.*(t164-t167+t175-t187-t196+t207),t296+b.*t285.*t289];
mt7 = [t77,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t290+t112.*t293+t156.*t295+a_n.*t289.*(t193-t242+t247+t253-t265-t268),t295+t112.*t289.*(t164-t167+t175-t187-t196+t207)+a_n.*t285.*t289+t156.*t279.*t289,t293+a_n.*t289.*(t165+t199-t202-t209-t227+t238)+t156.*t289.*(t164-t167+t175-t187-t196+t207)+t112.*t278.*t289,t296-t112.*t289.*(t165+t199-t202-t209-t227+t238)-a_n.*t281.*t289-t156.*t285.*t289,0.0,t77,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,kappa.*t3.*t84.*u_1,0.0,-kappa.^2.*t3.*t84.*u_1,0.0];
mt8 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t5.*t83.*u_1,t3.*u_1,-kappa.*t5.*t83.*u_1];
out1 = reshape([mt1,mt2,mt3,mt4,mt5,mt6,mt7,mt8],17,17);
