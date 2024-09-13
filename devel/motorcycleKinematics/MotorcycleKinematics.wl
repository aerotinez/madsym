(* ::Package:: *)

(* ::Input:: *)
(*BeginPackage["MotorcycleKinematics`"];*)
(**)
(*motorcycleKinematicsMassMatrix::usage="motorcycleKinematicsMassMatrix[in1, in2] computes the mass matrix based on inputs in1 and in2.";*)
(**)
(*Begin["`Private`"];*)
(**)
(*motorcycleKinematicsMassMatrix[in1_,in2_]:=Module[{a,aN,b,beta,delta,f,psi,rf,rr,theta,varphi,varepsilon,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13,t17,t14,t15,t16,t18,t19,t20,t21,t22,t25,t26,t30,t31,t32,t33,t36,t38,t39,t42,t43,t60,t61,t62,t81,t23,t24,t27,t28,t29,t34,t35,t37,t40,t41,t44,t45,t46,t47,t51,t52,t53,t54,t55,t56,t58,t59,t69,t70,t72,t75,t76,t77,t78,t79,t80,t91,t92,t93,t94,t48,t49,t50,t57,t63,t65,t66,t67,t68,t71,t73,t82,t83,t85,t87,t88,t90,t64,t74,t84,t89,t86,t95,t96,et1,et2,et3,et4,et5,et6,et7,et8,et9,et10,mt1,mt2,mt3},{a,aN,b,beta,delta,f,psi,rf,rr,theta,varphi,varepsilon}={in2[[1]],in2[[2]],in2[[3]],in1[[5]],in1[[4]],in2[[6]],in1[[1]],in2[[8]],in2[[9]],in1[[3]],in1[[2]],in2[[4]]};*)
(**)
(*t2=Cos[beta];*)
(*t3=Cos[delta];*)
(*t4=Cos[psi];*)
(*t5=Sin[beta];*)
(*t6=Sin[delta];*)
(*t7=Cos[theta];*)
(*t8=Cos[varphi];*)
(*t9=Cos[varepsilon];*)
(*t10=Sin[psi];*)
(*t11=Sin[theta];*)
(*t12=Sin[varphi];*)
(*t13=Sin[varepsilon];*)
(*t17=-varepsilon;*)
(*t14=t9^2;*)
(*t15=t9^3;*)
(*t16=t13^2;*)
(*t18=1/t9;*)
(*t19=a t8 t11;*)
(*t20=aN t8 t11;*)
(*t21=rr t8 t9;*)
(*t22=t17+theta;*)
(*t25=aN t6 t9 t12;*)
(*t26=b t8 t9 t11;*)
(*t30=rf t3 t7 t8 t9;*)
(*t31=aN t7 t8 t9 t13;*)
(*t32=rf t5 t8 t9 t11;*)
(*t33=rf t6 t9 t12 t13;*)
(*t36=2 a t7 t8 t9 t13;*)
(*t38=rf t2 t4 t9 t11 t13;*)
(*t39=rf t4 t5 t7 t9 t13;*)
(*t42=rf t2 t9 t10 t11 t13;*)
(*t43=rf t5 t7 t9 t10 t13;*)
(*t60=rf t2 t7 t9 t10 t12 t13;*)
(*t61=rf t4 t5 t9 t11 t12 t13;*)
(*t62=rf t5 t9 t10 t11 t12 t13;*)
(*t81=rf t2 t4 t7 t9 t12 t13;*)
(*t23=Cos[t22];*)
(*t24=Sin[t22];*)
(*t27=t7 t21;*)
(*t28=-t19;*)
(*t29=-t21;*)
(*t34=t14 t20;*)
(*t35=rf t7 t8 t15;*)
(*t37=2 t14 t19;*)
(*t40=t2 t33;*)
(*t41=-t30;*)
(*t44=-t36;*)
(*t45=-t32;*)
(*t46=-t33;*)
(*t47=rf t2 t4 t7 t14;*)
(*t51=rf t2 t7 t10 t14;*)
(*t52=rf t4 t5 t11 t14;*)
(*t53=rf t5 t6 t12 t14;*)
(*t54=rf t5 t8 t11 t15;*)
(*t55=rf t5 t10 t11 t14;*)
(*t56=rf t8 t11 t13 t14;*)
(*t58=t2 t30;*)
(*t59=t3 t31;*)
(*t69=rf t2 t4 t11 t12 t14;*)
(*t70=rf t4 t5 t7 t12 t14;*)
(*t72=rf t5 t7 t8 t13 t14;*)
(*t75=rf t2 t10 t11 t12 t14;*)
(*t76=rf t5 t7 t10 t12 t14;*)
(*t77=t3 t38;*)
(*t78=t3 t39;*)
(*t79=t3 t42;*)
(*t80=t3 t43;*)
(*t91=t3 t81;*)
(*t92=t3 t60;*)
(*t93=t3 t61;*)
(*t94=t3 t62;*)
(*t48=t2 t35;*)
(*t49=t3 t34;*)
(*t50=t3 t35;*)
(*t57=-t34;*)
(*t63=t3 t47;*)
(*t65=t3 t51;*)
(*t66=t3 t52;*)
(*t67=t3 t54;*)
(*t68=t3 t55;*)
(*t71=t2 t56;*)
(*t73=t3 t56;*)
(*t82=t3 t69;*)
(*t83=t3 t70;*)
(*t85=t3 t72;*)
(*t87=t3 t75;*)
(*t88=t3 t76;*)
(*t90=-t72;*)
(*t64=t3 t48;*)
(*t74=-t49;*)
(*t84=t3 t71;*)
(*t89=-t67;*)
(*t86=-t64;*)
(*t95=-t84;*)
(*t96=t20+t25+t26+t27+t28+t29+t31+t35+t37+t40+t41+t44+t45+t46+t48+t50+t53+t54+t56+t57+t58+t59+t71+t73+t74+t85+t86+t89+t90+t95;*)
(*et1=t12 t38-t12 t39+t12 t47+t12 t52-t12 t63-t12 t66-t12 t77+t12 t78+b t7 t10-rr t4 t12-rr t10 t11+2 a t7 t9 t10+2 a t10 t11 t13-a t7 t10 t18-aN t4 t6 t8-aN t7 t9 t10-aN t10 t11 t13+aN t7 t10 t18+b t4 t11 t12-rf t5 t7 t10+rf t3 t10 t11-rf t10 t11 t14+rr t4 t7 t12-2 a t4 t7 t12 t13+2 a t4 t9 t11 t12-a t4 t11 t12 t18-aN t3 t7 t9 t10+aN t4 t7 t12 t13-aN t4 t9 t11 t12-aN t3 t10 t11 t13+aN t4 t11 t12 t18-rf t2 t3 t10 t11-rf t3 t4 t7 t12;*)
(*et2=rf t4 t6 t8 t13-rf t4 t5 t11 t12+rf t5 t7 t10 t14-rf t2 t10 t11 t14+rf t4 t7 t12 t14-rf t3 t10 t11 t14+rf t7 t9 t10 t13+aN t3 t4 t7 t12 t13-aN t3 t4 t9 t11 t12+rf t2 t3 t4 t7 t12-rf t4 t5 t6 t8-rf t5 t7 t10+rf t2 t10 t11-rf t4 t7 t12+aN t3 t4 t7 t9 t10;*)
(*et3=t96;*)
(*mt1={{et1,et2},{et2,et3}};*)
(*Return[mt1]*)
(*];*)
(**)
(*End[];*)
(**)
(*EndPackage[];*)
(**)
