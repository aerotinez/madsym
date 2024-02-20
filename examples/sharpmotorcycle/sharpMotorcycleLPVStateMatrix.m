function As = sharpMotorcycleLPVStateMatrix(V,d,xi,kappa)
%sharpMotorcycleLPVStateMatrix
%    As = sharpMotorcycleLPVStateMatrix(V,D,XI,KAPPA)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    20-Feb-2024 16:03:22

t2 = cos(xi);
t3 = sin(xi);
t4 = d.*kappa;
t6 = 1.0./V;
t5 = t3.^2;
t7 = 1.0./t2;
t8 = t4-1.0;
t9 = t5-1.0;
t11 = t6.*t7.*t8;
t13 = t7.*t8.*4.101722723543888;
t10 = 1.0./t9;
t12 = -t11;
mt1 = [-kappa.*t3.*t7,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t8.*t10,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t11.*2.617965739525643e+1,t11.*(-8.274201506546905e-1),t11.*(-4.280086086874913e+1),t11.*(-2.618743416629438e+2),t7.*t8.*(-5.437243642329779e+3),t7.*t8.*(-3.849876948318294e+3)];
mt2 = [0.0,0.0,0.0,0.0,t11.*1.862458313558448,t11.*2.443173842432228,t11.*(-2.949938861242202),t11.*(-1.274547589021009e+2),0.0,t7.*t8.*(-4.258045877029179e+4),t6.*t8.*t10,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
mt3 = [t11.*6.49343724364233e+4,t11.*4.583264971287941e+4,0.0,t12,0.0,0.0,t7.*t8.*8.992500375948788e-1];
mt4 = [t7.*t8.*(-2.819993331366436e-2)];
mt5 = [t7.*t8.*1.635237866339509e-1];
mt6 = [t7.*t8.*2.372835273453382,t11.*(-3.11555118949959e+4),t11.*4.28410446747656e+4,0.0,0.0,t12,0.0];
mt7 = [t7.*t8.*1.55324628510465e-2,t7.*t8.*(-3.305357146121931e-2)];
mt8 = [t7.*t8.*2.831101360300849e-3];
mt9 = [t7.*t8.*(-4.516425686272255),0.0,0.0,0.0,0.0,0.0,t12];
mt10 = [t11.*(V.*4.429645638026684e+34-7.630146612256428e+34).*(-7.582401355855259e-37)];
mt11 = [t11.*(V.*9.681789031617823e+49+2.218847653355391e+51).*(-1.683631313443903e-52)];
mt12 = [(t11.*(V.*1.063545895749968e+51-1.936839040819943e+51))./1.781862796233837e+52];
mt13 = [(t11.*(V.*1.506792594720213e+50+9.259775083751082e+52))./5.939542654112789e+51,0.0,t11.*(-5.307420836751436e+3),0.0,0.0,0.0];
mt14 = [0.0,t11.*(-1.858987986555566e-2),t11.*1.807876120989101e-2,t11.*2.148507356330391e-2,t11.*(-3.502138054062041e-2),t13,0.0,0.0,0.0,0.0,0.0,t11.*(-9.441822717445154e-3)];
mt15 = [t11.*(-2.46063705436728e-2),t11.*1.20984229225678e-2,t11.*3.096987957057304e-1,0.0,t13];
As = reshape([mt1,mt2,mt3,mt4,mt5,mt6,mt7,mt8,mt9,mt10,mt11,mt12,mt13,mt14,mt15],10,10);
