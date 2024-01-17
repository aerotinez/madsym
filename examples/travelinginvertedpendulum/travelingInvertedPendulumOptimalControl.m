function out1 = travelingInvertedPendulumOptimalControl(in1,g)
%travelingInvertedPendulumOptimalControl
%    OUT1 = travelingInvertedPendulumOptimalControl(IN1,G)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    17-Jan-2024 23:53:39

lambda_5 = in1(12,:);
lambda_6 = in1(13,:);
varphi = in1(4,:);
t2 = sin(varphi);
out1 = [-lambda_5+lambda_6.*t2-lambda_5.*t2.^2;-lambda_6+lambda_5.*t2];
