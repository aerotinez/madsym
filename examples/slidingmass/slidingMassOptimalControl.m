function fun = slidingMassOptimalControl(in1,m)
%slidingMassOptimalControl
%    FUN = slidingMassOptimalControl(IN1,M)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    18-Jan-2024 13:27:35

lambda_2 = in1(4,:);
fun = -lambda_2./m;