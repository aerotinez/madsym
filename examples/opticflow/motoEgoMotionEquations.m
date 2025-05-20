function [A,b] = motoEgoMotionEquations(f,in2,in3,in4,in5,in6)
    A = motoEgoMotionMassMatrix(f,in2,in3,in4,in5,in6);
    b = in2 + motoEgoMotionAngVelJac(f,in2,in3,in4,in5,in6)*in5;
end