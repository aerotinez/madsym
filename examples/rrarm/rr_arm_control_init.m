close("all"); clear; clc;
setmadsympath;

g = 9.81;

Ixx1 = 1;
Iyy1 = 1;
Izz1 = 1;
m1 = 1;
l1 = 1;
b1 = 0.1;

Ixx2 = 1;
Iyy2 = 1;
Izz2 = 1;
m2 = 1;
l2 = 1;
b2 = 0.1;

params = [Iyy1,Iyy2,m1,m2,l1,l2,g].';

S1 = rscrew('y',[0,0,0]);
S2 = rscrew('y',[l1,0,0]);

function s = jointAxis(axis)
arguments
    axis (1,1) char {mustBeMember(axis,'xyzXYZ')};
end
I = eye(3);
s = I(:,lower(axis) == 'xyz');
end

function S = rscrew(axis,position,orientation)
arguments
    axis;
    position (3,1) double;
    orientation (3,3) double = eye(3);
end
w = orientation*jointAxis(axis);
S = [w.',cross(position,w).'].';
end