function out = derive(f,eomk,bodies)
arguments
    f (1,1) function_handle;
    eomk KinematicEquations;
    bodies Body;
end
out = EquationsOfMotion(eomk,bodies).derive(f);