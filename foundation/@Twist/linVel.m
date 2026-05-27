function pd = linVel(obj)
    arguments
        obj (1,1) Twist;
    end
    T = obj.Pose.transform(); 
    Vm = obj.matrix();
    Td = T*Vm;
    pd = Td(1:3,4);
end