function Pg = gravitationalEnergy(obj,g)
    arguments
        obj (1,1) Body;
        g (1,1) sym = sym('g');
    end
    Pg = obj.Mass*g*Frame().z.'*obj.MassCenter.posFrom();
end