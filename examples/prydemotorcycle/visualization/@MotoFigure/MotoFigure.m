classdef MotoFigure < handle
    properties
        RearTire;
        SwingArm;
        Base;
        Body;
        FrontTire;
        Fork;
        FrontFender;
        ForkTop;
        Handlebars;
        LeftLeg;
        RightLeg;
        Driver;
        LeftArm;
        RightArm;
    end
    methods (Access = public)
        function setPose(obj,q)
            setPose(obj.RearTire,q);
            setPose(obj.SwingArm,q);
            setPose(obj.Base,q);
            setPose(obj.Body,q);
            setPose(obj.FrontTire,q);
            setPose(obj.Fork,q);
            setPose(obj.FrontFender,q);
            setPose(obj.ForkTop,q);
            setPose(obj.Handlebars,q);
            setPose(obj.LeftLeg,q);
            setPose(obj.RightLeg,q);
            setPose(obj.Driver,q);
            setPose(obj.LeftArm,q);
            setPose(obj.RightArm,q);
        end
    end
end