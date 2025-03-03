function setColor(obj,rear_tire,rear_chassis,front_chassis,front_tire)
    arguments
        obj (1,1) MotoFigure;
        rear_tire (1,3) double;
        rear_chassis (1,3) double;
        front_chassis (1,3) double;
        front_tire (1,3) double;
    end
    obj.RearTire.Patch.FaceColor = rear_tire;
    obj.SwingArm.Patch.FaceColor = rear_chassis;
    obj.Base.Patch.FaceColor = rear_chassis;
    obj.Body.Patch.FaceColor = rear_chassis;
    obj.Handlebars.Patch.FaceColor = front_chassis;
    obj.ForkTop.Patch.FaceColor = front_chassis;
    obj.FrontFender.Patch.FaceColor = front_chassis;
    obj.Fork.Patch.FaceColor = front_chassis;
    obj.FrontTire.Patch.FaceColor = front_tire;
    obj.LeftLeg.Patch.FaceColor = rear_chassis;
    obj.RightLeg.Patch.FaceColor = rear_chassis;
    obj.Driver.Patch.FaceColor = rear_chassis;
    obj.LeftArm.Patch.FaceColor = rear_chassis;
    obj.RightArm.Patch.FaceColor = rear_chassis;
end