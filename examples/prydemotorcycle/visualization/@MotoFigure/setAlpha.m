function setAlpha(obj,rear_tire,rear_chassis,front_chassis,front_tire)
    arguments
        obj (1,1) MotoFigure;
        rear_tire (1,1) double;
        rear_chassis (1,1) double;
        front_chassis (1,1) double;
        front_tire (1,1) double;
    end
    obj.RearTire.Patch.FaceAlpha = rear_tire;
    obj.SwingArm.Patch.FaceAlpha = rear_chassis;
    obj.Base.Patch.FaceAlpha = rear_chassis;
    obj.Body.Patch.FaceAlpha = rear_chassis;
    obj.Handlebars.Patch.FaceAlpha = front_chassis;
    obj.ForkTop.Patch.FaceAlpha = front_chassis;
    obj.FrontFender.Patch.FaceAlpha = front_chassis;
    obj.Fork.Patch.FaceAlpha = front_chassis;
    obj.FrontTire.Patch.FaceAlpha = front_tire;
    obj.LeftLeg.Patch.FaceAlpha = rear_chassis;
    obj.RightLeg.Patch.FaceAlpha = rear_chassis;
    obj.Driver.Patch.FaceAlpha = rear_chassis;
    obj.LeftArm.Patch.FaceAlpha = rear_chassis;
    obj.RightArm.Patch.FaceAlpha = rear_chassis;
end