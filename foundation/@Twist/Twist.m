classdef Twist
    properties (GetAccess = public, SetAccess = private)
        AngularVelocity;
        TranslationalVelocity;
        Vector;
        Matrix;
        Adjoint;
        Rate;
    end
    methods
        function obj = Twist(pose)
            arguments
                pose (1,1) Pose = Pose();
            end
            V = pose.inv().Transform*diff(pose.Transform,sym('t'));
            obj.Matrix = simplify(expand(V));
            obj.AngularVelocity = skew2vec(obj.Matrix(1:3,1:3));
            obj.TranslationalVelocity = obj.Matrix(1:3,4);
            
            obj.Vector = [
                obj.AngularVelocity;
                obj.TranslationalVelocity
                ];

            obj.Adjoint = [
                vec2skew(obj.AngularVelocity),zeros(3);
                vec2skew(obj.TranslationalVelocity),vec2skew(obj.AngularVelocity)
                ];

            obj.Rate = diff(obj.Vector,sym('t'));
        end 
    end
end