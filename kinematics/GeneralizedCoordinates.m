classdef GeneralizedCoordinates
properties (GetAccess = public, SetAccess = private)
    All (:,1) JointVariable = JointVariable.empty([0,1]); 
    Independent (:,1) JointVariable = JointVariable.empty([0,1]);
    Dependent (:,1) JointVariable = JointVariable.empty([0,1]);
end
methods (Access = public)
function obj = GeneralizedCoordinates(independent,dependent)
    arguments
        independent (:,1) JointVariable = JointVariable.empty([0,1]);
        dependent (:,1) JointVariable = JointVariable.empty([0,1]);
    end
    obj.Independent = independent;
    obj.Dependent = dependent;
    obj.All = [
        independent;
        dependent
        ];
end
end
end