classdef MotoBody < handle
    properties (GetAccess = public, SetAccess = protected)
        Vertices;
        Patch;
        AnimatorParameters;
        GeometricParameters;
    end
    methods (Access = public)
        function obj = MotoBody(stl_file,anim_params,geom_params)
            arguments
                stl_file (1,1) string;
                anim_params (1,1) struct;
                geom_params (1,1) struct;
            end
            tr = stlread(stl_file + ".stl");
            hold on;
            obj.Patch = trisurf(tr);
            obj.Vertices = obj.Patch.Vertices;
            obj.Patch.EdgeColor = 'none';
            obj.Patch.FaceColor = 0.5.*ones(1,3);
            hold off;
            obj.AnimatorParameters = anim_params;
            obj.GeometricParameters = geom_params;
            obj.centerGeometry();
        end
    end
end