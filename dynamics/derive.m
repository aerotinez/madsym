function out = derive(f,eomk,bodies,options)
arguments
    f (1,1) function_handle;
    eomk KinematicEquations;
    bodies Body;
    options.Inputs sym = sym.empty;
    options.FrictionCoeffs sym = zeros([numel(eomk.q),1],'sym');
    options.AuxiliarySpeeds (:,1) sym = sym.empty;
    options.AuxiliaryEquations (:,1) sym = sym.empty;
end
out = EquationsOfMotion(eomk,bodies, ...
'Inputs',options.Inputs, ....
'FrictionCoeffs',options.FrictionCoeffs, ...
'AuxiliarySpeeds',options.AuxiliarySpeeds, ...
'AuxiliaryEquations',options.AuxiliaryEquations).derive(f);