function varargout = stateSpace(eoml)
varargout{1} = simplify(expand(eoml.P.'*(eoml.M\eoml.F)));
if ~isempty(eoml.G)
    varargout{2} = simplify(expand(eoml.P.'*(eoml.M\eoml.G)));
else
    varargout{2} = [];
end