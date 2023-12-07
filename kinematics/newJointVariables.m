function varargout = newJointVariables(varargin)
if nargin > 1
    x = cell2sym(varargin);
else
    x = varargin{1};
end
varargout = arrayfun(@JointVariable,x,'UniformOutput',false);