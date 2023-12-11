function varargout = newDynamicVariables(varargin)
f = @DynamicVariable;
if nargout > 1
    varargout = cellfun(f,varargin,'UniformOutput',false);
    return
end
c = class(varargin{end});
if isequal(c,'char') || isequal(c,'string')
    outc = cellfun(f,varargin,'UniformOutput',false);
    varargout{1} = cellfun(@(x)x,outc);
    return
end
narginchk(2,2);
if isequal(c,'double')
    n = numel(varargin{end});
    x = varargin{1};
    xstr = arrayfun(@(i)sprintf("%s_%d",x,i),1:n).';
    outc = cellfun(f,xstr,'UniformOutput',false);
    varargout{1} = cellfun(@(x)x,outc);
    return
end
error('Invalid input type');