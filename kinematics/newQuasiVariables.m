function varargout = newQuasiVariables(varargin)
if nargin > 1
    x = cell2sym(varargin);
else
    x = varargin{1};
end
y = arrayfun(@QuasiVariable,x);
if nargout == 1
    varargout{1} = y;
else
    varargout = sym2cell(y);
end