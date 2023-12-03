function varargout = dynvars(varargin)
    if nargout > 1
        varargout = cellfun(@helper,varargin,'UniformOutput',false);
        return
    end
    c = class(varargin{end});
    if isequal(c,'char') || isequal(c,'string')
        varargout = {cell2sym(cellfun(@helper,varargin,'uniform',0)).'};
        return
    end
    narginchk(2,2);
    if isequal(c,'double')
        n = numel(varargin{end});
        x = varargin{1};
        xstr = arrayfun(@(i)sprintf("%s_%d",x,i),1:n).';
        varargout = {cell2sym(cellfun(@helper,xstr,'uniform',0))};
        return
    end
    error('Invalid input type');

function x = helper(xstr)
    f = symfun(sprintf('%s(t)',xstr),sym('t'));
    x = f(sym('t'));