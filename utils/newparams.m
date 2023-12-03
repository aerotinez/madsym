function varargout = newparams(varargin)
varargout = cellfun(@str2sym,varargin,'UniformOutput',false);