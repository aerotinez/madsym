function varargout = newparams(varargin)
varargout = cellfun(@sym,varargin,'UniformOutput',false);