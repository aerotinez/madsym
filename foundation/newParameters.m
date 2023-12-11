function varargout = newParameters(varargin)
varargout = cellfun(@sym,varargin,'UniformOutput',false);