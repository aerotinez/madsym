function q = vertcat(varargin)
    g = @(f)cell2sym(cellfun(f,varargin.',"uniform",0));
    all = g(@(x)x.All);
    dep = g(@(x)x.Dependent);
    trim = g(@(x)x.Trim);
    trim_rate = g(@(x)x.TrimRate);
    q = GeneralizedCoordinates(all,dep,trim,trim_rate);
end