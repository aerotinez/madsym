classdef (Abstract) element < handle
    properties (SetAccess = protected)
        Axes
        Handle
        Orientation = eye(3)
        Position = [0,0,0]
        Alpha = 1
        Color = [0,0,0]
    end

    methods
        function obj = element(varargin)
            if mod(nargin, 2) ~= 0
                error("anim:element:BadNameValue", ...
                    "Name-value inputs must come in pairs.");
            end

            opts = struct();
            for k = 1:2:numel(varargin)
                name = varargin{k};
                value = varargin{k+1};

                if isstring(name)
                    name = char(name);
                end

                if ~ischar(name) || isempty(name)
                    error("anim:element:BadNameValue", ...
                        "Name must be a char or string scalar.");
                end

                opts.(name) = value;
            end

            if isfield(opts, "Parent") && ~isempty(opts.Parent)
                ax = opts.Parent;
            else
                ax = gca;
            end

            obj.Axes = ax;

            wasHold = ishold(ax);
            hold(ax, "on");

            args = [fields(opts), struct2cell(opts)].';
            args = args(:);

            init(obj,args{:});

            if ~wasHold
                hold(ax, "off");
            end

            if isfield(opts, "Color") && ~isempty(opts.Color)
                setColor(obj,opts.Color);
            else
                setColor(obj,[0,0,0]);
            end
        end

        R = getOrientation(obj);
        p = getPosition(obj);
        setPose(obj,opts);
        setAlpha(obj,alpha);
        setColor(obj,c);
    end

    methods (Access = protected)
        init(obj, opts)
        setOrientation(obj,R);
        setPosition(obj,p);
    end
end
