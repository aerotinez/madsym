function [pexpr,vars] = prettytrig(expr)
    arguments
        expr sym;
    end
    x = [findSymType(expr,"symfun"),symvar(expr)];
    xc = unique(reshape(x + x.',1,[]));
    % x = [x,xc];
    x(x == sym('t')) = [];
    sx = arrayfun(@sin,x);
    cx = arrayfun(@cos,x);
    tx = arrayfun(@tan,x);
    fx = @(f,x)str2sym(f + "_" + string(x));
    sx0 = arrayfun(@(x)fx('s',x),prettify(x));
    cx0 = arrayfun(@(x)fx('c',x),prettify(x));
    tx0 = arrayfun(@(x)fx('t',x),prettify(x));
    ovars = [sx,cx,tx];
    vars = [sx0,cx0,tx0];
    pexpr = subs(expr,ovars,vars);
end