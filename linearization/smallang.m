function f0 = smallang(f, angs)
    arguments
        f sym
        angs (:,1) sym
    end

    x = angs;

    f0 = rewrite(f, 'sincos');
    f0 = expand(f0);

    % First-order trig substitutions
    f0 = subs(f0, sin(x), x);
    f0 = subs(f0, cos(x), sym(ones(size(x))));

    % Expand again so newly created products are visible
    f0 = expand(f0);

    % Remove all quadratic and higher terms in the chosen small angles
    for i = 1:numel(x)
        for j = i:numel(x)
            f0 = subs(f0, x(i)*x(j), sym(0));
        end
    end

    f0 = simplify(f0, 20);
end