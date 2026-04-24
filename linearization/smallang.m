function f0 = smallang(f, angs)
    arguments
        f sym
        angs (:,1) sym
    end

    x = angs;

    % f0 = rewrite(f, 'sincos');
    % f0 = expand(f0);

    % First-order trig substitutions
    f0 = f;
    f0 = subs(f0, cos(x), sym(ones(size(x))));
    f0 = subs(f0, sin(x), x);

    % % Expand again so newly created products are visible
    % f0 = expand(f0);
    % 
    % % Remove all quadratic and higher terms in the chosen small angles
    % for i = 1:numel(x)
    %     f0 = subs(f0, x(i)^2, sym(0));
    %     for j = i+1:numel(x)
    %         f0 = subs(f0, x(i)*x(j), sym(0));
    %         f0 = subs(f0, x(j)*x(i), sym(0)); % just in case ordering differs
    %     end
    % end
    % 
    % f0 = simplify(f0, 20);
end