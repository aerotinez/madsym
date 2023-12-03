classdef PrettyEquations
properties (GetAccess = public, SetAccess = private)
    Equations sym = sym.empty;
    q sym = sym.empty;
    v sym = sym.empty;
    a sym = sym.empty;
    j sym = sym.empty;
    s sym = sym.empty;
    params sym = sym.empty;
end
properties (Access = private)
    eq_in sym;
    q_in (:,1) sym;
    v_in (:,1) sym;
    a_in (:,1) sym;
    j_in (:,1) sym;
    s_in (:,1) sym;
end
methods (Access = public)
function obj = PrettyEquations(eqs)
    obj.eq_in = eqs;
    obj.params = obj.extractParams();
    obj.q_in = obj.extractVars();
    obj.v_in = diff(obj.q_in);
    obj.a_in = diff(obj.v_in);
    obj.j_in = diff(obj.a_in);
    obj.s_in = diff(obj.j_in);
    obj.q = obj.dynvarToSym();
    obj.v = arrayfun(@(x)obj.diffToDot(x,'dot'),obj.q);
    obj.a = arrayfun(@(x)obj.diffToDot(x,'ddot'),obj.q);
    obj.j = arrayfun(@(x)obj.diffToDot(x,'tdot'),obj.q);
    obj.s = arrayfun(@(x)obj.diffToDot(x,'qddot'),obj.q);

    ovars = [
        obj.s_in;
        obj.j_in;
        obj.a_in;
        obj.v_in;
        obj.q_in;
        ];

    nvars = [
        obj.s;
        obj.j;
        obj.a;
        obj.v;
        obj.q;
        ];

    obj.Equations = subs(obj.eq_in,ovars,nvars);
end
end
methods (Access = private)
function p = extractParams(obj)
    p = symvar(obj.eq_in).';
    if sum(has(p,sym('t')))
        p(has(p,sym('t'))) = [];
    end
end
function q = extractVars(obj)
    q = findSymType(obj.eq_in,'symfunOf',sym('t')).';
end
function qs = dynvarToSym(obj)
    qs = arrayfun(@(x)str2sym(strrep(string(x),'(t)','')),obj.q_in);
end
function xd = diffToDot(~,x,dstr)
    xs = char(x);
    inds = strfind(xs,'_');
    if isempty(inds)
        xd = str2sym(sprintf('%s_%s',xs,dstr));
        return
    end
    xd = str2sym(sprintf('%s_%s%s',xs(1:inds(1)-1),dstr,xs(inds(1):end)));
end
end
end