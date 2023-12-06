classdef Linearizer < handle
properties
    eom;
    trim_point;
end
properties
    q;
    u;
    qd;
    ud;
    fk1;
    fk2;
    fd1;
    fd2;
end
properties
    Mkqd;
    Mdud;
    Fkq;
    Fku;
    Fdq;
    Fdu;
    Gf; 
end
properties
    n;
    k;
    l;
    p;
end
properties
    qind;
    qdep;
    Pind;
    Pdep;
    C;
end
properties
    M;
    F;
    G;
    P;
end
methods
function obj = Linearizer(eom,trim_point,options)
    arguments
        eom (1,1) EquationsOfMotion;
        trim_point (1,1) TrimPoint;
        options.DependentCoordinates (:,1) sym = sym.empty;
    end
    obj.eom = eom;
    obj.trim_point = trim_point;
    obj.q = eom.eomk.q;
    obj.u = [
        eom.eomk.u;
        eom.AuxiliarySpeeds;
    ];
    obj.qd = diff(obj.q);
    obj.ud = diff(obj.u);
    obj.qdep = options.DependentCoordinates;
    obj.qind = obj.q(~has(obj.q,obj.qdep));
    obj.n = numel(obj.q);
    obj.k = numel(obj.u);
    obj.l = numel(eom.eomk.l);
    obj.p = numel(eom.AuxiliarySpeeds);
    obj.equations();
    obj.massMatrixJacobians();
    obj.forcingMatrixJacobians();
    obj.inputMatrixJacobian();
    obj.permutationMatrices();
    obj.projectionMatrix();
    obj.massMatrix();
    obj.forcingMatrix();
    obj.inputMatrix();
end
end
methods (Access = public)
function out = trimsub(obj,expr)
    qs = obj.trim_point.q;
    us = [
        obj.trim_point.u;
        obj.trim_point.u_aux;
    ];
    q0 = obj.trim_point.q0;
    u0 = [
        obj.trim_point.u0;
        obj.trim_point.u0_aux;
    ];
    uds = obj.ud;
    ud0 = 0.*uds;
    x = [
        uds;
        us;
        qs;
        obj.trim_point.F
    ];
    x0 = [
        ud0;
        u0;
        q0;
        obj.trim_point.F0
    ];
    out = simplify(expand(subs(expr,x,x0)));
end
function equations(obj)
    obj.fk1 = obj.qd;
    obj.fk2 = -obj.eom.eomk.Ju(:,1:(obj.k - obj.p))*obj.eom.eomk.u;
    obj.fd1 = obj.eom.MassMatrix*obj.ud;
    obj.fd2 = -obj.eom.ForcingVector;
end
function massMatrixJacobians(obj)
    obj.Mkqd = jacobian(obj.fk1, obj.qd);
    obj.Mdud = jacobian(obj.fd1, obj.ud);
end
function forcingMatrixJacobians(obj)
    obj.Fkq = jacobian(-obj.fk2, obj.q);
    obj.Fku = jacobian(-obj.fk2, obj.u);
    obj.Fdq = jacobian(-(obj.fd1 + obj.fd2), obj.q);
    obj.Fdu = jacobian(-obj.fd2, obj.u);
end
function inputMatrixJacobian(obj)
    obj.Gf = jacobian(-obj.fd2,obj.eom.Inputs);
end
function permutationMatrices(obj)
    fidx = @(xi)has(obj.q,xi).';
    fP = @(x)double(cell2mat(arrayfun(fidx,x,'uniform',0)));
    P0 = fP([obj.qind;obj.qdep])\eye(obj.n);
    obj.Pind = P0*[
        eye(obj.n - obj.l);
        zeros(obj.l,obj.n - obj.l);
    ];
    obj.Pdep = P0*[
        zeros(obj.n - obj.l,obj.l);
        eye(obj.l);
    ];
    obj.P = [
        obj.Pind, zeros(obj.n,obj.k);
        zeros(obj.k,obj.n - obj.l), eye(obj.k);
    ];
end
function projectionMatrix(obj)
    Jhc = jacobian(obj.eom.eomk.hc,obj.q);
    obj.C = (eye(obj.n) - obj.Pdep*((Jhc*obj.Pdep)\Jhc))*obj.Pind;
    obj.C = simplify(expand(obj.C));
end
function massMatrix(obj)
    obj.M = obj.trimsub(blkdiag(obj.Mkqd,obj.Mdud));
end
function forcingMatrix(obj)
    Fq = [obj.Fkq;obj.Fdq]*obj.C;
    Fu = [obj.Fku;obj.Fdu];
    obj.F = obj.trimsub([Fq,Fu]); 
end
function inputMatrix(obj)
    obj.G = obj.trimsub([zeros(obj.n,size(obj.Gf,2));obj.Gf]);
end
end
end