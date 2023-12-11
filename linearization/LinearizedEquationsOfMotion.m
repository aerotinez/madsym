classdef LinearizedEquationsOfMotion < handle
properties (Access = private)
    q (:,1) sym = sym.empty([0,1]); % generalized coordinates
    qd (:,1) sym = sym.empty([0,1]); % generalized velocities
    u (:,1) sym = sym.empty([0,1]); % quasi-velocities
    ud (:,1) sym = sym.empty([0,1]); % quasi-accelerations
    v (:,1) sym = sym.empty([0,1]); % auxiliary velocities
    vd (:,1) sym = sym.empty([0,1]); % auxiliary accelerations
    f (:,1) sym = sym.empty([0,1]); % input variables
end
properties (Access = private)
    n (1,1) double = 0; % number of generalized coordinates
    l (1,1) double = 0; % number of dependent coordinates
    m (1,1) double = 0; % number of nonholonomic constraints
    k (1,1) double = 0; % number of quasi-velocities
    p (1,1) double = 0; % number of auxiliary variables
end
properties (Access = private)
    qind (:,1) sym = sym.empty([0,1]); % independent coordinates
    qdep (:,1) sym = sym.empty([0,1]); % dependent coordinates
end
properties (Access = private)
    Pind; % independent coordinate permutation matrix
    Pdep; % dependent coordinate permutation matrix
end
properties (Access = private)
    hc; % holonomic constraints
    Jhc; % holonomic constraints Jacobian
    C; % projection matrix
end
properties (Access = private)
    fk0;
    fk1;
    fd0;
    fd1;
    fv0;
    fv1;
end
properties (Access = private)
    Mkqd;
    Mdud;
    Mavd;
end
properties (Access = private)
    Hkq;
    Hku;
    Hkv;
    Hdq;
    Hdu;
    Hdv;
    Haq;
    Hau;
    Hav;
end
properties (Access = private)
    Gkf;
    Gdf;
    Gaf;
end
properties (GetAccess = public, SetAccess = private)
    Trim TrimPoint {mustBeScalarOrEmpty} = TrimPoint.empty([0,1]);
    PermutationMatrix;
    MassMatrix;
    ForcingMatrix;
    InputMatrix;
end
methods (Access = public)
function obj = LinearizedEquationsOfMotion(equations,trim_point)
    arguments
        equations (1,1) EquationsOfMotion = EquationsOfMotion();
        trim_point (1,1) TrimPoint = TrimPoint(); 
    end
    obj.Trim = trim_point;
    obj.q = [equations.Kinematics.Variable.All.Position].';
    obj.qd = [equations.Kinematics.Variable.All.Velocity].';
    obj.qind = [equations.Kinematics.Variable.Independent.Position].';
    obj.qdep = [equations.Kinematics.Variable.Dependent.Position].';
    obj.u = [equations.Dynamics.Variable.Velocity].';
    obj.ud = [equations.Dynamics.Variable.Acceleration].';
    obj.v = [equations.Auxiliary.Variable.Velocity].';
    obj.vd = [equations.Auxiliary.Variable.Acceleration].';
    obj.f = trim_point.f; 
    obj.hc = equations.Kinematics.HolonomicConstraints;
    obj.Jhc = jacobian(obj.hc,obj.q);
    obj.dimensions();
    obj.permutationMatrices();
    obj.projectionMatrix();
    obj.equationsOfMotion(equations);
    obj.massMatrixJacobians();
    obj.massMatrix();
    obj.forcingMatrixJacobians();
    obj.forcingMatrix();
    obj.inputMatrixJacobians();
    obj.inputMatrix(); 
end
end
methods (Access = private)
function dimensions(obj)
    obj.n = numel(obj.q);
    obj.k = numel(obj.u);
    obj.l = numel(obj.qdep);
    obj.m = obj.n - obj.k;
    obj.p = numel(obj.v);
end
function equationsOfMotion(obj,equations)
    obj.fk0 = equations.Kinematics.MassMatrix*obj.qd;
    obj.fk1 = -equations.Kinematics.ForcingVector;
    obj.fd0 = equations.Dynamics.MassMatrix*obj.ud;
    obj.fd1 = -equations.Dynamics.ForcingVector;
    obj.fv0 = equations.Auxiliary.MassMatrix*obj.vd;
    obj.fv1 = -equations.Auxiliary.ForcingVector;
end
function expr_sub = trimSub(obj,expr)
    x = [
        obj.Trim.ud;
        obj.Trim.vd;
        obj.Trim.u;
        obj.Trim.v;
        obj.Trim.q;
        obj.Trim.f;
    ];

    xs = [
        obj.Trim.ud0;
        obj.Trim.vd0;
        obj.Trim.u0;
        obj.Trim.v0;
        obj.Trim.q0;
        obj.Trim.f0;
    ];

    expr_sub = simplify(expand(subs(expr,x,xs)));
end
function massMatrixJacobians(obj)
    obj.Mkqd = eye(obj.n,'sym'); 
    obj.Mdud = jacobian(obj.fd0,obj.ud); 
    obj.Mavd = jacobian(obj.fv0,obj.vd);
end
function massMatrix(obj)
    M = blkdiag(obj.Mkqd,obj.Mdud,obj.Mavd);
    obj.MassMatrix = obj.trimSub(M);
end
function forcingMatrixJacobians(obj)
    obj.Hkq = -jacobian(obj.fk1,obj.q);
    obj.Hku = -jacobian(obj.fk1,obj.u);
    obj.Hkv = zeros(obj.n,obj.p,'sym');
    obj.Hdq = -jacobian(obj.fd0 + obj.fd1,obj.q);
    obj.Hdu = -jacobian(obj.fd1,obj.u);
    obj.Hdv = -jacobian(obj.fd1,obj.v);
    obj.Haq = -jacobian(obj.fv0 + obj.fv1,obj.q);
    obj.Hau = -jacobian(obj.fv1,obj.u);
    obj.Hav = -jacobian(obj.fv1,obj.v);
end
function forcingMatrix(obj)
    Hq = [
        obj.Hkq;
        obj.Hdq;
        obj.Haq;
    ];

    Hu = [
        obj.Hku;
        obj.Hdu;
        obj.Hau;
    ];

    Hv = [
        obj.Hkv;
        obj.Hdv;
        obj.Hav;
    ];

    H = [
        Hq*obj.C,Hu,Hv 
    ];

    obj.ForcingMatrix = obj.trimSub(H);
end
function inputMatrixJacobians(obj)
    obj.Gkf = -jacobian(obj.fk0 + obj.fk1,obj.f);
    obj.Gdf = -jacobian(obj.fd0 + obj.fd1,obj.f);
    obj.Gaf = -jacobian(obj.fv0 + obj.fv1,obj.f);
end
function inputMatrix(obj)
    G = [
        obj.Gkf;
        obj.Gdf;
        obj.Gaf;
    ];

    obj.InputMatrix = obj.trimSub(G);
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

    obj.PermutationMatrix = [
        obj.Pind, zeros(obj.n,obj.k + obj.p);
        zeros(obj.k + obj.p,obj.n - obj.l), eye(obj.k + obj.p);
    ];
end
function projectionMatrix(obj)
    obj.C = (eye(obj.n) - obj.Pdep*((obj.Jhc*obj.Pdep)\obj.Jhc))*obj.Pind;
    obj.C = simplify(expand(obj.C));
end
end
end