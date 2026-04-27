function matlabFunction(obj,name,x,parameters)
    arguments
        obj (1,1) DynamicEquations
        name (1,1) string
        x (:,1) DynamicVariable = obj.States
        parameters (:,1) = sym.empty(0,1)
    end

    eom = obj;

    x = prettify(state(x));
    u = prettify(eom.States.state);

    G  = prettify(eom.SpatialInertia);
    J  = prettify(eom.Jacobian);
    dJ = prettify(eom.JacobianRate);
    W  = prettify(eom.ActiveForces);
    Jc = prettify(eom.constrainingJacobian());

    if isempty(parameters)
        parameters = setdiff( ...
            symvar([G(:).',J(:).',dJ(:).',W(:).',Jc(:).']), ...
            [x.',u.'] ...
            ).';
    end

    dir_name = name + "Dynamics";
    if ~exist(dir_name,"dir")
        mkdir(dir_name)
    end

    state_str = "   states = [" + strjoin(string(x).') + "]";
    input_str = "   inputs = [" + strjoin(string(u).') + "]";
    param_str = "   params = [" + strjoin(string(parameters).') + "]";
    comment_str = [state_str,input_str,param_str,""].';

    Gstr  = dir_name + "/" + name + "DynamicsSpatialInertia";
    Jstr  = dir_name + "/" + name + "DynamicsJacobian";
    dJstr = dir_name + "/" + name + "DynamicsJacobianRate";
    Wstr  = dir_name + "/" + name + "DynamicsActiveForces";
    Jcstr = dir_name + "/" + name + "DynamicsConstrainingJacobian";

    matlabFunction(G, ...
        "File",Gstr, ...
        "Vars",{x,u,parameters}, ...
        "Comments",comment_str);

    matlabFunction(J, ...
        "File",Jstr, ...
        "Vars",{x,u,parameters}, ...
        "Comments",comment_str);

    matlabFunction(dJ, ...
        "File",dJstr, ...
        "Vars",{x,u,parameters}, ...
        "Comments",comment_str);

    matlabFunction(W, ...
        "File",Wstr, ...
        "Vars",{x,u,parameters}, ...
        "Comments",comment_str);

    matlabFunction(Jc, ...
        "File",Jcstr, ...
        "Vars",{x,u,parameters}, ...
        "Comments",comment_str);

    fid = fopen(dir_name + "/" + name + "DynamicsMassMatrix.m","w");
    fprintf(fid,"function M = %sDynamicsMassMatrix(x,u,p)\n",name);
    fprintf(fid,"   %% %s\n",state_str);
    fprintf(fid,"   %% %s\n",input_str);
    fprintf(fid,"   %% %s\n",param_str);
    fprintf(fid,"   \n");
    fprintf(fid,"   G  = %sDynamicsSpatialInertia(x,u,p);\n",name);
    fprintf(fid,"   J  = %sDynamicsJacobian(x,u,p);\n",name);
    fprintf(fid,"   Jc = %sDynamicsConstrainingJacobian(x,u,p);\n",name);
    fprintf(fid,"   \n");
    fprintf(fid,"   M = J.'*G*J;\n");
    fprintf(fid,"   \n");
    fprintf(fid,"   n = size(M,1);\n");
    fprintf(fid,"   if ~isequal(Jc,eye(n))\n");
    fprintf(fid,"      m = size(Jc,1);\n");
    fprintf(fid,"      k = n - m;\n");
    fprintf(fid,"      M = M(1:k,:) + Jc.'*M(k+1:end,:);\n");
    fprintf(fid,"   end\n");
    fprintf(fid,"   \n");
    fprintf(fid,"end\n");
    fclose(fid);

    fid = fopen(dir_name + "/" + name + "DynamicsForcingVector.m","w");
    fprintf(fid,"function f = %sDynamicsForcingVector(x,u,p)\n",name);
    fprintf(fid,"   %% %s\n",state_str);
    fprintf(fid,"   %% %s\n",input_str);
    fprintf(fid,"   %% %s\n",param_str);
    fprintf(fid,"   \n");
    fprintf(fid,"   G  = %sDynamicsSpatialInertia(x,u,p);\n",name);
    fprintf(fid,"   J  = %sDynamicsJacobian(x,u,p);\n",name);
    fprintf(fid,"   dJ = %sDynamicsJacobianRate(x,u,p);\n",name);
    fprintf(fid,"   W  = %sDynamicsActiveForces(x,u,p);\n",name);
    fprintf(fid,"   Jc = %sDynamicsConstrainingJacobian(x,u,p);\n",name);
    fprintf(fid,"   \n");
    fprintf(fid,"   V = J*u;\n");
    fprintf(fid,"   nb = size(G,1)/6;\n");
    fprintf(fid,"   adw = zeros(6*nb,6*nb);\n");
    fprintf(fid,"   \n");
    fprintf(fid,"   for i = 1:nb\n");
    fprintf(fid,"      idx = (6*(i-1)+1):(6*i);\n");
    fprintf(fid,"      w = V(idx(1:3));\n");
    fprintf(fid,"      wx = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];\n");
    fprintf(fid,"      adw(idx,idx) = blkdiag(wx,wx);\n");
    fprintf(fid,"   end\n");
    fprintf(fid,"   \n");
    fprintf(fid,"   f = J.'*(W - (G*dJ + adw*G*J)*u);\n");
    fprintf(fid,"   \n");
    fprintf(fid,"   n = numel(u);\n");
    fprintf(fid,"   if ~isequal(Jc,eye(n))\n");
    fprintf(fid,"      m = size(Jc,1);\n");
    fprintf(fid,"      k = n - m;\n");
    fprintf(fid,"      f = f(1:k) + Jc.'*f(k+1:end);\n");
    fprintf(fid,"   end\n");
    fprintf(fid,"   \n");
    fprintf(fid,"end\n");
    fclose(fid);
end