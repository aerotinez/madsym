function matlabFunction(obj,name,parameters)
    arguments
        obj (1,1) MechanicsEquations;
        name (1,1) string;
        parameters (:,1) = sym.empty(0,1);
    end

    x = obj.States;

    dir_name = name + "DAE";
    if ~exist(dir_name, 'dir')
        mkdir(dir_name)
    end

    matlabFunction(obj.Kinematics,name,parameters);
    movefile(name + "Kinematics",dir_name);

    matlabFunction(obj.Constraints,name,parameters);
    movefile(name + "Constraints",dir_name);

    for k = 1:numel(obj.BodyDynamics)
        matlabFunction(obj.BodyDynamics(k),name + "Body" + k,x,parameters);
        movefile(name + "Body" + k + "Dynamics",dir_name);
    end

    nq = numel(obj.Kinematics.States);
    nu = numel(obj.Kinematics.Inputs);
    nui = numel(independent(obj.Kinematics.Inputs));
    nud = numel(dependent(obj.Kinematics.Inputs));
    nb = numel(obj.BodyDynamics);

    fid = fopen(dir_name + "/" + dir_name + "MassMatrix.m","w");
    fprintf(fid,"function M = %sDAEMassMatrix(x,p)\n",name);
    fprintf(fid,"    Mk = " + name + "KinematicMassMatrix();\n");
    for k = 1:nb
        fprintf(fid,"    Md" + k + " = " + name + "Body" + k + "DynamicsMassMatrix(x,p);\n");
    end
    fprintf(fid,"    Md = Md1");
    if nb > 1
        for k = 2:nb
            fprintf(fid," + Md" + k);
        end
    end
    fprintf(fid,";\n");
    fprintf(fid,"    Mc = " + name + "ConstraintMassMatrix(x,p);\n");
    fprintf(fid,"    M = [\n");
    fprintf(fid,"        Mk,zeros(" + nq + "," + nu + ");\n");
    fprintf(fid,"        zeros(" + nui + "," + nq + "),Md;\n");
    fprintf(fid,"        zeros(" + nud + "," + nq + "),Mc\n");
    fprintf(fid,"        ];\n");
    fprintf(fid,"end\n");
    fclose(fid);

    fid = fopen(dir_name + "/" + dir_name + "ForcingVector.m","w");
    fprintf(fid,"function f = %sDAEForcingVector(x,u,p)\n",name);
    fprintf(fid,"    fk = " + name + "KinematicForcingVector(x(1:" + nq + "),x(" + (nq + 1) + ":end),p);\n");
    for k = 1:nb
        fprintf(fid,"    fd" + k + " = " + name + "Body" + k + "DynamicsForcingVector(x,u,p);\n");
    end
    fprintf(fid,"    fd = fd1");
    if nb > 1
        for k = 2:nb
            fprintf(fid," + fd" + k);
        end
    end
    fprintf(fid,";\n");
    fprintf(fid,"    fc = " + name + "ConstraintForcingVector(x(1:" + nq + "),x(" + (nq + 1) + ":end),p);\n");
    fprintf(fid,"    f = [\n");
    fprintf(fid,"        fk;\n");
    fprintf(fid,"        fd;\n");
    fprintf(fid,"        fc\n");
    fprintf(fid,"        ];\n");
    fprintf(fid,"end\n");
    fclose(fid);
end