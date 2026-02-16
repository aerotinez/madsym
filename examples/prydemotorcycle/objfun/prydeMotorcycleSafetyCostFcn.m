function Q = prydeMotorcycleSafetyCostFcn(params)
    arguments
        params struct;
    end

    p = cell2mat(struct2cell(params));
    idx = [1,4:9];
    
    %% Slip angle difference cost
    Ja = slipAngleJacobian(p);
    Jar = Ja(1,:);
    Jaf = Ja(2,:);
    Jda = Jar - Jaf;
    Qda = Jda'*Jda;
    Qda = Qda(idx,idx);
    
    %% Force ellipse cost
    
    % Rear tire coefficients
    rt = rearTireEqns(p);
    fzr = rt(1);
    Kkr = rt(2);
    Kar = rt(3);
    Kgr = rt(4);
    
    dfzr = (fzr - params.fzr0)/params.fzr0;
    Dxr = (params.pKxr1 + params.pKxr2*dfzr)*fzr;
    Dyr = fzr*params.pKyr1*exp(params.pKyr2*dfzr);
    
    % Front tire coefficients
    ft = frontTireEqns(p);
    fzf = ft(1);
    Kkf = ft(2);
    Kaf = ft(3);
    Kgf = ft(4);
    
    dfzf = (fzf - params.fzf0)/params.fzf0;
    Dxf = (params.pKxf1 + params.pKxf2*dfzf)*fzf;
    Dyf = fzf*params.pKyf1*exp(params.pKyf2*dfzf);
    
    % Slip ratio
    Jk = slipRatioJacobian(p);
    Jkr = Jk(1,:);
    Jkf = Jk(2,:);
    
    % Camber
    Jg = camberJacobian(p);
    Jgr = Jg(1,:);
    Jgf = Jg(2,:);
    
    % Force jacobians
    Jxr = Kkr*Jkr/Dxr;
    Jxf = Kkf*Jkf/Dxf;
    
    Jyr = [Kar*Jar;Kgr*Jgr]/Dyr;
    Jyf = [Kaf*Jaf;Kgf*Jgf]/Dyf;
    
    % Cost
    Qr = Jxr'*Jxr + Jyr'*Jyr;
    Qf = Jxf'*Jxf + Jyf'*Jyf;
    
    Qt = Qr + Qf;
    Qt = Qt(idx,idx);
    
    %% Rate costs
    sys = prydeMotorcycleStateSpace(p);
    A = sys.E\sys.A;
    
    % Yaw
    Jwz = A(6,:);
    Qwz = Jwz'*Jwz;
    Qwz = Qwz(idx,idx);
    
    % Steer
    Js = A(end,:);
    Qs = Js'*Js;
    Qs = Qs(idx,idx);
    
    %% Total cost
    Q = Qda + Qt + 1E-05*Qwz + 1E-07*Qs + diag([zeros(1,5),1,1]);

end
