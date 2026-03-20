close("all"); clear; clc;
setmadsympath();

dir="C:\Users\Public\Documents\BikeSim2017.1_Data\Generic\";
R0_front=0.05;
R0_rear=0.07;

fig_front=pacejkaAligningMomentPlot(dir+"Example_Front_v102.par","Front tire",R0_front);
set(fig_front,"Position",[100,480,1280,240]);

fig_rear=pacejkaAligningMomentPlot(dir+"Example_Rear_v102.par","Rear tire",R0_rear);

%% Plotting functions

function fig=pacejkaAligningMomentPlot(file_name,title_str,R0)
    pars=readPacejka(file_name);
    m=matlabColors();
    
    ns=1000;
    slip=deg2rad(linspace(-10,10,ns));

    camber_max=50;
    camber=deg2rad(linspace(-camber_max,camber_max,ns));
    fz=linspace(0,1.5E03,ns);
    
    fig=figure('Position',[100,100,1280,240]);
    
    set(fig,'DefaultAxesFontSize',12);
    set(fig,'DefaultTextFontSize',12);
    
    t=tiledlayout(fig,1,3,'TileSpacing','compact','Padding','compact');
    sgtitle(title_str+" aligning moment: Magic formula","FontSize",12);
    
    % -------- 1) Slip vs camber --------
    [X,Y]=meshgrid(slip,camber);
    Z=pacejkaAligningMoment(X,Y,fz(end),pars,R0);
    
    axe=nexttile(t,1);
    surf(axe,rad2deg(X),rad2deg(Y),Z,...
        'EdgeColor','none',...
        'FaceColor',m.blue,...
        'FaceLighting','gouraud');
    
    view(axe,225,45);
    box(axe,'on'); grid(axe,'on'); axis(axe,'tight');
    title(axe,"Slip vs camber vs aligning moment (Z = "+fz(end)/1E03+" kN)",'FontSize',12);
    xlabel(axe,'Slip angle (deg)','FontSize',12);
    ylabel(axe,'Camber (deg)','FontSize',12);
    zlabel(axe,'Moment (N m)','FontSize',12);
    camlight(axe,270,45,"local");
    camproj(axe,'perspective');
    lims=max(abs(Z(:,ns/2))).*[-1,1];
    
    % -------- 2) Slip vs vertical force --------
    mz=pacejkaAligningMoment(slip,0*camber,fz.',pars,R0);
    mzl=pacejkaAligningMoment(slip,0*camber,fz(end),pars,R0,"Type","Linearized");
    
    X=rad2deg(repmat(slip,ns,1));
    Y=mz;
    Z=repmat(fz,ns,1)./1E03;
    
    axe=nexttile(t,2);
    hold(axe,'on');
    surf(axe,X.',Y.',Z,'EdgeColor','none');
    plot3(axe,rad2deg(slip),mzl,fz(end)*ones(ns,1)./1E03,...
        'Color',m.orange,...
        'LineWidth',2);
    hold(axe,'off');
    view(axe,0,90);
    box(axe,'on'); axis(axe,'tight'); grid(axe,'on');
    title(axe,'Slip vs normal force vs aligning moment','FontSize',12);
    xlabel(axe,'Slip angle (deg)','FontSize',12);
    ylim(axe,lims);
    
    % -------- 3) Camber vs vertical force --------
    mz=pacejkaAligningMoment(0*slip,camber,fz.',pars,R0); 
    mzl=pacejkaAligningMoment(0*slip,camber,fz(end),pars,R0,"Type","Linearized");

    X=rad2deg(repmat(camber,ns,1));
    Y=mz;
    Z=repmat(fz,ns,1)./1E03;
    
    axe=nexttile(t,3);
    hold(axe,'on');
    surf(axe,X.',Y.',Z,'EdgeColor','none',"DisplayName","nonlinear");
    plot3(axe,rad2deg(camber),mzl,fz(end)*ones(ns,1)./1E03,...
        'Color',m.orange,...
        'LineWidth',2,...
        "DisplayName","linearized (Z ="+fz(end)/1E03+"kN)");
    hold(axe,'off');
    view(axe,0,90);
    box(axe,'on'); axis(axe,'tight'); grid(axe,'on');
    title(axe,'Camber vs normal force vs aligning moment','FontSize',12);
    xlabel(axe,'Camber angle (deg)','FontSize',12);
    xticks(axe,-camber_max:camber_max/2:camber_max);
    % ylim(axe,lims);
    
    cb=colorbar(axe);
    cb.FontSize=12;
    cb.Label.String="Normal force (kN)";
    cb.Label.FontSize=12;

    legend("show","FontSize",10,"Location","NorthEast","Orientation","vertical");
end

%% Helper functions

function mz=pacejkaAligningMoment(slip,camber,fz,pars,R0,options)

    arguments
        slip double
        camber double
        fz double
        pars struct
        R0 double {mustBePositive}
        options.Type string {mustBeMember(options.Type,["Full","Linearized"])}="Full"
    end

    epsz=1e-12;

    fz0=getPar(pars,'P_BIKE_FZ0',0);
    if abs(fz0)<epsz
        eID='pacejkaAligningMoment:InvalidFZ0';
        error(eID,'P_BIKE_FZ0 is missing or zero in the parameter struct.');
    end
    dfz=(fz-fz0)./fz0;

    % Lateral-force parameters needed for Fy and linearized stiffness
    pCy1=getPar(pars,'P_BIKE_PCY1',0);
    pDy1=getPar(pars,'P_BIKE_PDY1',0);
    pDy2=getPar(pars,'P_BIKE_PDY2',0);
    pDy3=getPar(pars,'P_BIKE_PDY3',0);
    pKy1=getPar(pars,'P_BIKE_PKY1',0);
    pKy2=getPar(pars,'P_BIKE_PKY2',0);
    pKy3=getPar(pars,'P_BIKE_PKY3',0);
    pKy4=getPar(pars,'P_BIKE_PKY4',0);
    pKy5=getPar(pars,'P_BIKE_PKY5',0);
    pKy6=getPar(pars,'P_BIKE_PKY6',0);
    pKy7=getPar(pars,'P_BIKE_PKY7',0);

    % Aligning-moment parameters from the shown equations
    qBz1=getPar(pars,'P_BIKE_QBZ1',0);
    qBz2=getPar(pars,'P_BIKE_QBZ2',0);
    qBz5=getPar(pars,'P_BIKE_QBZ5',0);
    qBz6=getPar(pars,'P_BIKE_QBZ6',0);
    qBz9=getPar(pars,'P_BIKE_QBZ9',0);
    qBz10=getPar(pars,'P_BIKE_QBZ10',0);

    qCz1=getPar(pars,'P_BIKE_QCZ1',0);

    qDz1=getPar(pars,'P_BIKE_QDZ1',0);
    qDz2=getPar(pars,'P_BIKE_QDZ2',0);
    qDz3=getPar(pars,'P_BIKE_QDZ3',0);
    qDz4=getPar(pars,'P_BIKE_QDZ4',0);
    qDz8=getPar(pars,'P_BIKE_QDZ8',0);
    qDz9=getPar(pars,'P_BIKE_QDZ9',0);
    qDz10=getPar(pars,'P_BIKE_QDZ10',0);
    qDz11=getPar(pars,'P_BIKE_QDZ11',0);

    qEz1=getPar(pars,'P_BIKE_QEZ1',0);
    qEz2=getPar(pars,'P_BIKE_QEZ2',0);
    qEz5=getPar(pars,'P_BIKE_QEZ5',0);

    qHz3=getPar(pars,'P_BIKE_QHZ3',0);
    qHz4=getPar(pars,'P_BIKE_QHZ4',0);

    % -------- Side-force terms needed by aligning moment --------
    beta=slip;
    gamma=camber;

    Cy=pCy1;

    mu_y=pDy1.*exp(pDy2.*dfz)./(1+pDy3.*gamma.^2);
    Dy=mu_y.*fz;

    denomKya=(pKy3+pKy4.*gamma.^2).*fz0;
    atanKya=atan(fz./(denomKya+epsz));
    Kya0=pKy1.*fz0.*sin(pKy2.*atanKya);
    Kya=Kya0./(1+pKy5.*gamma.^2);

    By=Kya./(Cy.*Dy+epsz);
    Kya00=pKy1.*fz0.*sin(pKy2.*atan(fz./(pKy3.*fz0+epsz)));
    Fy0_gamma0 = pacejkaSideForce(beta,0,fz,pars);

    % Camber stiffness term for the unchanged linearized version
    Kyg=(pKy6+pKy7.*dfz).*fz;

    % -------- Aligning moment, consistent with shown equations --------
    SHr=(qHz3+qHz4.*dfz).*gamma;

    Bt=(qBz1+qBz2.*dfz).*(1+qBz5.*abs(gamma)+qBz6.*gamma.^2);
    Ct=qCz1;
    Dt=fz.*(R0./fz0).*(qDz1+qDz2.*dfz).*(1+qDz3.*abs(gamma)+qDz4.*gamma.^2);

    atan_BtCtBeta=atan(Bt.*Ct.*beta);
    Et=(qEz1+qEz2.*dfz).*(1+qEz5.*gamma.*(2/pi).*atan_BtCtBeta);

    Br=qBz9+qBz10.*By.*Cy;
    Dr=fz.*R0.*(((qDz8+qDz9.*dfz).*gamma)+((qDz10+qDz11.*dfz).*gamma.*abs(gamma)))./sqrt(1+beta.^2);

    Bt_beta=Bt.*beta;
    atan_Bt_beta=atan(Bt_beta);
    trail_inner=Bt_beta-Et.*(Bt_beta-atan_Bt_beta);
    trail_cos_arg=Ct.*atan(trail_inner);

    Mzt0=-Dt.*cos(trail_cos_arg).*Fy0_gamma0./sqrt(1+beta.^2);

    alpha_r=beta+SHr;
    Mzr0=Dr.*cos(atan(Br.*alpha_r));

    mz=Mzt0+Mzr0;

    % -------- Linearized version unchanged --------
    if strcmp(options.Type,"Linearized")
        mz=(-Dt.*Kya00.*beta)+Kyg.*R0.*gamma;
    end
end

function fy=pacejkaSideForce(slip,camber,fz,pars,options)

    arguments
        slip double
        camber double
        fz double
        pars struct
        options.Type string {mustBeMember(options.Type,["Full","Linearized"])}="Full"
    end

    epsy=1e-12;

    fz0=getPar(pars,'P_BIKE_FZ0',0);
    if abs(fz0)<epsy
        eID='pacejkaLateralForce:InvalidFZ0';
        error(eID,'P_BIKE_FZ0 is missing or zero in the parameter struct.');
    end
    dfz=(fz-fz0)./fz0;

    pCy1=getPar(pars,'P_BIKE_PCY1',0);
    pCy2=getPar(pars,'P_BIKE_PCY2',0);

    pDy1=getPar(pars,'P_BIKE_PDY1',0);
    pDy2=getPar(pars,'P_BIKE_PDY2',0);
    pDy3=getPar(pars,'P_BIKE_PDY3',0);

    pEy1=getPar(pars,'P_BIKE_PEY1',0);
    pEy2=getPar(pars,'P_BIKE_PEY2',0);
    pEy3=getPar(pars,'P_BIKE_PEY3',0);
    pEy4=getPar(pars,'P_BIKE_PEY4',0);
    pEy5=getPar(pars,'P_BIKE_PEY5',0);

    pKy1=getPar(pars,'P_BIKE_PKY1',0);
    pKy2=getPar(pars,'P_BIKE_PKY2',0);
    pKy3=getPar(pars,'P_BIKE_PKY3',0);
    pKy4=getPar(pars,'P_BIKE_PKY4',0);
    pKy5=getPar(pars,'P_BIKE_PKY5',0);
    pKy6=getPar(pars,'P_BIKE_PKY6',0);
    pKy7=getPar(pars,'P_BIKE_PKY7',0);

    SHy=0;
    alpha_y=slip+SHy;

    Cy=pCy1;

    mu_y=pDy1.*exp(pDy2.*dfz)./(1+pDy3.*camber.^2);
    Dy=mu_y.*fz;

    Ey=pEy1+pEy2.*camber.^2+(pEy3+pEy4.*camber).*sign(alpha_y);

    denomKya0=(pKy3+pKy4.*camber.^2).*fz0;
    atan_term=atan(fz./(denomKya0+epsy));
    Kya0=pKy1.*fz0.*sin(pKy2.*atan_term);
    Kya=Kya0./(1+pKy5.*camber.^2);

    By=Kya./(Cy.*Dy+epsy);

    Cgamma=pCy2;
    Kyg=(pKy6+pKy7.*dfz).*fz;
    Eg=pEy5;
    Bgamma=Kyg./(Cgamma.*Dy+epsy);

    if strcmp(options.Type,"Linearized")
        fy=(Kya.*alpha_y+Kyg.*camber);
        return
    end

    By_alpha=By.*alpha_y;
    atan_By_alpha=atan(By_alpha);
    Fy_alpha_inner=By_alpha-Ey.*(By_alpha-atan_By_alpha);
    Fy_alpha=Dy.*sin(Cy.*atan(Fy_alpha_inner));

    Bgamma_camber=Bgamma.*camber;
    atan_Bgamma_camber=atan(Bgamma_camber);
    Fy_gamma_inner=Bgamma_camber-Eg.*(Bgamma_camber-atan_Bgamma_camber);
    Fy_gamma=Dy.*sin(Cgamma.*atan(Fy_gamma_inner));

    fy=(Fy_alpha+Fy_gamma);
end

function value=getPar(pars,name,defaultValue)
    if isfield(pars,name)
        value=pars.(name);
    else
        value=defaultValue;
    end
end

function pars=readPacejka(filename)
   arguments
        filename (1,1) string
    end

    if ~isfile(filename)
        eID='readPacBikeParFile:FileNotFound';
        error(eID,'File not found: %s',filename);
    end

    fid=fopen(filename,'r');
    if fid==-1
        eID='readPacBikeParFile:OpenFailed';
        error(eID,'Could not open file: %s',filename);
    end

    cleaner=onCleanup(@() fclose(fid));

    pars=struct();
    inBlock=false;

    while ~feof(fid)
        line=fgetl(fid);

        if ~ischar(line)
            continue
        end

        line=strtrim(line);

        if line==""
            continue
        end

        if startsWith(line,'#')||startsWith(line,'%')||startsWith(line,'//')
            continue
        end

        if strcmp(line,'BEGIN_PAC_BIKE_DATA')
            inBlock=true;
            continue
        elseif strcmp(line,'END_PAC_BIKE_DATA')
            inBlock=false;
            continue
        elseif strcmp(line,'Parsfile')||strcmp(line,'END')
            continue
        end

        if ~inBlock
            continue
        end

        expr='^([A-Za-z]\w*)\s+([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)$';
        tokens=regexp(line,expr,'tokens','once');

        if isempty(tokens)
            continue
        end

        name=tokens{1};
        value=str2double(tokens{2});

        if isnan(value)
            eID='readPacBikeParFile:InvalidValue';
            warning(eID,'Skipping line with invalid numeric value: %s',line);
            continue
        end

        fieldName=matlab.lang.makeValidName(name);
        pars.(fieldName)=value;
    end
end