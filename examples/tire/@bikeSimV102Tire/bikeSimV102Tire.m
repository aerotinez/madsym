classdef bikeSimV102Tire < handle
    properties (GetAccess = public, SetAccess = private)
        SET_THICKNESS_SGUI
        RRE
        KT
        FZ_MAX
        RAD_TC
        M_WHL
        IYY_WHL
        L_RELAX_X
        L_RELAX_Y
        VLOW_ALPHA
        RR_C
        RR_V
        P_BIKE_FZ0
        P_BIKE_PCX1 
        P_BIKE_PDX1
        P_BIKE_PDX2
        P_BIKE_PEX1
        P_BIKE_PEX2
        P_BIKE_PEX3
        P_BIKE_PKX1
        P_BIKE_PKX2
        P_BIKE_PKX3
        P_BIKE_RBX1
        P_BIKE_RBX2
        P_BIKE_RCX1
        P_BIKE_PCY1
        P_BIKE_PCY2
        P_BIKE_PDY1   
        P_BIKE_PDY2   
        P_BIKE_PDY3   
        P_BIKE_PEY1
        P_BIKE_PEY2
        P_BIKE_PEY4
        P_BIKE_PEY5
        P_BIKE_PKY1
        P_BIKE_PKY2
        P_BIKE_PKY3
        P_BIKE_PKY4
        P_BIKE_PKY5
        P_BIKE_PKY6
        P_BIKE_PKY7
        P_BIKE_RBY1
        P_BIKE_RBY2
        P_BIKE_RBY3
        P_BIKE_RCY1
        P_BIKE_QBZ1
        P_BIKE_QBZ2
        P_BIKE_QBZ5
        P_BIKE_QBZ6
        P_BIKE_QBZ9
        P_BIKE_QBZ10
        P_BIKE_QCZ1
        P_BIKE_QDZ1
        P_BIKE_QDZ2
        P_BIKE_QDZ3
        P_BIKE_QDZ4
        P_BIKE_QDZ8
        P_BIKE_QDZ9
        P_BIKE_QDZ10
        P_BIKE_QDZ11
        P_BIKE_QEZ1
        P_BIKE_QEZ2
        P_BIKE_QEZ5
        P_BIKE_QHZ1
        P_BIKE_QHZ2
        P_BIKE_QHZ3
        P_BIKE_QHZ4
        P_BIKE_MU_REF_X
        P_BIKE_MU_REF_Y
    end
    methods (Access = public)
        function obj = bikeSimV102Tire()

        end
        [Fx,Fy,Fz,Mx,My,Mz] = compute(obj,k,a,g,fz)
        export(obj)
        import(obj,par,pac)
        plot(obj,options)
        struct(obj)
        table(obj)
    end
    methods (Access = private)
        importPar(obj,par)
        importPac(obj,par)
        dfz = fzRatio(obj,fz)
        fx0 = fxPureSlip(obj,k,fz,dfz)
        fx = fxCombinedSlip(obj,fx0,a)
        fy0 = fyPureSlip(obj,a,g,fz,dfz)
        fy = fyCombinedSlip(obj,fy0,k)
        tx = overturningMoment(obj,fz)
        ty = rollingResistance(obj,k,a)
        tz0 = aligningMomentPureSlip(obj,a,g,fz,dfz)
        tz = aligningMomentCombinedSlip(obj,k)
    end
end