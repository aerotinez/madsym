function Mx = overturningMoment(obj,g,Fx,Fy,Fz,My,Mz)
    Mx = zeros(size(g));
    N = eye(3);
    nz = N(:,3);
    CGC = zeros(3,1);
    C = CGC + obj.RAD_TC.*nz;
  
    for i = 1:numel(g)
        R = rotx(rad2deg(g(i)));

        W = [
            0;
            0;
            Mz(i);
            Fx(i) - My(i)./obj.RRE;
            Fy(i);
            Fz(i);
            ];

        CTC = C - (obj.RAD_TC)./cos(g(i)).*R(:,3);

        pm = [
            0,-CTC(3),CTC(2);
            CTC(3),0,-CTC(1);
            -CTC(2),CTC(1),0
            ];

        Ad = [
            R,zeros(3);
            pm*R,R
            ];

        Wb = Ad.'*W;

        Mx(i) = Wb(1);
    end
end