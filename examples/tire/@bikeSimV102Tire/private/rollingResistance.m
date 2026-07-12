function My = rollingResistance(obj,w,fz)
    VlowRR = 1/3.6;
    if (w.*obj.RRE > VlowRR)
        Sr = 1;
    elseif (w.*obj.RRE < -VlowRR)
        Sr = -1;
    else
        Sr = -cos((VlowRR - w.*obj.RRE).*(pi/2)./VlowRR);
    end
    My = fz.*obj.RRE.*(obj.RR_C + obj.RR_V.*abs(w.*obj.RRE)).*Sr;
end