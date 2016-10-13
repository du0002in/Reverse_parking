function [phai_t omega]=targetPhai_Generation(phai_T, phai_t, t)
OMEGA=2.0; %radius per second
if phai_T>0
    if phai_t~=phai_T
        omega=OMEGA;
        phai_t=phai_t+omega*t;
        if phai_t>phai_T
            phai_t=phai_T;
        end
    else
        phai_t=phai_T;
        omega=0;
    end
elseif phai_T<0
    if phai_t~=phai_T
        omega=-OMEGA;
        phai_t=phai_t+omega*t;
        if phai_t<phai_T
            phai_t=phai_T;
        end
    else
        phai_t=phai_T;
        omega=0;
    end
else
    if phai_t~=phai_T
        if phai_t>0
            omega=-OMEGA;
            phai_t=phai_t+omega*t;
            if phai_t<0
                phai_t=phai_T;
            end
        else
            omega=OMEGA;
            phai_t=phai_t+omega*t;
            if phai_t>0
                phai_t=phai_T;
            end
        end
    else
        phai_t=phai_T;
        omega=0;
    end
end
end