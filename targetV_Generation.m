function [v acc]=targetV_Generation(v_T, v_p, S, t)
ACC=0.22; %constance acceleration/deceleration 0.22m/s^2 
if v_T<0
    if v_p~=v_T
        S1=abs(v_p^2-v_T^2)/(2*ACC);
        S3=v_T^2/(2*ACC);
        if S(1)-S1-S3>0
            acc=-ACC;
            v=v_p+acc*t;
            if v<v_T
                v=v_T;
            end
        else
            if S(1)>(v_p^2/2+0.1)
                acc=-ACC;
                v=v_p+acc*t;
            else
                acc=ACC;
                v=v_p+acc*t;
                if v>0
                    v=0;
                end
            end
        end
    else
        S3=v_T^2/(2*ACC);
        if S(1)-S3>0
            v=v_T;
            acc=0;
        else
            acc=ACC;
            v=v_p+acc*t;
        end
    end
elseif v_T>0
    if v_p~=v_T
        S1=abs(v_p^2-v_T^2)/(2*ACC);
        S3=v_T^2/(2*ACC);
        if S(2)-S1-S3>0
            acc=ACC;
            v=v_p+acc*t;
            if v>v_T
                v=v_T;
            end
        else
            if S(2)>(v_p^2/2+0.1)
                acc=ACC;
                v=v_p+acc*t;
            else
                acc=-ACC;
                v=v_p+acc*t;
            end
        end
    else
        S3=v_T^2/(2*ACC);
        if S(2)-S3>0
            v=v_T;
            acc=0;
        else
            acc=-ACC;
            v=v_p+acc*t;
        end
    end
else
    v=0;
    acc=0;
end
end