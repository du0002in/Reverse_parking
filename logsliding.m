%plot all signal related to sliding controller
% % % k_s=abs(v_e)/L_wheels*2.3;
% % % theta_error=theta_e-theta_d;
% % % y_error=-sin(theta_d)*(x_e-xd)+cos(theta_d)*(z_e-zd);
% % % ydot_error=v_e*sin(theta_error);
% % % s=ydot_error+k_s*y_error+k0_s*sign(y_error)*sign(theta_error)*(theta_error);
% % % 
% % % phai_t=atan(1/v_e*(L_car*(-Q_s*atan(s/P_s)-k_s*v_e*sin(theta_error))/(v_e*cos(theta_error)+k0_s*sign(theta_error)*sign(y_error))+v_e*tan(phai_T(1))));
% % % phai_t=atan(-Q_s*(exp(exp_n*abs(s))-1)*atan(s/P_s)+tan(phai_T(1)));
% % % phai_t=atan(-Q_s*atan(s/P_s)-k_s*L_wheels/v_e*tan(theta_error)+tan(phai_T(1)));

figure; plot(-Q_s*atan(S_debug/P_s));hold on;
plot(-3*sign(X_e(7,:)).*tan(THETA_error),'r');
plot(phai_t_s-phai_T,'g');
legend('s related','\theta related','\phi delta');