%% Choose S and v so that MPC_TS_SC yields same result as MPC_TS
% Get problem parameters
params = generate_params();
Q = diag(params.exercise.QdiagOptA);
R = eye(params.model.nu);
N = params.model.HorizonLength;
x0 = params.model.InitialConditionA;
[H,h] = lqr_maxPI(Q,R,params);

% Define penalty function parameters
s = 100;
S = eye(params.model.nx)*s;
v = 1e6;

save('templates/MPS_TS_SC_params','S','v');

ctrl_ts = MPC_TS(Q,R,N,H,h,params);
ctrl_ts_sc = MPC_TS_SC(Q,R,N,H,h,S,v,params);

[Xt_ts,Ut_ts,u_info_ts] = simulate(x0, ctrl_ts, params);
[Xt_sc,Ut_sc,u_info_sc] = simulate(x0, ctrl_ts_sc, params);

figure(1);
title('Control input');
plot(Ut_ts(1,:),Ut_ts(2,:),'r');
hold on;
plot(Ut_sc(1,:),Ut_sc(2,:),'b');
legend('Normal MPC', 'Soft constraints');

figure(2);
plot(Xt_ts(1,:),Xt_ts(2,:),'r');
hold on;
plot(Xt_sc(1,:),Xt_sc(2,:),'b');
legend('Normal MPC', 'Soft constraints');
