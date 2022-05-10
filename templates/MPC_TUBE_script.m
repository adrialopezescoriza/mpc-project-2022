%% MPC TUBE tuner script
% Predefined parameters
N = 50;
R = 1;
Q = diag([0.1,0.1]);
p = [0.1 0.5];
params = generate_params();
params_z = generate_params_z(params);

% Get stabilizing controller
K_tube = compute_tube_controller(p,params_z);

% Get terminal constraint
[H_N, h_N] = lqr_maxPI(Q,R,params_z);

% Get tubular constriants
[H_tube,h_tube,n_iter] = compute_minRPI(K_tube,params_z);
params_z_tube = compute_tightening(K_tube,H_tube,h_tube,params_z);

% Save relevant parameters
save('templates/MPC_TUBE_params','p','K_tube','H_tube','h_tube','H_N','h_N','params_z_tube');
clear;
