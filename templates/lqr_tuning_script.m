clear;
clc;
% LQR tuning script

% Get params and initial condition
params = generate_params();
 
% Performs grid search with q parameters in log-space
q_max_10 = 3;
q_min_10 = -3;
x0_A = [-15e-3; -400e-3; 24.4e-3; 0; 0.0081; 0];
N = 40;

q_vals = logspace(q_min_10,q_max_10,N);
Q_grid_pos = nmultichoosek(q_vals,params.model.nx-2)';
Q_grid_pos = [Q_grid_pos; zeros(2,size(Q_grid_pos,2))];

[tuning_struct, i_opt] = lqr_tuning(x0_A,Q_grid_pos,params);
q_opt_pos = tuning_struct(i_opt).Qdiag;

Q_grid_vel = nmultichoosek(q_vals,2)';
Q_grid_vel = [repmat(q_opt_pos(1:end-2,:),1,size(Q_grid_vel,2)); Q_grid_vel];

[tuning_struct, i_opt] = lqr_tuning(x0_A,Q_grid_vel,params);

q_opt = nan;
J_opt = tuning_struct(i_opt).InputCost;
if J_opt < 11
    q_opt = tuning_struct(i_opt).Qdiag;
end

save('templates/lqr_tuning_script',"q_opt");


function combs = nmultichoosek(values, k)
    % Return number of multisubsets or actual multisubsets.
    if numel(values)==1 
        n = values;
        combs = nchoosek(n+k-1,k);
    else
    n = numel(values);
    combs = bsxfun(@minus, nchoosek(1:n+k-1,k), 0:k-1);
    combs = reshape(values(combs),[],k);
    end
end