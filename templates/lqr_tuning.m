%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [tuning_struct, i_opt] = lqr_tuning(x0,Q,params)
    i = 1;
    i_opt = nan;
    J_opt = inf;

    for q=Q
        ctrl = LQR(diag(q),eye(params.model.nu),params);
        [Xt,Ut] = simulate(x0, ctrl, params);
        [s_max, y_max, u_max, J_u, df_max, vf_max, traj_feas]... 
            = traj_constraints(Xt,Ut,params);

        if J_u < J_opt && traj_feas
            i_opt = i;
            J_opt = J_u;
        end
        tuning_struct(i,:) = struct(...
            "InitialCondition", x0,...
            "Qdiag", q,...
            "MaxAbsPositionXZ", s_max,...
            "MaxAbsPositionY", y_max,...
            "MaxAbsThrust", u_max,...
            "InputCost", J_u,...
            "MaxFinalPosDiff", df_max,...
            "MaxFinalVelDiff", vf_max,...
            "TrajFeasible", traj_feas);

        i = i+1;
    end

    
end